// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file sparrow_planner.hpp
 * \brief SPARROW (POMCP) planner core: C++ port of vtr_obstacle_simulation's
 *        vtr3_sim/pomcp/ {types, calendar, state, model, belief, generator,
 *        solver, rollouts (shortest_path)}.
 *
 * HSHMAT: This is deliberately ROS-free and pose-graph-free. Vertices are
 * plain uint64 ids and edges are canonical (min,max) pairs, so the core can
 * be unit-tested offline and reused by the pybind debug module. The
 * SparrowStrategy adapter (sparrow_strategy.hpp) converts between
 * tactic::VertexId/EdgeId and these ids.
 *
 * Faithful-port notes (kept in sync with the Python source):
 *  - Undiscounted cost-minimizing POMCP; UCB is a lower-confidence score
 *    Q - c_uct * sqrt(log N / N_a); robust-child (most visits) at the root.
 *  - Fresh age-conditioned particles every decision (no persistent filter).
 *  - One absolute-time obstacle calendar (Poisson spawns + clearances) per
 *    particle; chunking-invariant advance_to().
 *  - Corridor-collapsed Traverse actions (degree-2 chains).
 *  - KM residual sampling via inverse-CDF on the shared SurvivalModel,
 *    Exp(60 s) fallback for classes with no data (OSCAR parity).
 */
#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vtr {
namespace navigation {

class SurvivalModel;  // vtr_navigation/survival_model.hpp

namespace sparrow {

using SVertex = uint64_t;
using SEdge = std::pair<uint64_t, uint64_t>;  // canonical: first <= second

inline SEdge canonical_edge(SVertex a, SVertex b) {
  return (a <= b) ? SEdge{a, b} : SEdge{b, a};
}

constexpr double kInf = std::numeric_limits<double>::infinity();

// ---------------------------------------------------------------------------
// RNG (cloneable stream; ports rng.FastRNG semantics we rely on)
// ---------------------------------------------------------------------------
class Rng {
 public:
  explicit Rng(uint64_t seed = 0x5EED) : gen_(seed) {}
  double uniform() { return uni_(gen_); }               // [0,1)
  double exponential(double mean) {
    if (mean <= 0.0) return 0.0;
    double u = std::max(1e-12, 1.0 - uniform());
    return -mean * std::log(u);
  }
  uint64_t integer(uint64_t n) {                        // [0, n)
    return n ? gen_() % n : 0;
  }
  Rng clone() const { return *this; }                   // copies state
  uint64_t raw() { return gen_(); }

 private:
  std::mt19937_64 gen_;
  std::uniform_real_distribution<double> uni_{0.0, 1.0};
};

/// Deterministic seed derivation (splitmix-style), mirrors rng.derive_seed.
inline uint64_t derive_seed(uint64_t base, uint64_t stream) {
  uint64_t z = base + 0x9E3779B97F4A7C15ULL * (stream + 1);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

// ---------------------------------------------------------------------------
// Frozen model snapshot (ports model.FrozenLearnedModel)
// ---------------------------------------------------------------------------
struct SparrowModel {
  std::vector<std::string> class_names;
  std::vector<double> class_probs;       // aligned with class_names
  double p_block = 0.0;                  // steady-state edge occupancy
  double spawn_rate_hz = 0.0;            // Little's law: p_block*E/E[R]
  int num_edges = 1;                     // blockable edge count
  double prior_residual_mean = 60.0;     // Exp fallback (KM_ZERO_DATA_MEAN_S)
  const SurvivalModel* km = nullptr;     // shared KM fits (frozen per call)
  std::set<std::string> fitted_classes;  // classes with >=1 KM sample

  bool is_fitted(const std::string& t) const {
    return fitted_classes.count(t) > 0;
  }

  /// S_k(t) with exponential-prior fallback for unfitted classes.
  double survival(const std::string& obs_type, double t) const;
  /// P(R > u | D > elapsed, class) = S(elapsed+u)/S(elapsed).
  double residual_survival(const std::string& obs_type, double u,
                           double elapsed) const;
  /// E[R | D > elapsed, class].
  double expected_residual(const std::string& obs_type, double elapsed) const;
  /// E[R | D > elapsed] marginalized over the age posterior.
  double expected_residual_mixture(double elapsed) const;
  /// p(k | D > c) proportional to p(k) S_k(c).
  std::map<std::string, double> class_posterior_given_age(double elapsed) const;

  std::string sample_class(Rng& rng) const;
  std::string sample_class_given_age(Rng& rng, double elapsed) const;
  /// Draw R ~ P(R > u | D > elapsed, class) via inverse-CDF bisection.
  double sample_residual(const std::string& obs_type, Rng& rng,
                         double elapsed) const;
  /// Duration of a newly spawned obstacle in rollouts (residual @ 0).
  double sample_duration(const std::string& obs_type, Rng& rng) const {
    return sample_residual(obs_type, rng, 0.0);
  }
};

// ---------------------------------------------------------------------------
// Graph context (ports state.GraphContext)
// ---------------------------------------------------------------------------
class GraphContext {
 public:
  GraphContext() = default;
  GraphContext(std::map<SVertex, std::vector<SVertex>> neighbors,
               std::map<SEdge, double> travel_time, SVertex goal);

  SVertex goal = 0;
  std::map<SVertex, std::vector<SVertex>> neighbors;   // sorted, deduped
  std::map<SEdge, double> travel_time;                 // canonical edges
  std::map<SVertex, std::vector<SEdge>> incident_edges;
  std::vector<SEdge> edges;                            // sorted canonical
  std::map<SVertex, double> time_to_goal;              // obstacle-free lower bound

  double edge_time(SVertex u, SVertex v) const;
  double heuristic_cost_to_go(SVertex v) const;
  bool is_decision_vertex(SVertex v) const;
  /// Chain of vertices from `vertex` towards `first_hop` up to the next
  /// decision vertex (inclusive). Cached.
  const std::vector<SVertex>& corridor(SVertex vertex, SVertex first_hop) const;

 private:
  mutable std::map<std::pair<SVertex, SVertex>, std::vector<SVertex>>
      corridor_cache_;
};

// ---------------------------------------------------------------------------
// Absolute-time obstacle process (ports calendar.AbsoluteTimeObstacleProcess)
// ---------------------------------------------------------------------------
struct ActiveObs {
  int uid = 0;
  std::string obs_type;
  double t_spawn = 0.0;
  double t_clear = 0.0;
  double remaining(double t) const { return std::max(0.0, t_clear - t); }
};

class ObstacleProcess {
 public:
  ObstacleProcess() = default;
  ObstacleProcess(const std::vector<SEdge>* edges, double spawn_rate_hz,
                  const SparrowModel* model, Rng rng, double t0,
                  bool no_adjacent_blocking = false);

  double time() const { return t_; }
  const ActiveObs* obstacle_on(const SEdge& e) const;
  bool is_blocked(const SEdge& e) const { return active_.count(e) > 0; }
  double next_clearance_of(const SEdge& e) const;

  /// Process every event in (t, T] chronologically (clear before spawn).
  void advance_to(double T);

  /// Force an obstacle onto an edge (particle initialization / repair).
  const ActiveObs& install_obstacle(const SEdge& e, const std::string& type,
                                    double t_spawn, double t_clear,
                                    int uid = -1);
  void remove_obstacle(const SEdge& e) { active_.erase(e); }

  const std::map<SEdge, ActiveObs>& active() const { return active_; }
  Rng& rng() { return rng_; }

 private:
  void process_clearance();
  void process_spawn(double t_spawn);
  double draw_next_spawn(double t_from);

  const std::vector<SEdge>* edges_ = nullptr;  // shared topology (not owned)
  double spawn_rate_hz_ = 0.0;
  const SparrowModel* model_ = nullptr;        // shared sampler (not owned)
  bool no_adjacent_blocking_ = false;
  Rng rng_;
  double t_ = 0.0;
  std::map<SEdge, ActiveObs> active_;
  // min-heap of (t_clear, uid, edge)
  struct Pending {
    double t;
    int uid;
    SEdge edge;
    bool operator>(const Pending& o) const {
      if (t != o.t) return t > o.t;
      return uid > o.uid;
    }
  };
  std::priority_queue<Pending, std::vector<Pending>, std::greater<Pending>>
      pending_;
  double next_spawn_time_ = kInf;
  int next_uid_ = 1;
};

// ---------------------------------------------------------------------------
// Planner state (ports state.PlannerState)
// ---------------------------------------------------------------------------
struct PlannerState {
  SVertex robot_vertex = 0;
  ObstacleProcess process;                    // value semantics: copy = clone
  std::map<SEdge, int> classified;            // edge -> encounter uid labeled
  std::map<SEdge, std::pair<int, double>> seen;  // edge -> (uid, t_first_seen)

  double time() const { return process.time(); }
  bool is_blocked(const SEdge& e) const { return process.is_blocked(e); }
  bool is_classified(const SEdge& e) const;
  void mark_classified(const SEdge& e, int uid) { classified[e] = uid; }
  void record_sighting(const std::map<SEdge, bool>& statuses);
  std::map<SEdge, bool> adjacent_statuses(const GraphContext& ctx) const;
};

// ---------------------------------------------------------------------------
// Actions + observation key (ports types.py)
// ---------------------------------------------------------------------------
struct SAction {
  enum Kind { TRAVERSE = 0, MAXWAIT = 1, OBSERVE = 2 };
  Kind kind = TRAVERSE;
  SEdge edge{0, 0};      // canonical target edge
  SVertex first_hop = 0; // TRAVERSE only: neighbor to drive towards
  double W = 0.0;        // MAXWAIT only

  bool operator<(const SAction& o) const {
    if (kind != o.kind) return kind < o.kind;
    if (edge != o.edge) return edge < o.edge;
    if (first_hop != o.first_hop) return first_hop < o.first_hop;
    return W < o.W;
  }
  bool operator==(const SAction& o) const {
    return kind == o.kind && edge == o.edge && first_hop == o.first_hop &&
           W == o.W;
  }
  std::string str() const;
};

struct ObservationKey {
  SVertex current_vertex = 0;
  std::vector<std::pair<SEdge, bool>> adjacent_statuses;  // sorted
  int64_t duration_bin = 0;
  std::string class_result;  // empty = none

  bool operator<(const ObservationKey& o) const {
    if (current_vertex != o.current_vertex)
      return current_vertex < o.current_vertex;
    if (duration_bin != o.duration_bin) return duration_bin < o.duration_bin;
    if (class_result != o.class_result) return class_result < o.class_result;
    return adjacent_statuses < o.adjacent_statuses;
  }
};

std::vector<SAction> valid_actions(SVertex vertex,
                                   const std::map<SEdge, bool>& statuses,
                                   const std::vector<SVertex>& neighbors,
                                   const std::set<SEdge>& classified_edges,
                                   const std::vector<double>& wait_set,
                                   bool allow_observe,
                                   const std::map<std::string,
                                                  std::vector<double>>*
                                       wait_set_by_class = nullptr,
                                   const std::map<SEdge, std::string>*
                                       edge_classes = nullptr);

// ---------------------------------------------------------------------------
// Transition model / generator (ports generator.TransitionModel)
// ---------------------------------------------------------------------------
struct StepResult {
  PlannerState next;
  ObservationKey obs;
  double cost = 0.0;
};

class TransitionModel {
 public:
  TransitionModel(const GraphContext* ctx, double delta_obs,
                  std::vector<double> wait_set, bool allow_observe,
                  bool corridor_traversal, double duration_bin_width)
      : ctx_(ctx),
        delta_obs_(delta_obs),
        wait_set_(std::move(wait_set)),
        allow_observe_(allow_observe),
        corridor_traversal_(corridor_traversal),
        bin_width_(duration_bin_width) {}

  /**
   * \brief Finer per-class wait budgets (handoff sec. 1.8).
   *
   * Once the robot has PAID to Observe an edge, the class is known and its
   * clearance law with it - so offer a grid resolved around that class's own
   * mean (person 3/9/22 s, chair 13/39/104 s) instead of one grid spanning
   * every class. Classes without an entry keep the shared grid.
   */
  void setWaitSetByClass(std::map<std::string, std::vector<double>> by_class) {
    wait_set_by_class_ = std::move(by_class);
  }

  bool is_terminal(const PlannerState& s) const {
    return s.robot_vertex == ctx_->goal;
  }
  void settle(PlannerState& s) const { s.process.advance_to(s.time()); }
  std::vector<SAction> actions(PlannerState& s) const;
  StepResult step(const PlannerState& s, const SAction& a) const;
  const GraphContext& context() const { return *ctx_; }

 private:
  const GraphContext* ctx_;
  double delta_obs_;
  std::vector<double> wait_set_;
  bool allow_observe_;
  bool corridor_traversal_;
  double bin_width_;
  std::map<std::string, std::vector<double>> wait_set_by_class_;
};

// ---------------------------------------------------------------------------
// Belief (ports belief.ParticleBelief)
// ---------------------------------------------------------------------------
struct EdgeMemoryRec {  // ports belief.EdgeMemory
  bool blocked = false;
  double t_obs = 0.0;
  double age_at_obs = 0.0;
  std::string label;  // empty = unlabeled
};

struct LocalObservation {  // ports belief.LocalObservation
  SVertex vertex = 0;
  double time = 0.0;
  std::map<SEdge, int> statuses;      // 1 blocked, 0 free (only observed edges)
  std::map<SEdge, double> ages;       // blocked edges: seconds observed present
  std::map<SEdge, std::string> labels;
  std::map<SEdge, EdgeMemoryRec> memory;  // non-adjacent last sightings
};

class ParticleBelief {
 public:
  ParticleBelief(const GraphContext* ctx, const SparrowModel* model,
                 int num_particles, bool planner_no_adjacent_blocking,
                 uint64_t seed)
      : ctx_(ctx),
        model_(model),
        num_particles_(num_particles),
        no_adj_(planner_no_adjacent_blocking),
        seed_(seed) {}

  void initialize(const LocalObservation& local);
  const std::vector<PlannerState>& particles() const { return particles_; }

  /**
   * \brief Per-edge occupancy weights for CONTRACTED planning.
   *
   * `raw[e] = macroPBlock(p_micro, n_micro(e)) / p_micro`, so `p_block * w_e`
   * recovers `1 - (1 - p_micro)^n` exactly - the composed occupancy of a whole
   * corridor. Ports belief.ParticleBelief's `_edge_p` / `_edge_weight`.
   *
   * Two derived maps, and they are NOT interchangeable:
   *   - `edge_p_` keeps the weights UNNORMALISED, because it is an absolute
   *     probability. Mean-normalising first destroys the scale: on trap_23 the
   *     80-micro-edge trap has p_macro = 0.57, but p_block * (mean-1 weight)
   *     gave 0.02, so 5.6% of particles believed it blocked instead of 57%,
   *     the search priced it at free-run cost, and the planner drove into it
   *     every episode. Invisible when corridors are similar lengths, severe
   *     when they are not.
   *   - `edge_weight_` IS mean-normalised, because the spawn rate only needs
   *     the RELATIVE preference between edges.
   *
   * Pass the map keyed by canonical macro-edge. Calling this with an empty map
   * restores the flat scalar `model_->p_block` (the uncontracted behaviour).
   */
  void setEdgeWeights(const std::map<SEdge, double>& raw);

  /// Steady-state occupancy the belief assigns to one edge under the frozen
  /// model. With contracted weights set this is the COMPOSED corridor
  /// occupancy; without them, the flat p_block. Public for diagnostics and
  /// for the parity check against the simulator.
  double p_edge(const SEdge& edge) const;

 private:
  /// Relative spawn rate multiplier for one edge (mean-normalised).
  double weight_edge(const SEdge& edge) const;

  PlannerState new_particle(const LocalObservation& local);
  void install_tracked_blockage(PlannerState& st, const LocalObservation& local,
                                const SEdge& edge, Rng& rng);
  void install_from_memory(ObstacleProcess& process,
                           const LocalObservation& local, const SEdge& edge,
                           const EdgeMemoryRec& mem, Rng& rng);
  void maybe_fresh_spawn(ObstacleProcess& process,
                         const LocalObservation& local, const SEdge& edge,
                         double delta, Rng& rng);

  const GraphContext* ctx_;
  const SparrowModel* model_;
  int num_particles_;
  bool no_adj_;
  uint64_t seed_;
  int stream_counter_ = 0;
  std::vector<PlannerState> particles_;
  // Empty in uncontracted planning: p_edge() then returns the flat p_block.
  std::map<SEdge, double> edge_p_;       // absolute, UNNORMALISED
  std::map<SEdge, double> edge_weight_;  // relative, mean-normalised
};

// ---------------------------------------------------------------------------
// Solver (ports solver.CostMinimizingPOMCP + rollouts.ShortestPathRollout)
// ---------------------------------------------------------------------------
struct SparrowSearchSettings {
  int num_simulations = 500;
  double max_planning_time_s = 5.0;  // 0/neg = unlimited
  int max_depth = 40;
  double max_sim_time_s = 600.0;
  double c_uct = 30.0;
  double duration_bin_width = 1.0;
  double cutoff_congestion_factor = 1.0;
  /**
   * Among root actions whose visit count is at least this fraction of the
   * most-visited one, return the action with the LOWEST mean cost.
   *
   * Plain robust-child ("most visits, tie-break on Q") is the standard POMCP
   * rule, but it interacts badly with an even exploration floor spread over
   * action CLASSES: the single Traverse and the five MaxWaits get comparable
   * floors, then UCB concentrates on one MaxWait, so a wait wins the visit
   * count while Traverse holds the better Q. Measured (sparrow_test3
   * 22:32:24, a bin): Traverse Q=123.187s with 293 visits LOST to
   * MaxWait(120) Q=124.675s with 375 visits - a 1.2% Q difference, well
   * inside search noise, that cost two more minutes of standing still.
   * 1.0 restores exact robust-child behaviour.
   */
  double robust_visit_frac = 0.75;
  /**
   * Fraction of the simulation budget reserved as an EVEN FLOOR across the
   * root's actions before UCB is allowed to concentrate (handoff sec. 1.4).
   *
   * Clearance times are heavy-tailed, so one unlucky rollout can strand a good
   * action at a ruinous Q with 1-2 visits, and c_uct can never fund a revisit.
   * Measured on trap_aisles: the 40 s aisle route scored q=228 from n=2
   * rollouts (one drew a bin's slow component) against q=76 for the 32 s
   * hallway; 2997 of 3000 simulations then went to the hallway and the
   * alternative was never re-examined. Deployment value: 0.5.
   */
  double root_explore_frac = 0.25;
  /**
   * Stratify that floor by action TYPE rather than spreading it evenly over
   * individual actions.
   *
   * Wait vs reroute vs observe are genuinely different decisions, whereas two
   * wait durations are near-duplicates that can share evidence. Splitting the
   * budget evenly across actions lets a long wait grid drown out the single
   * Traverse; splitting it across types keeps the comparison that matters.
   */
  bool root_explore_by_class = false;
};

struct RootActionStat {
  SAction action;
  int visits = 0;
  double q_cost = 0.0;
};

struct PlanResult {
  std::optional<SAction> action;
  std::vector<RootActionStat> root_actions;
  int simulations = 0;
  double planning_time_s = 0.0;
  int tree_nodes = 0;
  int tree_depth = 0;
};

class SparrowSolver {
 public:
  SparrowSolver(const TransitionModel* transition,
                const SparrowSearchSettings& settings, uint64_t seed)
      : transition_(transition), settings_(settings),
        rng_(derive_seed(seed, 0xC0FFEE)) {}

  /// One simulation per particle, cycling in order (Python parity).
  PlanResult plan(const std::vector<PlannerState>& particles,
                  const std::vector<SAction>& root_actions);

 private:
  struct HistoryNode;
  struct ActionNode {
    int visits = 0;
    double q_cost = 0.0;
    std::map<ObservationKey, std::unique_ptr<HistoryNode>> children;
    void update(double total) {
      visits += 1;
      q_cost += (total - q_cost) / visits;
    }
  };
  struct HistoryNode {
    int visits = 0;
    bool expanded = false;
    std::map<SAction, ActionNode> children;
  };

  double simulate(PlannerState state, HistoryNode& node, int depth,
                  double sim_start_time,
                  const std::vector<SAction>* forced_actions);
  const SAction* select(HistoryNode& node, const std::vector<SAction>& actions,
                        bool is_root);
  double rollout(PlannerState state, double sim_start_time);
  double cutoff_cost(const PlannerState& state) const;
  SAction rollout_action(PlannerState& state,
                         const std::vector<SAction>& actions) const;

  const TransitionModel* transition_;
  SparrowSearchSettings settings_;
  Rng rng_;
  int nodes_ = 0;
  int max_depth_seen_ = 0;
};

}  // namespace sparrow
}  // namespace navigation
}  // namespace vtr
