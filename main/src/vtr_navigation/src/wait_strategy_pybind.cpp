// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// pybind11 bridge: Python episodes call LearnedStrategy::computeWaitTime in C++.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_logging/configure.hpp"
#include "vtr_navigation/sparrow_contract.hpp"
#include "vtr_navigation/sparrow_strategy.hpp"
#include "vtr_navigation/wait_strategy.hpp"
#include "vtr_tactic/types.hpp"

namespace py = pybind11;

using vtr::navigation::EdgeIdSet;
using vtr::navigation::LearnedStrategy;
using vtr::navigation::SparrowStrategy;
using vtr::navigation::WaitDecision;
using vtr::navigation::WaitStrategyConfig;
using vtr::tactic::EdgeId;
using vtr::tactic::VertexId;

namespace {

void quietLogging() {
  static bool once = false;
  if (once) return;
  once = true;
  vtr::logging::configureLogging("", false);
  el::Configurations conf;
  conf.setToDefault();
  conf.setGlobally(el::ConfigurationType::Enabled, "false");
  el::Loggers::reconfigureAllLoggers(conf);
}

VertexId vid(int minor) { return VertexId(0, static_cast<uint32_t>(minor)); }

/// Full taught-graph vertex id (major << 32 | minor). sep12_4 spans 21 runs,
/// so a run-0-only helper cannot address it.
VertexId sv(uint64_t raw) { return VertexId(raw); }

/// dict.get(key, fallback), typed. Missing and None both fall back, so a
/// Python caller can hand the whole YAML sparrow block straight through and
/// let the C++ defaults stand for whatever it does not set.
template <typename T>
T dget(const py::dict& d, const char* key, T fallback) {
  if (!d.contains(key)) return fallback;
  py::object v = d[key];
  if (v.is_none()) return fallback;
  return v.cast<T>();
}

}  // namespace

/**
 * Persistent C++ wait engine for the Python sim hybrid.
 *
 * Call set_graph once (or when graph changes), sync_km / sync_stats / sync_memory
 * whenever Python state changes (typically each episode or each decision), then
 * compute_wait_time() which is wall-clock timed in C++.
 */
class WaitEngine {
 public:
  WaitEngine() { quietLogging(); }

  void configure(int W_grid_points, int T_grid_points, double speed_mps,
                 const std::map<std::string, double>& W_max_per_type) {
    cfg_.W_grid_points = W_grid_points;
    cfg_.T_grid_points = T_grid_points;
    cfg_.robot_speed_mps = speed_mps;
    cfg_.W_max_per_type = W_max_per_type;
    cfg_.type_weights.clear();
    for (const auto& kv : W_max_per_type) {
      cfg_.type_weights[kv.first] = 1.0;
    }
    cfg_.learned_data_dir.clear();
    cfg_.debug_plot_policy = false;
    cfg_.seed_samples.clear();
    strategy_ = std::make_unique<LearnedStrategy>(cfg_);
    wireGraph();
  }

  void set_graph(const std::vector<std::tuple<int, int, double>>& directed_edges) {
    neighbors_.clear();
    travel_.clear();
    for (const auto& trip : directed_edges) {
      const int u = std::get<0>(trip);
      const int v = std::get<1>(trip);
      const double tt = std::get<2>(trip);
      neighbors_[u].push_back(v);
      EdgeId e(vid(u), vid(v));
      travel_[e.hash()] = tt;
    }
    if (strategy_) wireGraph();
  }

  void sync_km(const std::vector<std::tuple<std::string, double, bool>>& samples) {
    ensureStrategy();
    strategy_->survivalModel()->clear();
    for (const auto& s : samples) {
      strategy_->survivalModel()->addSample(std::get<0>(s), std::get<1>(s),
                                            std::get<2>(s), /*episode=*/0);
    }
  }

  void sync_stats(int edges_traversed,
                  const std::vector<std::pair<std::string, int>>& type_episodes) {
    ensureStrategy();
    auto* stats = strategy_->obstacleStats();
    stats->clear();
    if (edges_traversed > 0) {
      stats->recordEdgeTraversals(edges_traversed);
    }
    for (const auto& kv : type_episodes) {
      for (int i = 0; i < kv.second; ++i) {
        stats->recordObstacleEpisode(kv.first);
      }
    }
  }

  void sync_memory(
      const std::vector<std::tuple<int, int, std::string, double, double>>& mems) {
    ensureStrategy();
    strategy_->resetMemory();
    for (const auto& m : mems) {
      EdgeId e(vid(std::get<0>(m)), vid(std::get<1>(m)));
      strategy_->seedMemoryForEdge(e, std::get<2>(m), std::get<3>(m), std::get<4>(m));
    }
  }

  py::dict compute_wait_time(const std::string& obs_type, int blocked_u, int blocked_v,
                             int current, int goal, double t_now, double obstacle_t_first) {
    ensureStrategy();
    EdgeIdSet blocked;
    blocked.insert(EdgeId(vid(blocked_u), vid(blocked_v)));

    const auto t0 = std::chrono::steady_clock::now();
    WaitDecision dec = strategy_->computeWaitTime(
        obs_type, blocked, vid(current), vid(goal), t_now, obstacle_t_first);
    const auto t1 = std::chrono::steady_clock::now();
    const double wall = std::chrono::duration<double>(t1 - t0).count();

    // computeWaitTime records the current obstacle into memory; strip it so the
    // next sync_memory from Python is authoritative.
    strategy_->clearMemoryForEdge(EdgeId(vid(blocked_u), vid(blocked_v)));

    py::dict out;
    out["W_star"] = dec.W_star;
    out["should_wait"] = dec.should_wait;
    out["wall_time_s"] = wall;
    out["speech"] = dec.speech;
    return out;
  }

 private:
  void ensureStrategy() {
    if (!strategy_) {
      throw std::runtime_error("WaitEngine: call configure() before use");
    }
  }

  void wireGraph() {
    if (!strategy_) return;
    strategy_->setGraphAccess(
        [this](const VertexId& v) -> std::vector<VertexId> {
          const int id = static_cast<int>(v.minorId());
          auto it = neighbors_.find(id);
          if (it == neighbors_.end()) return {};
          std::vector<VertexId> out;
          out.reserve(it->second.size());
          for (int n : it->second) out.push_back(vid(n));
          return out;
        },
        [this](const EdgeId& e) -> double {
          auto it = travel_.find(e.hash());
          if (it == travel_.end()) return 1e9;
          return it->second;
        });
  }

  WaitStrategyConfig cfg_;
  std::unique_ptr<LearnedStrategy> strategy_;
  std::unordered_map<int, std::vector<int>> neighbors_;
  std::unordered_map<size_t, double> travel_;
};

/**
 * Decision-level SPARROW bridge: the SAME SparrowStrategy the robot runs,
 * driven from Python.
 *
 * WaitEngine above wraps LearnedStrategy (OSCAR). It cannot answer the
 * question "given identical conditions, do the Python simulator and the C++
 * teach-and-repeat take the same decision?", because on the robot the
 * decision is not computeWaitTime's grid search - it is the POMCP over the
 * contracted graph, plus the observation hooks the Navigator supplies.
 *
 * So this engine reproduces the Navigator's side of that contract without a
 * robot: Python supplies the taught graph, a micro-edge status map (which is
 * how an obstacle gets placed on a chosen vertex pair by hand), the learned
 * model, and the scenario; the C++ runs the real planner and reports the
 * action AND the root-action ranking, which is what the handoff says must
 * match (Q-values cannot - different RNG).
 */
class SparrowEngine {
 public:
  SparrowEngine() { quietLogging(); }

  /**
   * \param sparrow  the config's `sparrow:` block, key for key.
   * \param top      the surrounding wait-strategy keys (W_max_per_type,
   *                 type_weights, seed_samples, ...). learned_data_dir is
   *                 deliberately NOT read: a parity run must not write the
   *                 robot's banked stats, and must not read them either.
   */
  void configure(const py::dict& sparrow, const py::dict& top) {
    cfg_ = WaitStrategyConfig{};
    auto& sp = cfg_.sparrow;

    sp.num_simulations = dget(sparrow, "num_simulations", sp.num_simulations);
    sp.max_planning_time_s =
        dget(sparrow, "max_planning_time_s", sp.max_planning_time_s);
    sp.max_depth = dget(sparrow, "max_depth", sp.max_depth);
    sp.max_sim_time_s = dget(sparrow, "max_sim_time_s", sp.max_sim_time_s);
    sp.c_uct = dget(sparrow, "c_uct", sp.c_uct);
    sp.duration_bin_width =
        dget(sparrow, "duration_bin_width", sp.duration_bin_width);
    sp.num_particles = dget(sparrow, "num_particles", sp.num_particles);
    sp.wait_durations =
        dget(sparrow, "wait_durations", sp.wait_durations);
    sp.wait_durations_by_class =
        dget(sparrow, "wait_durations_by_class", sp.wait_durations_by_class);
    sp.delta_obs_s = dget(sparrow, "delta_obs_s", sp.delta_obs_s);
    sp.allow_observe = dget(sparrow, "allow_observe", sp.allow_observe);
    sp.single_observe_per_encounter = dget(
        sparrow, "single_observe_per_encounter", sp.single_observe_per_encounter);
    sp.corridor_traversal =
        dget(sparrow, "corridor_traversal", sp.corridor_traversal);
    sp.max_total_wait_s =
        dget(sparrow, "max_total_wait_s", sp.max_total_wait_s);
    sp.p_block_override =
        dget(sparrow, "p_block_override", sp.p_block_override);
    sp.p_block_live = dget(sparrow, "p_block_live", sp.p_block_live);
    sp.teach_prior = dget(sparrow, "teach_prior", sp.teach_prior);
    sp.root_explore_frac =
        dget(sparrow, "root_explore_frac", sp.root_explore_frac);
    sp.root_explore_by_class =
        dget(sparrow, "root_explore_by_class", sp.root_explore_by_class);
    sp.robust_visit_frac =
        dget(sparrow, "robust_visit_frac", sp.robust_visit_frac);
    sp.contracted = dget(sparrow, "contracted", sp.contracted);
    sp.max_macro_len_s =
        dget(sparrow, "max_macro_len_s", sp.max_macro_len_s);
    sp.corner_vertices =
        dget<std::string>(sparrow, "corner_vertices", sp.corner_vertices);
    sp.plan_at_robot = dget(sparrow, "plan_at_robot", sp.plan_at_robot);
    sp.junction_replan = dget(sparrow, "junction_replan", sp.junction_replan);
    sp.junction_divert_margin_s =
        dget(sparrow, "junction_divert_margin_s", sp.junction_divert_margin_s);
    sp.kg_enabled = dget(sparrow, "kg_enabled", sp.kg_enabled);
    // A parity run is reproducible or it is not a parity run.
    sp.planner_seed = dget(sparrow, "planner_seed", 12345);

    cfg_.W_max_per_type =
        dget(top, "W_max_per_type", std::map<std::string, double>{});
    cfg_.unknown_W_max = dget(top, "unknown_W_max", cfg_.unknown_W_max);
    cfg_.type_weights =
        dget(top, "type_weights", std::map<std::string, double>{});
    cfg_.class_order =
        dget(top, "classes", std::vector<std::string>{});
    cfg_.spawn_type_weights =
        dget(top, "spawn_type_weights", std::map<std::string, double>{});
    cfg_.spawn_mean_duration_s =
        dget(top, "spawn_mean_duration_s", cfg_.spawn_mean_duration_s);
    cfg_.seed_samples = dget(
        top, "seed_samples", std::map<std::string, std::vector<double>>{});
    cfg_.seed_samples_censored =
        dget(top, "seed_samples_censored",
             std::map<std::string, std::vector<double>>{});
    cfg_.robot_speed_mps = dget(top, "robot_speed_mps", cfg_.robot_speed_mps);
    cfg_.W_grid_points = dget(top, "W_grid_points", cfg_.W_grid_points);
    cfg_.T_grid_points = dget(top, "T_grid_points", cfg_.T_grid_points);
    cfg_.learned_data_dir.clear();
    cfg_.debug_plot_policy = false;

    strategy_ = std::make_unique<SparrowStrategy>(cfg_);
    wire();
  }

  /** \param directed_edges (u, v, travel_time_s); pass both directions. */
  void set_graph(
      const std::vector<std::tuple<uint64_t, uint64_t, double>>& directed_edges) {
    neighbors_.clear();
    travel_.clear();
    for (const auto& t : directed_edges) {
      const uint64_t u = std::get<0>(t), v = std::get<1>(t);
      neighbors_[u].push_back(v);
      travel_[vtr::navigation::sparrow::canonical_edge(u, v)] = std::get<2>(t);
    }
    if (strategy_) wire();
  }

  /**
   * \brief Place the obstacles: micro-edge -> 1 blocked / 0 free / -1 unknown.
   *
   * This is the detector's output on the robot. Edges absent from the map read
   * UNKNOWN, exactly as they do when the costmap has not seen them - which
   * matters, because "unknown" is priced by the belief while "free" is not.
   */
  void set_micro_statuses(
      const std::map<std::pair<uint64_t, uint64_t>, int>& statuses) {
    micro_.clear();
    for (const auto& kv : statuses) {
      micro_[vtr::navigation::sparrow::canonical_edge(kv.first.first,
                                                      kv.first.second)] =
          kv.second;
    }
  }

  void sync_km(const std::vector<std::tuple<std::string, double, bool>>& samples) {
    ensure();
    strategy_->survivalModel()->clear();
    for (const auto& s : samples) {
      strategy_->survivalModel()->addSample(std::get<0>(s), std::get<1>(s),
                                            std::get<2>(s), /*episode=*/0);
    }
  }

  /**
   * \brief Set the occupancy counters directly. `p_block_override` in the
   *        config pins occupancy regardless, which is what the oracle run
   *        does; this exists for the learned-counter case.
   */
  void sync_stats(int edges_traversed,
                  const std::vector<std::pair<std::string, int>>& type_episodes,
                  int unlabeled_episodes) {
    ensure();
    auto* stats = strategy_->obstacleStats();
    stats->clear();
    stats->setDefaultTypeWeights(cfg_.type_weights);
    if (edges_traversed > 0) stats->recordEdgeTraversals(edges_traversed);
    for (const auto& kv : type_episodes)
      for (int i = 0; i < kv.second; ++i) stats->recordObstacleEpisode(kv.first);
    for (int i = 0; i < unlabeled_episodes; ++i) stats->recordUnlabeledEpisode();
    if (cfg_.sparrow.teach_prior > 0)
      stats->applyTeachPrior(cfg_.sparrow.teach_prior);
  }

  /** \brief A remembered blocked sighting of a known class and known age. */
  void seed_blocked(uint64_t u, uint64_t v, const std::string& label,
                    double t_first, double t_last) {
    ensure();
    strategy_->seedBlockedSighting(
        vtr::navigation::sparrow::canonical_edge(u, v), label, t_first, t_last);
  }

  void reset_memory() {
    ensure();
    strategy_->resetMemory();
  }

  void reset_encounter() {
    ensure();
    strategy_->resetEncounter();
  }

  void notify_episode_start(int episode_idx) {
    ensure();
    strategy_->notifyEpisodeStart(episode_idx);
  }

  /**
   * \brief The model the search will actually run against (num_edges is the
   *        contracted edge count the belief is built over).
   */
  py::dict model(int num_edges) {
    ensure();
    const auto m = strategy_->frozenModel(num_edges);
    py::dict out;
    out["p_block"] = m.p_block;
    out["spawn_rate_hz"] = m.spawn_rate_hz;
    out["num_edges"] = m.num_edges;
    out["prior_residual_mean"] = m.prior_residual_mean;
    out["class_names"] = m.class_names;
    out["class_probs"] = m.class_probs;
    std::vector<std::string> fitted(m.fitted_classes.begin(),
                                    m.fitted_classes.end());
    out["fitted_classes"] = fitted;
    return out;
  }

  double raw_p_block() {
    ensure();
    return strategy_->obstacleStats()->p_block();
  }

  double fresh_edge_expected_wait() {
    ensure();
    return strategy_->freshEdgeExpectedWait();
  }

  double survival(const std::string& type, double t) {
    ensure();
    return strategy_->survivalModel()->survival(type, t);
  }

  /// Raw draws from the model's residual sampler, for distribution parity.
  std::vector<double> sample_residuals(const std::string& type, int n,
                                       double elapsed, uint64_t seed) {
    ensure();
    const auto m = strategy_->frozenModel(1);
    vtr::navigation::sparrow::Rng rng(seed);
    std::vector<double> out;
    out.reserve(std::max(0, n));
    for (int i = 0; i < n; ++i)
      out.push_back(m.sample_residual(type, rng, elapsed));
    return out;
  }

  double mean_survival(const std::string& type, double w_max) {
    ensure();
    return strategy_->survivalModel()->meanSurvivalTime(type, w_max);
  }

  /**
   * \brief Run one real SPARROW decision.
   *
   * \param blocked_edges the FRONT blockage (what stopped the robot); pass an
   *        empty list with a junction vertex to exercise en-route replanning.
   */
  py::dict decide(const std::string& obs_type,
                  const std::vector<std::pair<uint64_t, uint64_t>>& blocked_edges,
                  uint64_t current, uint64_t goal, double t_now,
                  double obstacle_t_first) {
    ensure();
    EdgeIdSet blocked;
    for (const auto& e : blocked_edges)
      blocked.insert(EdgeId(sv(e.first), sv(e.second)));

    const auto t0 = std::chrono::steady_clock::now();
    const WaitDecision dec = strategy_->computeWaitTime(
        obs_type, blocked, sv(current), sv(goal), t_now, obstacle_t_first);
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    return pack(dec, wall);
  }

  /** \brief Every-decision replanning at an upcoming junction. */
  py::dict decide_en_route(uint64_t robot, uint64_t junction, uint64_t goal,
                           double t_now, uint64_t route_next_hop) {
    ensure();
    const auto t0 = std::chrono::steady_clock::now();
    const WaitDecision dec = strategy_->planEnRoute(
        sv(robot), sv(junction), sv(goal), t_now, route_next_hop);
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    return pack(dec, wall);
  }

 private:
  void ensure() const {
    if (!strategy_)
      throw std::runtime_error("SparrowEngine: call configure() first");
  }

  int microStatus(uint64_t a, uint64_t b) const {
    auto it = micro_.find(vtr::navigation::sparrow::canonical_edge(a, b));
    return (it == micro_.end()) ? vtr::navigation::sparrow::kEdgeUnknown
                                : it->second;
  }

  /// Action kind implied by the decision, in the vocabulary the simulator
  /// reports: the strategy answers in Navigator terms (observe / wait /
  /// detour) rather than returning the SAction it chose.
  static std::string kindOf(const WaitDecision& d) {
    if (d.request_observation) return "observe";
    if (d.should_wait) return "maxwait";
    return "traverse";
  }

  py::dict pack(const WaitDecision& dec, double wall) {
    py::dict out;
    out["action"] = kindOf(dec);
    out["W_star"] = dec.W_star;
    out["should_wait"] = dec.should_wait;
    out["request_observation"] = dec.request_observation;
    out["observe_edge"] = dec.observe_edge;
    out["traverse_edge"] = dec.traverse_edge;
    out["detour_ban_edges"] = dec.detour_ban_edges;
    out["speech"] = dec.speech;
    out["wall_time_s"] = wall;
    out["planning_vertex"] = strategy_->lastPlanningVertex();

    // Root ranking, best first: what the handoff requires to agree.
    py::list acts;
    for (const auto& r : strategy_->lastRootActions()) {
      py::dict a;
      a["kind"] = (r.kind == 0) ? "traverse" : (r.kind == 1 ? "maxwait"
                                                            : "observe");
      a["edge"] = r.edge;
      a["first_hop"] = r.first_hop;
      a["W"] = r.W;
      a["visits"] = r.visits;
      a["q_cost"] = r.q_cost;
      acts.append(a);
    }
    out["root_actions"] = acts;
    return out;
  }

  void wire() {
    if (!strategy_) return;
    namespace sp = vtr::navigation::sparrow;

    strategy_->setGraphAccess(
        [this](const VertexId& v) -> std::vector<VertexId> {
          auto it = neighbors_.find(static_cast<uint64_t>(v));
          if (it == neighbors_.end()) return {};
          std::vector<VertexId> out;
          out.reserve(it->second.size());
          for (uint64_t n : it->second) out.push_back(sv(n));
          return out;
        },
        [this](const EdgeId& e) -> double {
          auto it = travel_.find(sp::canonical_edge(
              static_cast<uint64_t>(e.id1()), static_cast<uint64_t>(e.id2())));
          return (it == travel_.end()) ? 1e9 : it->second;
        });

    // Detector hook 1: micro-edges incident to a vertex. Only edges Python
    // gave a status for are reported, so the rest stay unknown.
    strategy_->setAdjacentEdgeStatusFn(
        [this](const VertexId& v) -> std::map<EdgeId, int> {
          std::map<EdgeId, int> out;
          auto it = neighbors_.find(static_cast<uint64_t>(v));
          if (it == neighbors_.end()) return out;
          for (uint64_t n : it->second) {
            auto mit = micro_.find(
                vtr::navigation::sparrow::canonical_edge(
                    static_cast<uint64_t>(v), n));
            if (mit == micro_.end()) continue;
            out[EdgeId(sv(static_cast<uint64_t>(v)), sv(n))] = mit->second;
          }
          return out;
        });

    // Detector hook 2: per-corridor status, mirroring
    // Navigator::computeMacroEdgeStatuses - blocked as soon as any micro-edge
    // of the chain is, free only when every one was seen free, and only the
    // part of the chain still AHEAD of the robot counts.
    strategy_->setMacroEdgeStatusFn(
        [this](sp::SVertex root, const sp::MacroPlan& plan,
               sp::SVertex robot_at) -> std::map<sp::SEdge, int> {
          std::map<sp::SEdge, int> out;
          for (const auto& kv : plan.macros) {
            const auto& m = kv.second;
            if (root != m.u && root != m.v) continue;
            const std::vector<sp::SVertex> chain =
                (robot_at != 0) ? sp::remainingChainFrom(m, root, robot_at)
                                : sp::macroChainFrom(m, root);
            if (chain.size() < 2) {
              out[kv.first] = sp::kEdgeUnknown;
              continue;
            }
            out[kv.first] = sp::corridorStatus(
                chain, [this](sp::SVertex a, sp::SVertex b) {
                  return microStatus(a, b);
                });
          }
          return out;
        });
  }

  WaitStrategyConfig cfg_;
  std::unique_ptr<SparrowStrategy> strategy_;
  std::map<uint64_t, std::vector<uint64_t>> neighbors_;
  std::map<std::pair<uint64_t, uint64_t>, double> travel_;
  std::map<std::pair<uint64_t, uint64_t>, int> micro_;
};

PYBIND11_MODULE(vtr_wait_strategy_py, m) {
  m.doc() =
      "C++ wait strategies for Python: LearnedStrategy (OSCAR) and the real "
      "SPARROW POMCP decision, plus its components for parity checks";

  py::class_<WaitEngine>(m, "WaitEngine")
      .def(py::init<>())
      .def("configure", &WaitEngine::configure, py::arg("W_grid_points"),
           py::arg("T_grid_points"), py::arg("speed_mps"), py::arg("W_max_per_type"))
      .def("set_graph", &WaitEngine::set_graph, py::arg("directed_edges"))
      .def("sync_km", &WaitEngine::sync_km, py::arg("samples"))
      .def("sync_stats", &WaitEngine::sync_stats, py::arg("edges_traversed"),
           py::arg("type_episodes"))
      .def("sync_memory", &WaitEngine::sync_memory, py::arg("memories"))
      .def("compute_wait_time", &WaitEngine::compute_wait_time, py::arg("obs_type"),
           py::arg("blocked_u"), py::arg("blocked_v"), py::arg("current"), py::arg("goal"),
           py::arg("t_now"), py::arg("obstacle_t_first"));

  // ---- Decision-level SPARROW (the planner the robot actually runs) ------
  py::class_<SparrowEngine>(m, "SparrowEngine")
      .def(py::init<>())
      .def("configure", &SparrowEngine::configure, py::arg("sparrow"),
           py::arg("top") = py::dict())
      .def("set_graph", &SparrowEngine::set_graph, py::arg("directed_edges"))
      .def("set_micro_statuses", &SparrowEngine::set_micro_statuses,
           py::arg("statuses"))
      .def("sync_km", &SparrowEngine::sync_km, py::arg("samples"))
      .def("sync_stats", &SparrowEngine::sync_stats, py::arg("edges_traversed"),
           py::arg("type_episodes"), py::arg("unlabeled_episodes") = 0)
      .def("seed_blocked", &SparrowEngine::seed_blocked, py::arg("u"),
           py::arg("v"), py::arg("label"), py::arg("t_first"),
           py::arg("t_last"))
      .def("reset_memory", &SparrowEngine::reset_memory)
      .def("reset_encounter", &SparrowEngine::reset_encounter)
      .def("notify_episode_start", &SparrowEngine::notify_episode_start,
           py::arg("episode_idx"))
      .def("model", &SparrowEngine::model, py::arg("num_edges"))
      .def("raw_p_block", &SparrowEngine::raw_p_block)
      .def("fresh_edge_expected_wait", &SparrowEngine::fresh_edge_expected_wait)
      .def("survival", &SparrowEngine::survival, py::arg("type"), py::arg("t"))
      .def("mean_survival", &SparrowEngine::mean_survival, py::arg("type"),
           py::arg("w_max"))
      .def("sample_residuals", &SparrowEngine::sample_residuals,
           py::arg("type"), py::arg("n"), py::arg("elapsed") = 0.0,
           py::arg("seed") = 1)
      .def("decide", &SparrowEngine::decide, py::arg("obs_type"),
           py::arg("blocked_edges"), py::arg("current"), py::arg("goal"),
           py::arg("t_now"), py::arg("obstacle_t_first") = 0.0)
      .def("decide_en_route", &SparrowEngine::decide_en_route,
           py::arg("robot"), py::arg("junction"), py::arg("goal"),
           py::arg("t_now"), py::arg("route_next_hop") = 0);

  // ---- Kaplan-Meier parity -----------------------------------------------
  // Same samples in, same S(t) and same inverse-CDF draws out? This is the
  // one place the two implementations can still differ once the streams and
  // orderings agree, because the survival curve is what every residual draw
  // is read off. (Against POMCP_ORACLE it CANNOT agree - the oracle stores an
  // analytic table, the robot a KM step function - so this compares the
  // robot's KM against the SIMULATOR'S KM, which is the learned deployment
  // both actually run.)
  m.def(
      "km_probe",
      [](const std::vector<std::tuple<std::string, double, bool>>& samples,
         const std::string& type, const std::vector<double>& t_grid,
         const std::vector<double>& u_grid, double elapsed, double w_max) {
        vtr::navigation::SurvivalModel km;
        for (const auto& s : samples)
          km.addSample(std::get<0>(s), std::get<1>(s), std::get<2>(s), 0);

        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.class_names = {type};
        model.class_probs = {1.0};
        model.km = &km;
        model.fitted_classes.insert(type);

        py::dict out;
        std::vector<double> surv, resid, draws;
        for (double t : t_grid) surv.push_back(km.survival(type, t));
        for (double t : t_grid)
          resid.push_back(model.residual_survival(type, t, elapsed));
        // Inverse-CDF at explicit u values, so the comparison does not depend
        // on the two RNGs landing on the same numbers.
        for (double u : u_grid) {
          double lo = 0.0, hi = 1.0;
          int guard = 0;
          while (model.residual_survival(type, hi, elapsed) > u &&
                 guard++ < 40) {
            hi *= 2.0;
            if (hi > 1e6) break;
          }
          for (int i = 0; i < 50; ++i) {
            const double mid = 0.5 * (lo + hi);
            if (model.residual_survival(type, mid, elapsed) > u) lo = mid;
            else hi = mid;
          }
          draws.push_back(0.5 * (lo + hi));
        }
        out["survival"] = surv;
        out["residual_survival"] = resid;
        out["inverse_cdf"] = draws;
        out["mean"] = km.meanSurvivalTime(type, w_max);
        return out;
      },
      py::arg("samples"), py::arg("type"), py::arg("t_grid"),
      py::arg("u_grid"), py::arg("elapsed") = 0.0, py::arg("w_max") = 3180.0);

  // ---- Particle draw parity ----------------------------------------------
  // Draws ONE particle from ParticleBelief with an explicit edge order and
  // reports what it put on each edge. The belief consumes one uniform per edge
  // for the Bernoulli, then a class and a residual for the edges that come up
  // blocked, so the drawn EDGE SET and CLASS SEQUENCE are exactly the stream
  // alignment -- independent of how each side represents the survival curve,
  // which is the one thing that cannot be made identical (Kaplan-Meier steps
  // here, a tabulated analytic curve in the oracle).
  m.def(
      "debug_particle",
      [](const std::vector<std::pair<uint64_t, uint64_t>>& edge_order,
         const std::map<std::pair<uint64_t, uint64_t>, double>& raw_weights,
         double p_block, const std::vector<std::string>& class_names,
         const std::vector<double>& class_probs,
         const std::vector<double>& spawn_probs, double spawn_rate_hz,
         int num_edges, double prior_residual_mean, double t_now,
         uint64_t root, uint64_t seed, int index) {
        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.class_names = class_names;
        model.class_probs = class_probs;
        model.spawn_class_probs = spawn_probs;
        model.p_block = p_block;
        model.spawn_rate_hz = spawn_rate_hz;
        model.num_edges = num_edges;
        model.prior_residual_mean = prior_residual_mean;
        model.km = nullptr;  // exponential prior: representation-free

        sp::GraphContext ctx;
        ctx.edges.assign(edge_order.begin(), edge_order.end());
        sp::ParticleBelief belief(&ctx, &model, index + 1, false, seed);
        belief.setEdgeWeights(raw_weights);
        belief.setEdgeOrder(ctx.edges);

        sp::LocalObservation local;
        local.time = t_now;
        local.vertex = root;
        belief.initialize(local);

        const auto& st = belief.particles().at(index);
        std::vector<std::tuple<std::pair<uint64_t, uint64_t>, std::string,
                               double>> out;
        for (const auto& kv : st.process.active()) {
          out.emplace_back(kv.first, kv.second.obs_type, kv.second.t_clear);
        }
        return out;
      },
      py::arg("edge_order"), py::arg("raw_weights"), py::arg("p_block"),
      py::arg("class_names"), py::arg("class_probs"), py::arg("spawn_probs"),
      py::arg("spawn_rate_hz"), py::arg("num_edges"),
      py::arg("prior_residual_mean"), py::arg("t_now"), py::arg("root"),
      py::arg("seed"), py::arg("index") = 0);

  // ---- Corridor contraction (paper Sec. IV; handoff Sec. 2) ---------------
  // Exposed so the C++ contraction can be cross-checked against
  // vtr3_sim/pomcp/contract.py on the real taught graph, vertex by vertex,
  // rather than only against the two summary counts.
  m.def(
      "build_macro_plan",
      [](const std::map<uint64_t, std::vector<uint64_t>>& neighbors,
         const std::map<std::pair<uint64_t, uint64_t>, double>& travel_time,
         uint64_t goal, uint64_t root, const std::set<uint64_t>& extra_nodes,
         double max_macro_len_s) {
        namespace sp = vtr::navigation::sparrow;
        sp::GraphContext micro(neighbors, travel_time, goal);
        const auto plan =
            sp::buildMacroPlan(micro, root, extra_nodes, max_macro_len_s);

        // (u, v, first_hop, n_micro, travel_time, chain) per macro-edge,
        // keyed by the canonical edge.
        py::dict macros;
        for (const auto& kv : plan.macros) {
          const auto& m2 = kv.second;
          macros[py::make_tuple(kv.first.first, kv.first.second)] =
              py::make_tuple(m2.u, m2.v, m2.first_hop, m2.n_micro(),
                             m2.travel_time, m2.chain);
        }
        py::dict out;
        out["macros"] = macros;
        out["nodes"] = plan.context.neighbors.size();
        out["corridors"] = plan.macros.size();
        std::vector<uint64_t> node_list;
        for (const auto& kv : plan.context.neighbors)
          node_list.push_back(kv.first);
        out["node_list"] = node_list;
        // Discovery order of the macro-edges; the belief draws particles in
        // this order, so it has to match the simulator's dict order.
        std::vector<std::pair<uint64_t, uint64_t>> order(plan.order.begin(),
                                                         plan.order.end());
        out["order"] = order;
        return out;
      },
      py::arg("neighbors"), py::arg("travel_time"), py::arg("goal"),
      py::arg("root"), py::arg("extra_nodes") = std::set<uint64_t>{},
      py::arg("max_macro_len_s") = 0.0);

  m.def("parse_corner_spec", [](const std::string& spec) {
    return vtr::navigation::sparrow::parseCornerSpec(spec);
  });

  m.def("macro_p_block", [](double p_micro, size_t n_micro) {
    return vtr::navigation::sparrow::macroPBlock(p_micro, n_micro);
  });

  // Per-class MaxWait grids (handoff sec. 1.8): an edge the robot has PAID to
  // Observe gets its class's own grid; everything else gets the shared one.
  m.def(
      "valid_actions",
      [](uint64_t vertex, const std::map<std::pair<uint64_t, uint64_t>, bool>& st,
         const std::vector<uint64_t>& nbrs,
         const std::set<std::pair<uint64_t, uint64_t>>& classified,
         const std::vector<double>& wait_set, bool allow_observe,
         const std::map<std::string, std::vector<double>>& by_class,
         const std::map<std::pair<uint64_t, uint64_t>, std::string>& classes) {
        namespace sp = vtr::navigation::sparrow;
        const auto acts = sp::valid_actions(
            vertex, st, nbrs, classified, wait_set, allow_observe,
            by_class.empty() ? nullptr : &by_class,
            classes.empty() ? nullptr : &classes);
        // (kind, edge, W): kind 0=TRAVERSE 1=MAXWAIT 2=OBSERVE
        std::vector<std::tuple<int, std::pair<uint64_t, uint64_t>, double>> out;
        for (const auto& a : acts) {
          out.emplace_back(static_cast<int>(a.kind), a.edge, a.W);
        }
        return out;
      },
      py::arg("vertex"), py::arg("statuses"), py::arg("neighbors"),
      py::arg("classified"), py::arg("wait_set"), py::arg("allow_observe") = true,
      py::arg("wait_set_by_class") = std::map<std::string, std::vector<double>>{},
      py::arg("edge_classes") =
          std::map<std::pair<uint64_t, uint64_t>, std::string>{});

  // Contracted occupancy: exercises the REAL ParticleBelief::setEdgeWeights
  // and p_edge path, not a reimplementation of the arithmetic. Given each
  // macro-edge's micro-edge count, returns the occupancy the belief will use.
  m.def(
      "belief_edge_occupancy",
      [](const std::map<std::pair<uint64_t, uint64_t>, size_t>& n_micro,
         double p_block) {
        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.p_block = p_block;
        sp::GraphContext ctx;
        sp::ParticleBelief belief(&ctx, &model, 1, false, 1);

        // w_e = macroPBlock(p, n) / p  (contract.py's _macro_view weights)
        std::map<std::pair<uint64_t, uint64_t>, double> raw;
        for (const auto& kv : n_micro) {
          raw[kv.first] = (p_block > 0.0)
                              ? sp::macroPBlock(p_block, kv.second) / p_block
                              : static_cast<double>(kv.second);
        }
        belief.setEdgeWeights(raw);

        std::map<std::pair<uint64_t, uint64_t>, double> out;
        for (const auto& kv : n_micro) out[kv.first] = belief.p_edge(kv.first);
        return out;
      },
      py::arg("n_micro"), py::arg("p_block"));

  // Corridor sight: a macro-edge is blocked iff ANY of its micro-edges is,
  // free only when EVERY one was observed free, unknown otherwise. While
  // driving, only the part of the chain ahead of the robot counts.
  m.def(
      "corridor_status",
      [](const std::vector<uint64_t>& chain,
         const std::map<std::pair<uint64_t, uint64_t>, int>& micro) {
        namespace sp = vtr::navigation::sparrow;
        return sp::corridorStatus(chain, [&](uint64_t a, uint64_t b) {
          auto it = micro.find(sp::canonical_edge(a, b));
          return (it == micro.end()) ? sp::kEdgeUnknown : it->second;
        });
      },
      py::arg("chain"), py::arg("micro_status"));

  m.def(
      "macro_chain_from",
      [](uint64_t u, uint64_t v, const std::vector<uint64_t>& chain,
         uint64_t from, bool remaining, uint64_t robot_at) {
        namespace sp = vtr::navigation::sparrow;
        sp::MacroEdge m2;
        m2.u = u;
        m2.v = v;
        m2.chain = chain;
        m2.first_hop = chain.empty() ? v : chain.front();
        return remaining ? sp::remainingChainFrom(m2, from, robot_at)
                         : sp::macroChainFrom(m2, from);
      },
      py::arg("u"), py::arg("v"), py::arg("chain"), py::arg("from_vertex"),
      py::arg("remaining") = false, py::arg("robot_at") = 0);

  // Marthi B-set (handoff-adjacent; ports runner._blocked_vertex_set). The
  // model is built with km = nullptr, so every class falls back to the
  // exponential prior and the survival weighting is analytic - which is what
  // makes the parity check closed-form rather than dependent on a KM bridge.
  m.def(
      "blocked_vertex_set",
      [](const std::map<std::pair<uint64_t, uint64_t>, int>& statuses,
         // edge -> (blocked, t_obs, age_at_obs, label)
         const std::map<std::pair<uint64_t, uint64_t>,
                        std::tuple<bool, double, double, std::string>>& memory,
         double time, double p_block,
         const std::vector<std::string>& class_names,
         const std::vector<double>& class_probs, double prior_residual_mean) {
        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.class_names = class_names;
        model.class_probs = class_probs;
        model.p_block = p_block;
        model.prior_residual_mean = prior_residual_mean;
        model.km = nullptr;

        sp::LocalObservation local;
        local.time = time;
        local.statuses = statuses;
        for (const auto& kv : memory) {
          sp::EdgeMemoryRec rec;
          rec.blocked = std::get<0>(kv.second);
          rec.t_obs = std::get<1>(kv.second);
          rec.age_at_obs = std::get<2>(kv.second);
          rec.label = std::get<3>(kv.second);
          local.memory[kv.first] = rec;
        }
        const auto bs = sp::blockedVertexSet(local, model);

        py::dict out;
        std::vector<std::pair<uint64_t, uint64_t>> edges(bs.edges.begin(),
                                                         bs.edges.end());
        std::vector<uint64_t> nodes(bs.nodes.begin(), bs.nodes.end());
        out["edges"] = edges;
        out["nodes"] = nodes;
        return out;
      },
      py::arg("statuses"), py::arg("memory"), py::arg("time"),
      py::arg("p_block"), py::arg("class_names"), py::arg("class_probs"),
      py::arg("prior_residual_mean") = 60.0);

  m.def(
      "memory_alive_prob",
      [](bool blocked, double t_obs, double age_at_obs,
         const std::string& label, double t_now,
         const std::vector<std::string>& class_names,
         const std::vector<double>& class_probs, double prior_residual_mean) {
        namespace sp = vtr::navigation::sparrow;
        sp::SparrowModel model;
        model.class_names = class_names;
        model.class_probs = class_probs;
        model.prior_residual_mean = prior_residual_mean;
        model.km = nullptr;
        sp::EdgeMemoryRec rec;
        rec.blocked = blocked;
        rec.t_obs = t_obs;
        rec.age_at_obs = age_at_obs;
        rec.label = label;
        return sp::memoryAliveProb(rec, t_now, model);
      },
      py::arg("blocked"), py::arg("t_obs"), py::arg("age_at_obs"),
      py::arg("label"), py::arg("t_now"), py::arg("class_names"),
      py::arg("class_probs"), py::arg("prior_residual_mean") = 60.0);
}
