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
 * \file sparrow_strategy.cpp
 * \brief SPARROW (POMCP) wait strategy implementation.
 */
#include "vtr_navigation/sparrow_strategy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <sstream>

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace navigation {

using sparrow::GraphContext;
using sparrow::LocalObservation;
using sparrow::ParticleBelief;
using sparrow::PlanResult;
using sparrow::SAction;
using sparrow::SEdge;
using sparrow::SparrowModel;
using sparrow::SparrowSearchSettings;
using sparrow::SparrowSolver;
using sparrow::SVertex;
using sparrow::TransitionModel;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

// ============================================================================
// Construction / persistence (mirrors LearnedStrategy)
// ============================================================================

SparrowStrategy::SparrowStrategy(const WaitStrategyConfig& config)
    : config_(config) {
  if (!config_.learned_data_dir.empty()) {
    survival_stats_file_ = config_.learned_data_dir + "/survival_stats.yaml";
    obstacle_stats_file_ = config_.learned_data_dir + "/obstacle_stats.yaml";
    survival_model_.loadFromFile(survival_stats_file_);
    obstacle_stats_.loadFromFile(obstacle_stats_file_);
  }
  obstacle_stats_.setDefaultTypeWeights(config_.type_weights);
  for (const auto& kv : config_.seed_samples) {
    if (survival_model_.sampleCount(kv.first) < kv.second.size()) {
      survival_model_.addSeedSamples(kv.first, kv.second);
    }
  }
  CLOG(INFO, "navigation")
      << "HSHMAT SparrowStrategy: Initialized. sims="
      << config_.sparrow.num_simulations
      << ", particles=" << config_.sparrow.num_particles
      << ", c_uct=" << config_.sparrow.c_uct
      << ", max_depth=" << config_.sparrow.max_depth
      << ", p_block=" << obstacle_stats_.p_block()
      << ", edges_traversed=" << obstacle_stats_.totalEdgesTraversed()
      << ", episodes=" << obstacle_stats_.totalObstacleEpisodes()
      << ", data_dir=" << config_.learned_data_dir;
}

void SparrowStrategy::saveData() {
  if (config_.learned_data_dir.empty()) return;
  survival_model_.saveToFile(survival_stats_file_);
  obstacle_stats_.saveToFile(obstacle_stats_file_);
}

// ============================================================================
// Model snapshot (ports model.LearnedModelEstimator.snapshot)
// ============================================================================

SparrowModel SparrowStrategy::snapshotModel(int num_edges) const {
  SparrowModel m;
  m.km = &survival_model_;
  m.num_edges = std::max(1, num_edges);
  m.prior_residual_mean = 60.0;  // KM_ZERO_DATA_MEAN_S (OSCAR parity)

  // Type distribution: learned when available, config defaults otherwise
  // (GlobalObstacleStats falls back to default_type_weights internally).
  auto type_dist = obstacle_stats_.getTypeDistribution();
  if (type_dist.empty()) type_dist = config_.type_weights;
  double total = 0.0;
  for (const auto& kv : type_dist) total += std::max(0.0, kv.second);
  for (const auto& kv : type_dist) {
    m.class_names.push_back(kv.first);
    m.class_probs.push_back(total > 0.0 ? std::max(0.0, kv.second) / total
                                        : 1.0 / type_dist.size());
    if (survival_model_.hasData(kv.first)) {
      m.fitted_classes.insert(kv.first);
    }
  }

  // Occupancy: empirical (obstacle episodes / edges traversed), optionally
  // pinned via config for experiments/debugging.
  m.p_block = (config_.sparrow.p_block_override >= 0.0)
                  ? config_.sparrow.p_block_override
                  : obstacle_stats_.p_block();
  m.p_block = std::max(0.0, std::min(1.0, m.p_block));

  // Spawn rate from Little's law so the model's steady state matches the
  // learned occupancy: lambda = p_block * num_edges / E[R].
  double mixture_mean = 0.0;
  for (size_t i = 0; i < m.class_names.size(); ++i) {
    const double mean =
        m.is_fitted(m.class_names[i])
            ? std::max(0.0, survival_model_.meanSurvivalTime(
                                m.class_names[i],
                                config_.getWMax(m.class_names[i])))
            : m.prior_residual_mean;
    mixture_mean += m.class_probs[i] * mean;
  }
  m.spawn_rate_hz =
      (mixture_mean > 1e-9) ? m.p_block * m.num_edges / mixture_mean : 0.0;
  return m;
}

double SparrowStrategy::freshEdgeExpectedWait() const {
  // p_block * sum_types(p_type * mean_duration) - mirrors LearnedStrategy
  // so the Navigator's reroute TDSP prices edges identically.
  auto type_dist = obstacle_stats_.getTypeDistribution();
  if (type_dist.empty()) type_dist = config_.type_weights;
  double expected = 0.0;
  for (const auto& kv : type_dist) {
    const double mean = survival_model_.hasData(kv.first)
                            ? survival_model_.meanSurvivalTime(
                                  kv.first, config_.getWMax(kv.first))
                            : 60.0;
    expected += kv.second * mean;
  }
  const double p_block = (config_.sparrow.p_block_override >= 0.0)
                             ? config_.sparrow.p_block_override
                             : obstacle_stats_.p_block();
  return std::max(0.0, p_block) * expected;
}

// ============================================================================
// Graph context from the privileged (teach) graph
// ============================================================================

GraphContext SparrowStrategy::buildContext(
    const tactic::VertexId& current_vertex,
    const tactic::VertexId& goal_vertex) const {
  std::map<SVertex, std::vector<SVertex>> neighbors;
  std::map<SEdge, double> travel_time;

  // BFS over the privileged graph from both seeds (they are connected on a
  // teach graph, but seed both defensively).
  std::deque<tactic::VertexId> queue;
  std::set<uint64_t> visited;
  constexpr size_t kMaxVertices = 200000;  // safety valve

  for (const auto& seed : {current_vertex, goal_vertex}) {
    if (seed.isValid() && !visited.count(static_cast<uint64_t>(seed))) {
      visited.insert(static_cast<uint64_t>(seed));
      queue.push_back(seed);
    }
  }

  while (!queue.empty() && visited.size() < kMaxVertices) {
    const tactic::VertexId v = queue.front();
    queue.pop_front();
    const SVertex vu = static_cast<uint64_t>(v);
    auto nbrs = get_neighbors_(v);
    for (const auto& n : nbrs) {
      const SVertex nu = static_cast<uint64_t>(n);
      const double tt = get_travel_time_(tactic::EdgeId(v, n));
      if (!std::isfinite(tt) || tt <= 0.0) continue;
      neighbors[vu].push_back(nu);
      travel_time[sparrow::canonical_edge(vu, nu)] = tt;
      if (!visited.count(nu)) {
        visited.insert(nu);
        queue.push_back(n);
      }
    }
  }

  return GraphContext(std::move(neighbors), std::move(travel_time),
                      static_cast<uint64_t>(goal_vertex));
}

// ============================================================================
// The decision
// ============================================================================

WaitDecision SparrowStrategy::computeWaitTime(
    const std::string& obs_type, const EdgeIdSet& blocked_edges,
    const tactic::VertexId& current_vertex, const tactic::VertexId& goal_vertex,
    double t_now, double obstacle_t_first) {
  const auto& sp = config_.sparrow;

  if (!get_neighbors_ || !get_travel_time_) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: No graph access, defaulting to wait";
    return WaitDecision::waitForever(obs_type + ". Waiting.");
  }
  if (blocked_edges.empty()) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: No blocked edges reported, waiting";
    return WaitDecision::waitForever(obs_type + ". Waiting.");
  }

  const auto t_start = std::chrono::steady_clock::now();

  // ---- 1. Graph context --------------------------------------------------
  GraphContext ctx = buildContext(current_vertex, goal_vertex);
  const SVertex current_u = static_cast<uint64_t>(current_vertex);
  if (ctx.neighbors.empty() || !std::isfinite(ctx.heuristic_cost_to_go(current_u))) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: Graph context unusable (verts="
        << ctx.neighbors.size() << "), defaulting to wait";
    return WaitDecision::waitForever(obs_type + ". Waiting.");
  }

  std::set<SEdge> blocked_s;
  for (const auto& e : blocked_edges) blocked_s.insert(toSEdge(e));

  // ---- 2. Planning vertex: nearest endpoint of the nearest blocked edge ---
  // The robot may be several micro-edges behind the blocked edge; the wait /
  // detour decision is taken at the vertex the obstacle is incident to. The
  // travel cost to get there is common to every action, so it drops out.
  std::map<SVertex, double> dist_from_robot;
  {
    using QItem = std::pair<double, SVertex>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> heap;
    dist_from_robot[current_u] = 0.0;
    heap.push({0.0, current_u});
    while (!heap.empty()) {
      auto [d, v] = heap.top();
      heap.pop();
      if (d > dist_from_robot[v] + 1e-9) continue;
      auto nit = ctx.neighbors.find(v);
      if (nit == ctx.neighbors.end()) continue;
      for (const auto& w : nit->second) {
        const double nd = d + ctx.edge_time(v, w);
        auto it = dist_from_robot.find(w);
        if (it == dist_from_robot.end() || nd < it->second - 1e-9) {
          dist_from_robot[w] = nd;
          heap.push({nd, w});
        }
      }
    }
  }
  SVertex planning_vertex = current_u;
  {
    double best = kInf;
    for (const auto& e : blocked_s) {
      for (const SVertex endpoint : {e.first, e.second}) {
        auto it = dist_from_robot.find(endpoint);
        if (it != dist_from_robot.end() && it->second < best) {
          best = it->second;
          planning_vertex = endpoint;
        }
      }
    }
    if (!std::isfinite(best)) {
      CLOG(WARNING, "navigation")
          << "HSHMAT SparrowStrategy: Blocked edges unreachable in context; "
             "planning from current vertex";
      planning_vertex = current_u;
    }
  }

  // ---- 3. Label, age, memory update ---------------------------------------
  const std::string label =
      (obs_type.empty() || obs_type == "unknown") ? std::string() : obs_type;
  const double age = (obstacle_t_first > 0.0 && t_now > obstacle_t_first)
                         ? (t_now - obstacle_t_first)
                         : 0.0;
  // Fresh encounter (first plan for this obstacle): no VLM calls made yet.
  if (obstacle_t_first <= 0.0) {
    observed_edges_.clear();
    pending_observe_edge_.reset();
  }
  // Route an on-demand VLM answer to the specific edge that was observed.
  // The label applies to the detector's blocked edges only when they are what
  // was observed (or when it came from an auto-classifying flow with no
  // pending Observe).
  std::optional<SEdge> observed_edge;
  if (pending_observe_edge_) {
    observed_edge = *pending_observe_edge_;
    pending_observe_edge_.reset();
    if (!label.empty()) {
      auto& mem = memory_[*observed_edge];
      if (mem.t_first <= 0.0) mem.t_first = t_now;
      mem.t_last = t_now;
      mem.label = label;
    }
  }
  const bool label_is_front =
      !observed_edge.has_value() || blocked_s.count(*observed_edge) > 0;
  // Re-sighting handling: if an edge we remember as blocked is seen blocked
  // again after a gap, do NOT pretend it was watched the whole time. The old
  // sighting is exported to the belief's memory records so the particle
  // installer runs its same-obstacle-vs-fresh-obstacle mixture; the current
  // streak restarts and any old label becomes probabilistic (it conditions
  // the mixture but no longer forces the class).
  std::map<SEdge, sparrow::EdgeMemoryRec> resight_records;
  auto handle_resight = [&](const SEdge& e, MemEntry& mem,
                            double streak_start) {
    if (mem.t_last > 0.0 &&
        mem.t_last < streak_start - config_.sparrow.resight_gap_s) {
      sparrow::EdgeMemoryRec rec;
      rec.blocked = true;
      rec.t_obs = mem.t_last;
      rec.age_at_obs = std::max(0.0, mem.t_last - mem.t_first);
      rec.label = mem.label;
      resight_records[e] = rec;
      mem.t_first = streak_start;
      mem.label.clear();
      return true;
    }
    return false;
  };
  for (const auto& e : blocked_s) {
    auto& mem = memory_[e];
    const double first = t_now - age;
    const bool resight = handle_resight(e, mem, first);
    if (mem.t_first <= 0.0) {
      mem.t_first = first;
    } else if (!resight && first < mem.t_first) {
      mem.t_first = first;
    }
    mem.t_last = t_now;
    if (!label.empty() && label_is_front) mem.label = label;
  }

  // ---- 4. Local observation ------------------------------------------------
  LocalObservation local;
  local.vertex = planning_vertex;
  local.time = t_now;
  for (const auto& e : blocked_s) {
    if (!ctx.travel_time.count(e)) continue;  // outside context
    local.statuses[e] = 1;
    local.ages[e] = age;
    // memory_ holds the per-edge label (routed there from the VLM answer).
    const auto mit = memory_.find(e);
    if (mit != memory_.end() && !mit->second.label.empty())
      local.labels[e] = mit->second.label;
  }
  int n_free = 0, n_unknown = 0;
  if (adjacent_status_fn_) {
    const auto adj =
        adjacent_status_fn_(tactic::VertexId(uint64_t(planning_vertex)));
    for (const auto& kv : adj) {
      const SEdge se = toSEdge(kv.first);
      if (local.statuses.count(se)) continue;  // blocked set wins
      if (kv.second == 0) {
        local.statuses[se] = 0;
        ++n_free;
        // A confirmed-free sighting invalidates any stale blocked memory.
        memory_.erase(se);
      } else if (kv.second == 1) {
        local.statuses[se] = 1;
        // Track the sighting so re-plans know how long this edge has been
        // seen blocked, and carry any label a previous Observe produced.
        // After a gap this is a RE-sighting: streak restarts at age 0 and the
        // old record goes to the belief's mixture instead.
        auto& mem = memory_[se];
        handle_resight(se, mem, t_now);
        if (mem.t_first <= 0.0) mem.t_first = t_now;
        mem.t_last = t_now;
        local.ages[se] = std::max(0.0, t_now - mem.t_first);
        if (!mem.label.empty()) local.labels[se] = mem.label;
        ++n_unknown;  // blocked adjacent edge (counted for the log only)
      }
      // -1 (unknown): leave unobserved; belief samples the prior.
    }
  }
  for (const auto& kv : memory_) {
    if (local.statuses.count(kv.first)) continue;
    if (!ctx.travel_time.count(kv.first)) continue;
    sparrow::EdgeMemoryRec rec;
    rec.blocked = true;
    rec.t_obs = kv.second.t_last;
    rec.age_at_obs = std::max(0.0, kv.second.t_last - kv.second.t_first);
    rec.label = kv.second.label;
    local.memory[kv.first] = rec;
  }
  // Re-sighted edges: currently observed blocked (in statuses) AND carrying an
  // old sighting record - the belief's install_tracked_blockage mixes
  // "same obstacle survived the gap" vs "cleared and a new one spawned".
  for (const auto& kv : resight_records) {
    if (!ctx.travel_time.count(kv.first)) continue;
    local.memory[kv.first] = kv.second;
  }

  // ---- 5. Frozen model + belief ---------------------------------------------
  const SparrowModel model = snapshotModel(static_cast<int>(ctx.edges.size()));
  const uint64_t seed =
      (sp.planner_seed != 0)
          ? sparrow::derive_seed(static_cast<uint64_t>(sp.planner_seed),
                                 ++plan_counter_)
          : sparrow::derive_seed(
                static_cast<uint64_t>(
                    std::chrono::steady_clock::now().time_since_epoch().count()),
                ++plan_counter_);
  ParticleBelief belief(&ctx, &model, sp.num_particles,
                        /*planner_no_adjacent_blocking=*/false, seed);
  belief.initialize(local);

  // ---- 6. Root actions ------------------------------------------------------
  // Unknown adjacent edges are traversable at the root: the robot cannot rule
  // them out, and the belief prices the risk of them being blocked.
  std::map<SEdge, bool> root_statuses;
  auto iit = ctx.incident_edges.find(planning_vertex);
  if (iit != ctx.incident_edges.end()) {
    for (const auto& e : iit->second) {
      auto sit = local.statuses.find(e);
      root_statuses[e] = (sit != local.statuses.end()) ? (sit->second == 1)
                                                       : false;
    }
  }
  static const std::vector<SVertex> kNoNbrs;
  auto nit = ctx.neighbors.find(planning_vertex);
  const auto& pv_nbrs = (nit == ctx.neighbors.end()) ? kNoNbrs : nit->second;
  // HSHMAT: Observe IS a root action - this is the whole point of SPARROW:
  // the VLM is only called when the POMCP decides a classification is worth
  // delta_obs_s, not on every detection. ANY adjacent blocked edge (statuses
  // come from the lidar costmap + teach graph) can be observed, except edges
  // that already have a label or already used their one VLM call this
  // encounter.
  std::set<SEdge> root_classified(observed_edges_);
  for (const auto& kv : local.labels) root_classified.insert(kv.first);
  const auto root_actions =
      sparrow::valid_actions(planning_vertex, root_statuses, pv_nbrs,
                             root_classified, sp.wait_durations,
                             /*allow_observe=*/sp.allow_observe);
  if (root_actions.empty()) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: No root actions at vertex "
        << planning_vertex << ", waiting";
    return WaitDecision::waitForever(obs_type + ". Waiting.");
  }

  // ---- 7. Search -------------------------------------------------------------
  TransitionModel transition(&ctx, sp.delta_obs_s, sp.wait_durations,
                             sp.allow_observe, sp.corridor_traversal,
                             sp.duration_bin_width);
  SparrowSearchSettings settings;
  settings.num_simulations = sp.num_simulations;
  settings.max_planning_time_s = sp.max_planning_time_s;
  settings.max_depth = sp.max_depth;
  settings.max_sim_time_s = sp.max_sim_time_s;
  settings.c_uct = sp.c_uct;
  settings.duration_bin_width = sp.duration_bin_width;
  SparrowSolver solver(&transition, settings, seed);
  const PlanResult result = solver.plan(belief.particles(), root_actions);

  const double total_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start)
          .count();

  // ---- 8. Log + map to WaitDecision -----------------------------------------
  {
    std::ostringstream ss;
    ss << "HSHMAT SparrowStrategy: encounter @v=" << planning_vertex
       << " type='" << obs_type << "' age=" << age << "s"
       << " | ctx: " << ctx.neighbors.size() << " verts, " << ctx.edges.size()
       << " edges | model: p_block=" << model.p_block
       << ", lambda=" << model.spawn_rate_hz << "/s"
       << " | obs: " << blocked_s.size() << " blocked, " << n_free << " free, "
       << n_unknown << " extra-blocked, memory=" << local.memory.size()
       << " | search: " << result.simulations << " sims, "
       << result.tree_nodes << " nodes, depth " << result.tree_depth << ", "
       << result.planning_time_s << "s (total " << total_time << "s)";
    CLOG(INFO, "navigation") << ss.str();
    for (const auto& s : result.root_actions) {
      CLOG(INFO, "navigation")
          << "HSHMAT SparrowStrategy:   " << s.action.str()
          << "  visits=" << s.visits << "  Q=" << s.q_cost << "s";
    }
  }

  if (!result.action.has_value()) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: Search returned no action, waiting";
    return WaitDecision::waitForever(obs_type + ". Waiting.");
  }

  const SAction& best = result.action.value();
  if (best.kind == SAction::OBSERVE) {
    // The POMCP decided the VLM label on this edge is worth its cost. The
    // Navigator requests one classification and calls computeWaitTime again;
    // the answer will be routed to pending_observe_edge_. In the tree Observe
    // costs the constant delta_obs_s; in the real world its cost is simply
    // the measured wall-clock time, absorbed into the obstacle age.
    observed_edges_.insert(best.edge);
    pending_observe_edge_ = best.edge;
    return WaitDecision::observe("Observing obstacle.", best.edge);
  }
  if (best.kind == SAction::MAXWAIT) {
    // Safety valve: cap the cumulative wait across re-plans of one episode.
    const double waited_so_far = age;
    if (sp.max_total_wait_s > 0.0 &&
        waited_so_far + best.W > sp.max_total_wait_s) {
      CLOG(WARNING, "navigation")
          << "HSHMAT SparrowStrategy: max_total_wait_s ("
          << sp.max_total_wait_s << "s) would be exceeded (waited "
          << waited_so_far << "s + W=" << best.W << "s) -> forcing detour";
      return WaitDecision::detour(obs_type + ". Rerouting.");
    }
    std::ostringstream speech;
    speech << obs_type << ". Waiting up to "
           << static_cast<int>(std::llround(best.W)) << " seconds.";
    return WaitDecision::wait(best.W, speech.str());
  }

  // TRAVERSE (or, defensively, anything else): detour. Receding-horizon
  // parity with the simulation: the POMCP committed to ONE action - leaving
  // the planning vertex through best.first_hop. Ban the other corridors out
  // of the planning vertex (except the approach the robot arrives by) so the
  // Navigator's reroute TDSP executes that action; the rest of its route is
  // tentative and gets revised at the next obstacle encounter.
  WaitDecision d = WaitDecision::detour(obs_type + ". Rerouting.");
  if (best.kind == SAction::TRAVERSE) {
    const auto pit = dist_from_robot.find(planning_vertex);
    const double d_pv = (pit != dist_from_robot.end()) ? pit->second : 0.0;
    for (const auto& w : pv_nbrs) {
      if (w == best.first_hop) continue;
      auto dit = dist_from_robot.find(w);
      const bool is_approach =
          dit != dist_from_robot.end() && dit->second < d_pv - 1e-9;
      if (is_approach) continue;  // robot needs this edge to reach the vertex
      d.detour_ban_edges.push_back(
          sparrow::canonical_edge(planning_vertex, w));
    }
    CLOG(INFO, "navigation")
        << "HSHMAT SparrowStrategy: Traverse commits corridor "
        << planning_vertex << "->" << best.first_hop << "; banning "
        << d.detour_ban_edges.size()
        << " alternative corridor entrances for the reroute";
  }
  return d;
}

// ============================================================================
// Learning + memory hooks (mirror LearnedStrategy)
// ============================================================================

void SparrowStrategy::onObstacleCleared(const std::string& obs_type,
                                        double wait_duration, int episode) {
  pending_samples_.push_back({obs_type, wait_duration, false, episode});
  CLOG(INFO, "navigation")
      << "HSHMAT SparrowStrategy: Buffered uncensored sample for " << obs_type
      << ": " << wait_duration << "s, episode=" << episode
      << " (pending=" << pending_samples_.size() << ")";
}

void SparrowStrategy::onRerouteTimeout(const std::string& obs_type,
                                       double wait_duration, int episode) {
  pending_samples_.push_back({obs_type, wait_duration, true, episode});
  CLOG(INFO, "navigation")
      << "HSHMAT SparrowStrategy: Buffered censored sample for " << obs_type
      << ": " << wait_duration << "s, episode=" << episode
      << " (pending=" << pending_samples_.size() << ")";
}

void SparrowStrategy::flushPendingSamplesToKM() {
  if (pending_samples_.empty()) return;
  CLOG(INFO, "navigation")
      << "HSHMAT SparrowStrategy: Flushing " << pending_samples_.size()
      << " pending samples to KM and obstacle stats";
  for (const auto& s : pending_samples_) {
    survival_model_.addSample(s.obs_type, s.duration, s.censored, s.episode);
    obstacle_stats_.recordObstacleEpisode(s.obs_type);
  }
  pending_samples_.clear();
  saveData();
}

void SparrowStrategy::updateMemoryAfterCensoredWait(
    const EdgeIdSet& blocked_edges, double t_after_wait) {
  for (const auto& e : blocked_edges) {
    auto it = memory_.find(toSEdge(e));
    if (it != memory_.end()) {
      it->second.t_last = std::max(it->second.t_last, t_after_wait);
    }
  }
}

void SparrowStrategy::clearMemoryForEdge(const EdgeId& edge) {
  memory_.erase(toSEdge(edge));
}

void SparrowStrategy::resetMemory() { memory_.clear(); }

void SparrowStrategy::notifyEpisodeStart(int episode_idx) {
  if (episode_idx > 0) episode_idx_ = episode_idx;
  CLOG(INFO, "navigation")
      << "HSHMAT SparrowStrategy: notifyEpisodeStart -> episode "
      << episode_idx_;
}

}  // namespace navigation
}  // namespace vtr
