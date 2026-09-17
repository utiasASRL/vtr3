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

namespace {

// HSHMAT SPARROW speech.
//
// OSCAR always classifies before it decides, so every announcement can lead
// with the class ("Chair. Waiting 50 seconds."). SPARROW cannot: it plans with
// an UNLABELED obstacle and only calls the VLM when the POMCP actually picks
// the Observe action, which is roughly a quarter of the time. Feeding the raw
// obs_type into the speech therefore made the robot say the literal word
// "unknown" ("unknown. Rerouting."). These helpers make the class name
// optional: it is spoken when the robot genuinely knows it, and silently
// dropped when it does not.
std::string classPrefix(const std::string& obs_type) {
  if (obs_type.empty() || obs_type == "unknown") return std::string();
  return obs_type + ". ";
}

// "45 seconds" / "1 minute 30 seconds", matching LearnedStrategy's phrasing so
// the two planners sound like the same robot.
std::string spokenDuration(double seconds_d) {
  const int seconds = static_cast<int>(std::llround(seconds_d));
  std::ostringstream ss;
  if (seconds >= 60) {
    const int minutes = seconds / 60;
    const int secs = seconds % 60;
    ss << minutes << " minute" << (minutes > 1 ? "s" : "");
    if (secs != 0) ss << " " << secs << " seconds";
  } else {
    ss << seconds << " second" << (seconds == 1 ? "" : "s");
  }
  return ss.str();
}

// SPARROW's MaxWait is a BUDGET, not a countdown to a reroute: the wait ends
// the moment the edge clears, and when it expires the planner re-plans and may
// well wait again. "Waiting up to N" is the honest description; OSCAR's
// "Waiting N" would promise a commitment SPARROW has not made.
std::string waitSpeech(const std::string& obs_type, double W,
                       bool for_learning) {
  std::ostringstream ss;
  ss << classPrefix(obs_type) << "Waiting up to " << spokenDuration(W);
  if (for_learning) ss << " to learn";
  ss << ".";
  return ss.str();
}

std::string rerouteSpeech(const std::string& obs_type) {
  return classPrefix(obs_type) + "Rerouting.";
}

}  // namespace

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
  // Apply the teach prior here as well as at episode start, so the occupancy
  // estimate is correct even for a decision taken before the first
  // notifyEpisodeStart. It runs AFTER loadFromFile, and applyTeachPrior seeds
  // only a cold start, so a resumed deployment keeps its banked counters.
  if (config_.sparrow.teach_prior > 0) {
    obstacle_stats_.applyTeachPrior(config_.sparrow.teach_prior);
  }
  // Seed the KM fits on a COLD start only: a class whose banked data already
  // has at least as many samples as the seed lists keeps its own.
  {
    std::set<std::string> seeded_types;
    for (const auto& kv : config_.seed_samples) seeded_types.insert(kv.first);
    for (const auto& kv : config_.seed_samples_censored)
      seeded_types.insert(kv.first);
    for (const auto& type : seeded_types) {
      const auto u_it = config_.seed_samples.find(type);
      const auto c_it = config_.seed_samples_censored.find(type);
      const size_t n_u =
          (u_it != config_.seed_samples.end()) ? u_it->second.size() : 0;
      const size_t n_c = (c_it != config_.seed_samples_censored.end())
                             ? c_it->second.size()
                             : 0;
      if (survival_model_.sampleCount(type) >= n_u + n_c) continue;
      if (n_u > 0) survival_model_.addSeedSamples(type, u_it->second);
      for (double d : (n_c > 0 ? c_it->second : std::vector<double>{}))
        survival_model_.addSample(type, d, /*censored=*/true);
      CLOG(INFO, "navigation")
          << "HSHMAT SparrowStrategy: seeded '" << type << "' with " << n_u
          << " uncensored + " << n_c << " censored samples -> E[T] = "
          << survival_model_.meanSurvivalTime(type, config_.getWMax(type))
          << "s, S(last event) = "
          << survival_model_.survival(
                 type, u_it != config_.seed_samples.end() && n_u > 0
                           ? *std::max_element(u_it->second.begin(),
                                               u_it->second.end())
                           : 0.0);
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
                  : (config_.sparrow.p_block_live
                         ? obstacle_stats_.pBlockJeffreys()
                         : obstacle_stats_.p_block());
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
// Sighting streaks + continuous monitoring
// ============================================================================

bool SparrowStrategy::noteBlockedSighting(const SEdge& e, double streak_start,
                                          double t_now) {
  auto& mem = memory_[e];
  // A focus edge is one the robot is parked in front of, waiting on. Such an
  // edge drops out of the costmap corridor check now and then (hysteresis at
  // the edge of the view), and calling that a re-sighting archives the streak
  // and restarts t_first - so a chair the robot has been staring at for a
  // minute reads as brand new and "wait a little longer" keeps winning. Hold
  // the streak open for focus edges; a genuine departure arrives as status 0
  // (confirmed free), which the caller handles.
  const bool pinned = config_.sparrow.pin_wait_edge_continuity &&
                      wait_focus_edges_.count(e) > 0;
  const bool resight =
      !pinned && mem.t_last > 0.0 &&
      mem.t_last < streak_start - config_.sparrow.monitor_gap_s;
  if (resight) {
    // The edge left view and is blocked again: archive the old streak for the
    // belief's same-obstacle-vs-new-obstacle mixture and restart. Any old
    // label becomes probabilistic (conditions the mixture, no longer forced).
    sparrow::EdgeMemoryRec rec;
    rec.blocked = true;
    rec.t_obs = mem.t_last;
    rec.age_at_obs = std::max(0.0, mem.t_last - mem.t_first);
    rec.label = mem.label;
    archived_sightings_[e] = rec;
    mem.t_first = streak_start;
    mem.label.clear();
  }
  if (mem.t_first <= 0.0) {
    mem.t_first = streak_start;
  } else if (!resight) {
    // e.g. the detector reports an age implying an earlier streak start.
    mem.t_first = std::min(mem.t_first, streak_start);
  }
  mem.t_last = std::max(mem.t_last, t_now);
  return resight;
}

void SparrowStrategy::updateEdgeMonitoring(
    const std::map<tactic::EdgeId, int>& statuses, double t_now) {
  // Never stall the sensor callback behind a plan in flight: monitoring is
  // periodic, so skipping one pass while the search runs is harmless.
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  for (const auto& kv : statuses) {
    const SEdge se = toSEdge(kv.first);
    if (kv.second == 0) {
      // Confirmed free: any remembered obstacle is gone and the archived
      // record is obsolete (the next blockage is a fresh encounter).
      // Erasing a remembered blockage IS a belief revision (a route through
      // this edge may just have become viable).
      if (memory_.erase(se) > 0) {
        ++belief_revision_;
        // A remembered blockage clearing always matters to a wait in
        // progress: it is either the edge being waited on, or an edge a
        // detour could now use.
        ++wait_focus_revision_;
        CLOG(INFO, "navigation")
            << "HSHMAT SparrowStrategy: monitored edge (" << se.first << ","
            << se.second << ") transitioned BLOCKED->free (belief revision "
            << belief_revision_.load() << ")";
      }
      archived_sightings_.erase(se);
    } else if (kv.second == 1) {
      const bool was_known = memory_.count(se) > 0;
      const bool resight = noteBlockedSighting(se, t_now, t_now);
      // New blockage or re-sighting revises the belief; an ongoing
      // "still blocked" confirmation does not (its aging effect is
      // deterministic and priced at decision epochs).
      if (!was_known || resight) {
        ++belief_revision_;
        // A new blockage somewhere else on the graph does not change the
        // wait-vs-detour question for the edge we are parked at; it is priced
        // at the next decision epoch.
        if (wait_focus_edges_.count(se) > 0) ++wait_focus_revision_;
        CLOG(INFO, "navigation")
            << "HSHMAT SparrowStrategy: monitored edge (" << se.first << ","
            << se.second << ") "
            << (resight ? "RE-sighted blocked after a gap"
                        : "transitioned free->BLOCKED")
            << " (belief revision " << belief_revision_.load() << ")";
      }
    }
    // -1 (unknown / out of view): no update; the streak gap grows naturally.
  }
}

void SparrowStrategy::setWaitFocusEdges(const std::vector<SEdge>& edges) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  wait_focus_edges_.clear();
  wait_focus_edges_.insert(edges.begin(), edges.end());
}

std::vector<std::pair<uint64_t, uint64_t>>
SparrowStrategy::rememberedBlockedEdges() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<std::pair<uint64_t, uint64_t>> out;
  out.reserve(memory_.size());
  for (const auto& kv : memory_) out.push_back(kv.first);
  return out;
}

// ============================================================================
// The decision
// ============================================================================

WaitDecision SparrowStrategy::computeWaitTime(
    const std::string& obs_type, const EdgeIdSet& blocked_edges,
    const tactic::VertexId& current_vertex, const tactic::VertexId& goal_vertex,
    double t_now, double obstacle_t_first) {
  if (blocked_edges.empty()) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: No blocked edges reported, waiting";
    return WaitDecision::waitForever(classPrefix(obs_type) + "Waiting.");
  }
  return planInternal(obs_type, blocked_edges, current_vertex, goal_vertex,
                      t_now, obstacle_t_first, tactic::VertexId::Invalid(),
                      /*route_next_hop=*/0);
}

WaitDecision SparrowStrategy::planEnRoute(const tactic::VertexId& robot_vertex,
                                          const tactic::VertexId& junction_vertex,
                                          const tactic::VertexId& goal_vertex,
                                          double t_now,
                                          uint64_t route_next_hop) {
  // Every-decision replanning: no front blockage; blocked information comes
  // from the adjacent-status hook at the junction and remembered sightings.
  return planInternal("unknown", EdgeIdSet{}, robot_vertex, goal_vertex, t_now,
                      /*obstacle_t_first=*/0.0, junction_vertex,
                      route_next_hop);
}

WaitDecision SparrowStrategy::planInternal(
    const std::string& obs_type, const EdgeIdSet& blocked_edges,
    const tactic::VertexId& current_vertex, const tactic::VertexId& goal_vertex,
    double t_now, double obstacle_t_first,
    const tactic::VertexId& plan_vertex_override, uint64_t route_next_hop) {
  // Serialize against the costmap-driven monitoring updates.
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto& sp = config_.sparrow;

  if (!get_neighbors_ || !get_travel_time_) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: No graph access, defaulting to wait";
    return WaitDecision::waitForever(classPrefix(obs_type) + "Waiting.");
  }

  const auto t_start = std::chrono::steady_clock::now();

  // ---- 1. Graph context --------------------------------------------------
  GraphContext ctx = buildContext(current_vertex, goal_vertex);
  const SVertex current_u = static_cast<uint64_t>(current_vertex);
  if (ctx.neighbors.empty() || !std::isfinite(ctx.heuristic_cost_to_go(current_u))) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: Graph context unusable (verts="
        << ctx.neighbors.size() << "), defaulting to wait";
    return WaitDecision::waitForever(classPrefix(obs_type) + "Waiting.");
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
  if (plan_vertex_override.isValid()) {
    // Junction replanning: the decision epoch is the upcoming decision
    // vertex, planned for while the robot is still driving toward it.
    const SVertex ov = static_cast<uint64_t>(plan_vertex_override);
    if (ctx.neighbors.count(ov)) {
      planning_vertex = ov;
    } else {
      CLOG(WARNING, "navigation")
          << "HSHMAT SparrowStrategy: plan vertex override " << ov
          << " not in context; planning from current vertex";
    }
  } else if (sp.plan_at_robot) {
    // Decide where the robot actually is. It has already stopped; its real
    // options are to wait here or to turn around, and only a root at the robot
    // offers both.
    planning_vertex = current_u;
  } else {
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
    if (!blocked_s.empty() && !std::isfinite(best)) {
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
  // NOTE: which edge was observed no longer gates whether the class is
  // applied - see the loop below. One encounter is one obstacle.
  const bool label_is_front =
      !observed_edge.has_value() || blocked_s.count(*observed_edge) > 0;
  (void)label_is_front;
  // Sighting-streak update for the detector's blocked edges. Re-sightings
  // (edge left view since its last confirmation) are detected inside
  // noteBlockedSighting: the old streak is archived for the belief's
  // same-obstacle-vs-new-obstacle mixture and the streak restarts. Because
  // the Navigator's continuous monitoring refreshes t_last at ~1 Hz while an
  // edge is in costmap view, a gap here means the robot genuinely looked
  // away - not merely that a wait cycle passed between plans.
  for (const auto& e : blocked_s) {
    const bool resight = noteBlockedSighting(e, t_now - age, t_now);
    // A known class sticks to the obstacle for as long as the sighting streak
    // is unbroken, and every particle is then installed with it
    // (install_particle forces obs_type when a label is present), so the
    // belief cannot drift back to "might be a person" while the robot is
    // staring at a chair.
    //
    // It applies to EVERY edge this obstacle blocks, not just the one the VLM
    // was pointed at. The POMCP usually picks an adjacent edge to observe, so
    // gating on that left the detector's blocked edges unlabelled: at
    // sparrow_test3 23:07 the robot announced "chair" and then waited 120 s
    // from the SHARED grid, because the edges it was waiting on carried no
    // class and the per-class grid [13,39,104] never applied.
    //
    // But not across a break in the streak. noteBlockedSighting clears the
    // label when the edge was out of view long enough to be a re-sighting,
    // and re-asserting the episode's class here would defeat that: after a
    // gap this may be a different obstacle, so the class must go back to the
    // mixture. A confirmed-free reading erases the memory entry outright
    // (updateEdgeMonitoring), which covers "cleared, then blocked again".
    if (!label.empty() && !resight) memory_[e].label = label;
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
        archived_sightings_.erase(se);
      } else if (kv.second == 1) {
        local.statuses[se] = 1;
        // Track the sighting so re-plans know how long this edge has been
        // seen blocked, and carry any label a previous Observe produced.
        // After a genuine out-of-view gap this is a RE-sighting: the streak
        // restarts and the old record goes to the belief's mixture instead.
        noteBlockedSighting(se, t_now, t_now);
        const auto& mem = memory_[se];
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
  // archived pre-gap sighting - the belief's install_tracked_blockage mixes
  // "same obstacle survived the gap" vs "cleared and a new one spawned".
  // Archived records persist across replans of the same streak so every
  // fresh belief initialization applies the same mixture.
  for (const auto& kv : archived_sightings_) {
    auto sit = local.statuses.find(kv.first);
    if (sit == local.statuses.end() || sit->second != 1) continue;
    if (!ctx.travel_time.count(kv.first)) continue;
    local.memory[kv.first] = kv.second;
  }

  // ---- 5. Frozen model --------------------------------------------------------
  const SparrowModel model = snapshotModel(static_cast<int>(ctx.edges.size()));

  // ---- 5b. Knowledge-gradient exploration (paper Sec. IV-D) -------------------
  // Ports the runner's macro machinery: at real decision points, price the
  // value one more KM sample has for future episodes; when it beats its
  // immediate cost, commit to Observe -> MaxWait(W*) and override the search.
  macro_wait_active_ = false;  // any re-plan ends a running macro wait
  // Episode-zero guard mirrors the sim (episode 0's samples are discarded);
  // episode_idx_ is the 1-based mission number, sim indices are 0-based.
  const bool kg_on =
      sp.kg_enabled && sp.kg_num_episodes > 0 && episode_idx_ > 1;
  if (kg_on) {
    if (!explorer_) {
      sparrow::KgConfig kc;
      kc.enabled = true;
      kc.num_episodes = sp.kg_num_episodes;
      kc.encounter_rate_prior = sp.kg_encounter_rate_prior;
      kc.mean_uncertainty = sp.kg_mean_uncertainty;
      kc.detour_cost = sp.kg_detour_cost;
      kc.value_mode = sp.kg_value_mode;
      kc.horizon_cap = sp.kg_horizon_cap;
      kc.value_scale = sp.kg_value_scale;
      kc.censored_credit = sp.kg_censored_credit;
      kc.w_max_s = sp.kg_w_max_s;
      kc.delta_obs_exec_s = sp.delta_obs_s;
      kc.wait_durations = sp.wait_durations;
      explorer_ = std::make_unique<sparrow::KmExplorer>(kc);
    }
    const int ep0 = episode_idx_ - 1;  // 0-based, as in the sim
    explorer_->beginEpisode(&ctx, &model, ep0, /*episodes_done=*/ep0,
                            obstacle_stats_.totalObstacleEpisodes());

    // Largest learning wait allowed (the sim reserves mission budget; the
    // robot's analog is the per-episode cumulative wait cap).
    double w_cap = explorer_->maxWMax();
    if (sp.max_total_wait_s > 0.0)
      w_cap = std::min(w_cap, sp.max_total_wait_s - age);

    // Candidate encounters: adjacent blocked edges whose encounter has not
    // already run its macro (consumed is keyed on the streak start t_first,
    // so a NEW obstacle on the same edge is eligible again).
    std::vector<sparrow::KmExplorer::Candidate> candidates;
    for (const auto& kv : local.statuses) {
      if (kv.second != 1) continue;
      auto mit = memory_.find(kv.first);
      const double t_first =
          (mit != memory_.end()) ? mit->second.t_first : t_now;
      auto cit = kg_consumed_.find(kv.first);
      if (cit != kg_consumed_.end() && cit->second == t_first) continue;
      const auto ait = local.ages.find(kv.first);
      const auto lit = local.labels.find(kv.first);
      candidates.emplace_back(
          kv.first, (ait != local.ages.end()) ? ait->second : 0.0,
          (lit != local.labels.end()) ? lit->second : std::string());
    }

    if (macro_stage_ == MacroStage::kObserve) {
      // The committed Observe has been answered: re-run the optimisation
      // with the posterior collapsed to the revealed class.
      const auto sit = local.statuses.find(macro_edge_);
      const bool still_blocked =
          sit != local.statuses.end() && sit->second == 1;
      std::string lbl;
      const auto lit = local.labels.find(macro_edge_);
      if (lit != local.labels.end()) lbl = lit->second;
      macro_stage_ = MacroStage::kNone;
      if (still_blocked && !lbl.empty() && w_cap > 0.0) {
        const auto mit = memory_.find(macro_edge_);
        const double t_first =
            (mit != memory_.end()) ? mit->second.t_first : t_now;
        const auto ait = local.ages.find(macro_edge_);
        const auto follow = explorer_->evaluateEdge(
            planning_vertex, macro_edge_,
            (ait != local.ages.end()) ? ait->second : 0.0, lbl, w_cap);
        // Either way the encounter is consumed: it got its one macro.
        kg_consumed_[macro_edge_] = t_first;
        if (follow.has_value()) {
          // The revealed class still has positive net learning value:
          // commit the wait sized for it.
          macro_wait_active_ = true;
          CLOG(INFO, "navigation")
              << "HSHMAT SparrowStrategy: EXPLORE macro wait | "
              << follow->detail;
          return WaitDecision::wait(
              follow->wait_s,
              waitSpeech(lbl, follow->wait_s, /*for_learning=*/true));
        }
        // Nothing left to learn from this class: release control back to
        // the search; the labeled (possibly censored) sample still lands.
        CLOG(INFO, "navigation")
            << "HSHMAT SparrowStrategy: EXPLORE macro released after "
               "Observe (class '" << lbl << "' has no net learning value)";
      }
      // Cleared while observing: the uncensored sample was already recorded
      // by the clearance hooks; the macro simply completes (not consumed -
      // the encounter is over).
    } else if (!candidates.empty() && w_cap > 0.0) {
      // Known blockages elsewhere make some detours worse than the
      // steady-state term alone suggests.
      explorer_->setLiveBlockages(candidates);
      auto decision = explorer_->propose(planning_vertex, candidates, w_cap);
      // The macro must respect the planner's information structure: without
      // the Observe action there is no way to label the sample, and an
      // unlabeled clearance is dropped from the KM, so it teaches nothing.
      if (decision.has_value() && decision->needs_observe &&
          !sp.allow_observe)
        decision.reset();
      if (decision.has_value()) {
        CLOG(INFO, "navigation")
            << "HSHMAT SparrowStrategy: EXPLORE macro | " << decision->detail;
        if (decision->needs_observe) {
          macro_stage_ = MacroStage::kObserve;
          macro_edge_ = decision->edge;
          observed_edges_.insert(decision->edge);
          pending_observe_edge_ = decision->edge;
          // Exploration macro: the label is being bought to improve the model
          // for FUTURE missions, not to settle this encounter.
          return WaitDecision::observe("Identifying obstacle to learn.",
                                       decision->edge);
        }
        // Already labeled: commit the learning wait directly.
        const auto mit = memory_.find(decision->edge);
        kg_consumed_[decision->edge] =
            (mit != memory_.end()) ? mit->second.t_first : t_now;
        macro_wait_active_ = true;
        return WaitDecision::wait(
            decision->wait_s,
            waitSpeech(obs_type, decision->wait_s, /*for_learning=*/true));
      }
    }
  }

  // ---- 5b2. Contracted planning view (handoff sec. 2) ------------------------
  // The search's horizon is measured in ACTIONS, and the taught graph is mostly
  // degree-2 filler (sep12_4: 645 of 664 vertices), so a micro-edge search
  // cannot see far enough to compare routes. Contract each corridor into one
  // atomic "drive this corridor" action.
  //
  // The world and the executor are untouched: this only changes what the search
  // reasons over. Below, `ctx` and `local` are SWAPPED to the contracted view,
  // so every downstream stage (root actions, belief, transition, solver) works
  // on corridors without further change. `micro_local` keeps the taught-edge
  // view for the reroute bans, which the Navigator needs in micro terms.
  const LocalObservation micro_local = local;
  sparrow::MacroPlan macro_plan;
  bool contracted = false;
  std::map<SEdge, double> macro_weights;
  if (sp.contracted) {
    if (!corner_vertices_parsed_) {
      corner_vertices_ = sparrow::parseCornerSpec(sp.corner_vertices);
      corner_vertices_parsed_ = true;
      CLOG(INFO, "navigation")
          << "HSHMAT SPARROW: " << corner_vertices_.size()
          << " corner vertices kept explicit";
    }
    // Believed-blocked micro-edges stay their own planning vertices, so the
    // search can wait at them or route around them at the right cost; the
    // corners keep a bend from being swallowed into a single long corridor.
    const auto bs = sparrow::blockedVertexSet(local, model);
    // Blocked micro-edges stay their own planning vertices. I briefly dropped
    // this when rooting at the robot, so that the forward corridor would carry
    // the blockage and read BLOCKED (giving the root a MaxWait). That made
    // every corridor MAXIMAL, and on a graph with few junctions both
    // directions out of the robot then contain the blockage - sparrow_test3
    // 23:06, root 11 with "Neighbours: 23=BLOCKED 8589934604=BLOCKED" while
    // the backward MICRO edge {<0,10>,<0,11>} read free in 277 of 285 samples.
    // Splitting at blockages keeps a corridor from being condemned by an
    // obstacle at its far end.
    std::set<SVertex> extra = bs.nodes;
    extra.insert(corner_vertices_.begin(), corner_vertices_.end());

    // Reuse the contraction when the robot is at the same root with the same
    // believed-blocked set: it depends on neither the model nor the time.
    const MacroCacheKey key{planning_vertex, extra};
    if (macro_cache_valid_ && macro_cache_key_ == key) {
      macro_plan = macro_cache_plan_;
    } else {
      macro_plan = sparrow::buildMacroPlan(ctx, planning_vertex, extra,
                                           sp.max_macro_len_s);
      macro_cache_key_ = key;
      macro_cache_plan_ = macro_plan;
      macro_cache_valid_ = true;
    }
    if (macro_plan.context.neighbors.count(planning_vertex) &&
        !macro_plan.macros.empty()) {
      // Per-corridor occupancy: w_e = macroPBlock(p, n) / p, so the belief's
      // p_block * w_e is 1 - (1 - p_micro)^n. A 109-micro-edge corridor is
      // ~73% blocked, not 1.2%; pricing it at the flat micro rate is what made
      // a long trap look safer than a short ladder.
      const double p_micro = std::max(0.0, std::min(1.0, model.p_block));
      for (const auto& kv : macro_plan.macros) {
        const size_t n = kv.second.n_micro();
        macro_weights[kv.first] =
            (p_micro > 0.0)
                ? sparrow::macroPBlock(p_micro, n) / p_micro
                : static_cast<double>(n);
      }

      // Corridor statuses from the lidar: blocked if ANY micro-edge in the
      // corridor is, and while part-way down one, only the part ahead counts.
      LocalObservation ml;
      ml.vertex = planning_vertex;
      ml.time = local.time;
      if (macro_status_fn_) {
        const SVertex robot_at =
            (current_u != planning_vertex) ? current_u : 0;
        for (const auto& kv :
             macro_status_fn_(planning_vertex, macro_plan, robot_at)) {
          if (kv.second >= 0) ml.statuses[kv.first] = kv.second;
        }
      }
      // Age / label / memory ride on the macro-edge that CONTAINS the micro
      // sighting, so a remembered blockage deep in a corridor still prices it.
      for (const auto& kv : macro_plan.macros) {
        const auto chain = sparrow::macroChainFrom(kv.second, kv.second.u);
        for (size_t i = 0; i + 1 < chain.size(); ++i) {
          const SEdge me = sparrow::canonical_edge(chain[i], chain[i + 1]);
          auto ait = local.ages.find(me);
          if (ait != local.ages.end()) {
            auto cur = ml.ages.find(kv.first);
            if (cur == ml.ages.end() || ait->second > cur->second)
              ml.ages[kv.first] = ait->second;
          }
          auto lit = local.labels.find(me);
          if (lit != local.labels.end()) ml.labels[kv.first] = lit->second;
          auto mit2 = local.memory.find(me);
          if (mit2 != local.memory.end() && mit2->second.blocked) {
            auto cur = ml.memory.find(kv.first);
            if (cur == ml.memory.end() ||
                mit2->second.t_obs > cur->second.t_obs)
              ml.memory[kv.first] = mit2->second;
          }
          // A micro-edge KNOWN blocked blocks its whole corridor, even if the
          // costmap hook is unavailable.
          auto sit = local.statuses.find(me);
          if (sit != local.statuses.end() && sit->second == 1)
            ml.statuses[kv.first] = 1;
        }
      }
      ctx = macro_plan.context;
      local = ml;
      contracted = true;
      CLOG(INFO, "navigation")
          << "HSHMAT SPARROW: contracted view rooted at " << planning_vertex
          << ": " << ctx.neighbors.size() << " decision vertices, "
          << macro_plan.macros.size() << " corridors ("
          << bs.edges.size() << " believed-blocked micro-edges kept explicit)";
    } else {
      CLOG(WARNING, "navigation")
          << "HSHMAT SPARROW: contraction produced no usable view at "
          << planning_vertex << "; planning on the micro graph";
    }
  }

  // ---- 5c. Belief -------------------------------------------------------------
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
  // No-op when uncontracted, so the micro path keeps the flat p_block exactly.
  if (contracted) belief.setEdgeWeights(macro_weights);
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
  // The robot stopped BECAUSE of this encounter's blocked edges, and waiting
  // for them to clear is a legitimate action wherever it happens to be
  // standing. But the detector stops it several vertices short of the
  // obstacle, so those edges are usually NOT adjacent to the root - and
  // valid_actions() only offers MaxWait/Observe on adjacent blocked edges.
  // The result was a robot that announced "Obstacle detected" and then
  // "Rerouting" without waiting ever being in the action set at all:
  // sparrow_test3 00:00:10, robot at <0,7>, blockage on <0,0>..<0,2>, root
  // offered Traverse(->8) and Traverse(->2) and nothing else.
  //
  // A corridor that runs INTO one of those edges is shut, whatever its own
  // micro-edges say. Blocked micro-edges are kept as explicit vertices, so
  // such a corridor terminates exactly at the blockage - mark it blocked and
  // the root regains MaxWait/Observe on the way the robot actually wanted to
  // go, while the other corridors stay traversable.
  size_t n_lead_to_block = 0;
  for (auto& kv : root_statuses) {
    if (kv.second) continue;
    const SVertex far = (kv.first.first == planning_vertex) ? kv.first.second
                                                            : kv.first.first;
    for (const auto& be : blocked_s) {
      if (be.first == far || be.second == far) {
        kv.second = true;
        ++n_lead_to_block;
        break;
      }
    }
  }
  if (n_lead_to_block > 0) {
    CLOG(INFO, "navigation")
        << "HSHMAT SparrowStrategy: " << n_lead_to_block
        << " corridor(s) out of " << planning_vertex
        << " run into this encounter's blockage - offering MaxWait/Observe on "
           "them even though the robot has not reached it yet.";
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
  // Root edges that already carry a VLM label get that class's own wait grid,
  // the same rule the in-tree actions use.
  std::map<SEdge, std::string> root_edge_classes;
  if (!sp.wait_durations_by_class.empty()) {
    for (const auto& kv : local.labels) {
      if (!kv.second.empty()) root_edge_classes[kv.first] = kv.second;
    }
    // local.labels is keyed by the MICRO edge the VLM was pointed at, but on
    // the contracted view the root's actions are MACRO edges - so the lookup
    // never hit and a classified obstacle still drew from the shared grid.
    // Worse, the label lands on whichever edge was observed, which is often
    // an adjacent one rather than the detector's: sparrow_test3 23:07 asked
    // about (2:11,2:12), got 'chair', and then announced "chair. Waiting up
    // to 2 minutes" - 120 s from the shared [4,15,45,120] grid, when chair's
    // own grid is [13,39,104] and tops out at 104.
    //
    // One encounter is one obstacle, so its class applies to every blocked
    // edge at the root, whatever the contraction keyed them as.
    if (!label.empty()) {
      for (const SVertex w : pv_nbrs) {
        const SEdge ce = sparrow::canonical_edge(planning_vertex, w);
        auto it = root_statuses.find(ce);
        if (it != root_statuses.end() && it->second) root_edge_classes[ce] = label;
      }
    }
  }
  // One observation per encounter: with a trusted classifier, a second VLM
  // call on another edge of the same obstacle buys nothing and costs real
  // seconds (measured 16-17 s each, while the tree prices them at
  // delta_obs_s).
  const bool observe_budget_left =
      !sp.single_observe_per_encounter || observed_edges_.empty();
  if (!observe_budget_left && sp.allow_observe) {
    CLOG(INFO, "navigation")
        << "HSHMAT SparrowStrategy: observation already spent this encounter ("
        << observed_edges_.size()
        << " edge(s) classified) - Observe withheld from the action set.";
  }
  const auto root_actions = sparrow::valid_actions(
      planning_vertex, root_statuses, pv_nbrs, root_classified,
      sp.wait_durations,
      /*allow_observe=*/sp.allow_observe && observe_budget_left,
      sp.wait_durations_by_class.empty() ? nullptr : &sp.wait_durations_by_class,
      root_edge_classes.empty() ? nullptr : &root_edge_classes);
  if (root_actions.empty()) {
    CLOG(WARNING, "navigation")
        << "HSHMAT SparrowStrategy: No root actions at vertex "
        << planning_vertex << ", waiting";
    return WaitDecision::waitForever(classPrefix(obs_type) + "Waiting.");
  }

  // A root with no TRAVERSE cannot reroute: the only actions are waits and
  // observes, so the episode can only end when the obstacle clears or
  // max_total_wait_s fires. That happens when the planning vertex sits
  // mid-corridor with every neighbour believed blocked - one chair between
  // v17 and v18 on sparrow_test3 also marked (16,17) through the costmap
  // corridor check, which removed the one escape and left the robot waiting
  // out the full safety valve. Say so loudly: it is invisible otherwise,
  // reading only as "the robot kept choosing to wait".
  {
    bool has_traverse = false;
    for (const auto& a : root_actions) {
      if (a.kind == SAction::TRAVERSE) { has_traverse = true; break; }
    }
    if (!has_traverse) {
      std::stringstream ss;
      ss << "HSHMAT SparrowStrategy: NO TRAVERSE ACTION at planning vertex "
         << planning_vertex << " - the search cannot reroute from here and "
            "can only wait/observe until the obstacle clears or "
            "max_total_wait_s (" << sp.max_total_wait_s << "s) fires. "
            "Neighbours:";
      for (SVertex n : pv_nbrs) {
        const SEdge e = sparrow::canonical_edge(planning_vertex, n);
        auto it = root_statuses.find(e);
        const int st = (it == root_statuses.end()) ? -1 : it->second;
        ss << " " << n << "="
           << (st == 0 ? "free" : (st == 1 ? "BLOCKED" : "unknown"));
      }
      ss << ". If a neighbour is blocked only because the costmap corridor "
            "check over-attributed one obstacle to several edges, widen the "
            "planning root or shrink edge_check_length_m.";
      CLOG(WARNING, "navigation") << ss.str();
    }
  }

  // ---- 7. Search -------------------------------------------------------------
  // Macro-edges ARE corridors, so the transition must not collapse chains a
  // second time: a macro node of degree 2 (a corner, say) would otherwise have
  // its two corridors merged into one action, undoing the explicit decision
  // vertex the contraction just created. Ports _ensure_macro's
  // corridor_traversal=False.
  // The tree must not value future Observes that the root will never offer.
  TransitionModel transition(&ctx, sp.delta_obs_s, sp.wait_durations,
                             sp.allow_observe && observe_budget_left,
                             contracted ? false : sp.corridor_traversal,
                             sp.duration_bin_width);
  if (!sp.wait_durations_by_class.empty())
    transition.setWaitSetByClass(sp.wait_durations_by_class);
  SparrowSearchSettings settings;
  settings.num_simulations = sp.num_simulations;
  settings.max_planning_time_s = sp.max_planning_time_s;
  settings.max_depth = sp.max_depth;
  settings.max_sim_time_s = sp.max_sim_time_s;
  settings.c_uct = sp.c_uct;
  settings.root_explore_frac = sp.root_explore_frac;
  settings.root_explore_by_class = sp.root_explore_by_class;
  settings.robust_visit_frac = sp.robust_visit_frac;
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
    return WaitDecision::waitForever(classPrefix(obs_type) + "Waiting.");
  }

  SAction best = result.action.value();
  // Divert hysteresis (junction replanning only): stay on the current route
  // unless the chosen corridor beats the route continuation by a clear
  // margin. Q-values within search noise must not cause route dithering.
  if (route_next_hop != 0 && best.kind == SAction::TRAVERSE &&
      best.first_hop != route_next_hop) {
    double q_best = kInf, q_route = kInf;
    for (const auto& s : result.root_actions) {
      if (s.action.kind != SAction::TRAVERSE) continue;
      if (s.action.first_hop == best.first_hop) q_best = s.q_cost;
      if (s.action.first_hop == route_next_hop) q_route = s.q_cost;
    }
    if (std::isfinite(q_route) &&
        q_route <= q_best + sp.junction_divert_margin_s) {
      CLOG(INFO, "navigation")
          << "HSHMAT SparrowStrategy: divert hysteresis - route continuation "
             "(Q=" << q_route << "s) within " << sp.junction_divert_margin_s
          << "s of best corridor (Q=" << q_best << "s); staying on route";
      best.first_hop = route_next_hop;
    }
  }
  // On the contracted view an action's edge is a CORRIDOR, but everything
  // outside the search - the VLM corridor publisher, the blocked-edge
  // bookkeeping, the wait timers - addresses taught edges. Resolve a macro
  // edge to the micro edge that actually carries the sighting: the blocked
  // one inside that corridor if we know of one, else its first micro hop.
  auto micro_edge_of = [&](const SEdge& e) -> SEdge {
    if (!contracted) return e;
    auto mit = macro_plan.macros.find(e);
    if (mit == macro_plan.macros.end()) return e;
    const auto chain = sparrow::macroChainFrom(mit->second, mit->second.u);
    for (size_t i = 0; i + 1 < chain.size(); ++i) {
      const SEdge me = sparrow::canonical_edge(chain[i], chain[i + 1]);
      auto sit = micro_local.statuses.find(me);
      if (sit != micro_local.statuses.end() && sit->second == 1) return me;
    }
    for (size_t i = 0; i + 1 < chain.size(); ++i) {
      const SEdge me = sparrow::canonical_edge(chain[i], chain[i + 1]);
      if (micro_local.memory.count(me)) return me;
    }
    return chain.size() >= 2 ? sparrow::canonical_edge(chain[0], chain[1]) : e;
  };

  if (best.kind == SAction::OBSERVE) {
    // The POMCP decided the VLM label on this edge is worth its cost. The
    // Navigator requests one classification and calls computeWaitTime again;
    // the answer will be routed to pending_observe_edge_. In the tree Observe
    // costs the constant delta_obs_s; in the real world its cost is simply
    // the measured wall-clock time, absorbed into the obstacle age.
    const SEdge obs_edge = micro_edge_of(best.edge);
    observed_edges_.insert(obs_edge);
    pending_observe_edge_ = obs_edge;
    return WaitDecision::observe("Identifying obstacle.", obs_edge);
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
      return WaitDecision::detour(rerouteSpeech(obs_type));
    }
    return WaitDecision::wait(
        best.W, waitSpeech(obs_type, best.W, /*for_learning=*/false));
  }

  // TRAVERSE (or, defensively, anything else): detour. Receding-horizon
  // parity with the simulation: the POMCP committed to ONE action - leaving
  // the planning vertex through best.first_hop. Ban the other corridors out
  // of the planning vertex (except the approach the robot arrives by) so the
  // Navigator's reroute TDSP executes that action; the rest of its route is
  // tentative and gets revised at the next obstacle encounter.
  WaitDecision d = WaitDecision::detour(rerouteSpeech(obs_type));
  if (best.kind == SAction::TRAVERSE) {
    // On the contracted view best.first_hop is the far END of a corridor, but
    // the Navigator drives taught edges - so translate it back to the first
    // MICRO hop of that corridor. toMicro handles either orientation.
    auto to_micro_hop = [&](SVertex w) -> SVertex {
      if (!contracted) return w;
      const auto hop = macro_plan.toMicro(planning_vertex, w);
      return hop.has_value() ? *hop : w;
    };
    const SVertex commit_hop = to_micro_hop(best.first_hop);
    d.traverse_edge = {planning_vertex, commit_hop};
    const auto pit = dist_from_robot.find(planning_vertex);
    const double d_pv = (pit != dist_from_robot.end()) ? pit->second : 0.0;
    for (const auto& w : pv_nbrs) {
      if (w == best.first_hop) continue;
      auto dit = dist_from_robot.find(w);
      const bool is_approach =
          dit != dist_from_robot.end() && dit->second < d_pv - 1e-9;
      if (is_approach) continue;  // robot needs this edge to reach the vertex
      const SVertex hop = to_micro_hop(w);
      if (hop == commit_hop) continue;  // never ban the committed corridor
      d.detour_ban_edges.push_back(
          sparrow::canonical_edge(planning_vertex, hop));
    }
    // Also ban every edge currently OBSERVED blocked (front + adjacent): the
    // POMCP's Traverse routed around them, so the reroute must too. These come
    // from the MICRO observation - the Navigator's TDSP bans taught edges, and
    // a macro-edge id is not one.
    for (const auto& kv : micro_local.statuses) {
      if (kv.second == 1) d.detour_ban_edges.push_back(kv.first);
    }
    CLOG(INFO, "navigation")
        << "HSHMAT SparrowStrategy: Traverse commits corridor "
        << planning_vertex << "->" << best.first_hop << "; reroute bans "
        << d.detour_ban_edges.size()
        << " edges (alternative corridor entrances + observed blocked)";
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
    // Paper Sec. IV-A: samples enter D_k only when the class is known from
    // Observe; unlabeled encounters count toward occupancy (p_block) but are
    // NOT assigned to a class (no KM sample, no class-mixture count).
    if (s.obs_type.empty() || s.obs_type == "unknown") {
      obstacle_stats_.recordUnlabeledEpisode();
      continue;
    }
    survival_model_.addSample(s.obs_type, s.duration, s.censored, s.episode);
    obstacle_stats_.recordObstacleEpisode(s.obs_type);
  }
  pending_samples_.clear();
  saveData();
}

void SparrowStrategy::updateMemoryAfterCensoredWait(
    const EdgeIdSet& blocked_edges, double t_after_wait) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto& e : blocked_edges) {
    auto it = memory_.find(toSEdge(e));
    if (it != memory_.end()) {
      it->second.t_last = std::max(it->second.t_last, t_after_wait);
    }
  }
}

void SparrowStrategy::clearMemoryForEdge(const EdgeId& edge) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  memory_.erase(toSEdge(edge));
  archived_sightings_.erase(toSEdge(edge));
}

void SparrowStrategy::resetMemory() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  memory_.clear();
  archived_sightings_.clear();
  kg_consumed_.clear();
  macro_stage_ = MacroStage::kNone;
  macro_wait_active_ = false;
  // The contraction is keyed on the believed-blocked set, which memory feeds.
  macro_cache_valid_ = false;
}

void SparrowStrategy::notifyEpisodeStart(int episode_idx) {
  if (episode_idx > 0) episode_idx_ = episode_idx;
  // The teach pass is evidence the graph is drivable: seed the occupancy
  // counters with one passable observation per corridor, so the planner does
  // not start the deployment believing every edge is blocked. Idempotent, so
  // repeating it across episodes is harmless.
  if (config_.sparrow.teach_prior > 0) {
    obstacle_stats_.applyTeachPrior(config_.sparrow.teach_prior);
  }
  CLOG(INFO, "navigation")
      << "HSHMAT SparrowStrategy: notifyEpisodeStart -> episode "
      << episode_idx_ << " (p_block="
      << (config_.sparrow.p_block_live ? obstacle_stats_.pBlockJeffreys()
                                       : obstacle_stats_.p_block())
      << ", edges_traversed=" << obstacle_stats_.totalEdgesTraversed() << ")";
}

}  // namespace navigation
}  // namespace vtr
