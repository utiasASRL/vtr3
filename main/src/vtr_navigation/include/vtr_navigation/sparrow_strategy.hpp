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
 * \file sparrow_strategy.hpp
 * \brief SPARROW (POMCP) wait strategy: adapter between the Navigator's
 *        WaitStrategy interface and the sparrow_planner POMCP core.
 *
 * HSHMAT: SPARROW is the POMCP planner from vtr_obstacle_simulation
 * ("POMCP" strategy in the sim, SPARROW in the paper). At every decision
 * epoch - obstacle encounter, wait timeout, Observe completion, and junction
 * approach with a relevant belief (every-decision replanning, paper
 * Table VII) - it:
 *
 *  1. Builds a graph context from the privileged (teach) graph via the same
 *     graph-access lambdas the Learned strategy uses.
 *  2. Assembles the robot's local observation: which adjacent edges are
 *     blocked / free / unknown (from the obstacle detector + costmap via the
 *     Navigator's adjacent-edge-status hook), the observed ages / labels of
 *     blocked edges, and remembered / archived past sightings.
 *  3. Draws N fresh particles conditioned on that observation from a frozen
 *     snapshot of the learned model (KM survival fits + obstacle stats).
 *  4. Runs cost-minimizing POMCP and maps the root action to a WaitDecision:
 *     MaxWait(W) -> wait(W), Observe(e) -> request one VLM classification of
 *     edge e (this is the ONLY path that calls the VLM), Traverse -> detour
 *     committed to the chosen corridor (the Navigator bans the alternative
 *     corridor entrances so the reroute executes exactly this action).
 *
 * Continuous monitoring: the Navigator calls updateEdgeMonitoring at ~1 Hz
 * with the costmap-derived statuses of the edges around the robot. This keeps
 * per-edge sighting streaks honest: a gap longer than monitor_gap_s means the
 * edge genuinely left view, so the old record is archived and the belief's
 * same-obstacle-vs-new-obstacle mixture applies on the next sighting.
 */
#pragma once

#include "vtr_navigation/sparrow_planner.hpp"
#include "vtr_navigation/wait_strategy.hpp"

namespace vtr {
namespace navigation {

/**
 * \brief Function the Navigator provides to report the observed status of
 *        edges incident to a vertex: 1 = blocked, 0 = free, -1 = unknown.
 */
using AdjacentEdgeStatusFn =
    std::function<std::map<tactic::EdgeId, int>(const tactic::VertexId&)>;

class SparrowStrategy : public WaitStrategy {
 public:
  explicit SparrowStrategy(const WaitStrategyConfig& config);

  WaitDecision computeWaitTime(const std::string& obs_type,
                               const EdgeIdSet& blocked_edges,
                               const tactic::VertexId& current_vertex,
                               const tactic::VertexId& goal_vertex,
                               double t_now,
                               double obstacle_t_first) override;

  void onObstacleCleared(const std::string& obs_type, double wait_duration,
                         int episode) override;
  void onRerouteTimeout(const std::string& obs_type, double wait_duration,
                        int episode) override;
  void updateMemoryAfterCensoredWait(const EdgeIdSet& blocked_edges,
                                     double t_after_wait) override;
  void clearMemoryForEdge(const EdgeId& edge) override;
  void resetMemory() override;
  void notifyEpisodeStart(int episode_idx) override;
  void flushPendingSamplesToKM() override;

  StrategyType type() const override { return StrategyType::SPARROW; }
  SurvivalModel* survivalModel() override { return &survival_model_; }
  GlobalObstacleStats* obstacleStats() { return &obstacle_stats_; }

  void setGraphAccess(route_planning::NeighborsFn get_neighbors,
                      route_planning::TravelTimeFn get_travel_time) override {
    get_neighbors_ = get_neighbors;
    get_travel_time_ = get_travel_time;
  }

  /** \brief Navigator hook: observed status of edges incident to a vertex. */
  void setAdjacentEdgeStatusFn(AdjacentEdgeStatusFn fn) {
    adjacent_status_fn_ = fn;
  }

  /**
   * \brief Uniform per-edge expected wait for the reroute TDSP
   *        (= p_block * E[residual], mirrors LearnedStrategy).
   */
  double freshEdgeExpectedWait() const;

  /**
   * \brief Every-decision replanning entry point: plan at an upcoming
   *        decision vertex (junction) while the robot is still driving
   *        toward it. No front blockage is required; blocked information
   *        comes from the adjacent-status hook and remembered sightings.
   */
  WaitDecision planEnRoute(const tactic::VertexId& robot_vertex,
                           const tactic::VertexId& junction_vertex,
                           const tactic::VertexId& goal_vertex, double t_now);

  /**
   * \brief Continuous monitoring update (~1 Hz from the Navigator): statuses
   *        of edges currently in costmap view (1 blocked / 0 free / -1
   *        unknown). Maintains per-edge sighting streaks so "re-sighting"
   *        means the edge genuinely left view, not "a wait cycle passed".
   */
  void updateEdgeMonitoring(const std::map<tactic::EdgeId, int>& statuses,
                            double t_now);

  /** \brief Edges currently remembered as blocked (junction-replan trigger). */
  std::vector<std::pair<uint64_t, uint64_t>> rememberedBlockedEdges() const;

  /** \brief Persist survival model + obstacle stats. */
  void saveData();

 private:
  /// Shared core of computeWaitTime / planEnRoute. If plan_vertex_override is
  /// nonzero, planning happens at that vertex (junction replanning);
  /// otherwise at the nearest endpoint of the nearest blocked edge.
  WaitDecision planInternal(const std::string& obs_type,
                            const EdgeIdSet& blocked_edges,
                            const tactic::VertexId& current_vertex,
                            const tactic::VertexId& goal_vertex, double t_now,
                            double obstacle_t_first,
                            const tactic::VertexId& plan_vertex_override);

  /// Record a blocked sighting of `e` whose current streak started at
  /// `streak_start`. Detects re-sightings (gap > monitor_gap_s since t_last):
  /// archives the old record for the belief mixture and restarts the streak.
  void noteBlockedSighting(const sparrow::SEdge& e, double streak_start,
                           double t_now);
  /// Snapshot the learned model (KM + stats) into a frozen SparrowModel.
  sparrow::SparrowModel snapshotModel(int num_edges) const;
  /// Enumerate the privileged graph reachable from the given seeds.
  sparrow::GraphContext buildContext(const tactic::VertexId& current_vertex,
                                     const tactic::VertexId& goal_vertex) const;

  static sparrow::SEdge toSEdge(const tactic::EdgeId& e) {
    return sparrow::canonical_edge(
        static_cast<uint64_t>(e.id1()), static_cast<uint64_t>(e.id2()));
  }

  WaitStrategyConfig config_;
  SurvivalModel survival_model_;
  GlobalObstacleStats obstacle_stats_;

  std::string survival_stats_file_;
  std::string obstacle_stats_file_;

  route_planning::NeighborsFn get_neighbors_;
  route_planning::TravelTimeFn get_travel_time_;
  AdjacentEdgeStatusFn adjacent_status_fn_;

  // Memory of past sightings (ports belief.EdgeMemory bookkeeping).
  struct MemEntry {
    double t_first = 0.0;          // start of the current sighting streak
    double t_last = 0.0;           // last time confirmed still blocked
    std::string label;             // VLM label if known (current streak)
  };
  std::map<sparrow::SEdge, MemEntry> memory_;
  // Archived pre-gap sightings: when an edge is re-sighted blocked after
  // leaving view, the old streak's record lands here so every belief
  // (re)initialization during the new streak runs the same-obstacle-vs-new-
  // obstacle mixture. Erased when the edge is confirmed free; replaced on the
  // next gap.
  std::map<sparrow::SEdge, sparrow::EdgeMemoryRec> archived_sightings_;

  // Pending KM samples, flushed at episode end (matches sim per-episode mode).
  struct PendingSample {
    std::string obs_type;
    double duration;
    bool censored;
    int episode;
  };
  std::vector<PendingSample> pending_samples_;

  int episode_idx_ = 0;
  uint64_t plan_counter_ = 0;  // varies the search seed across encounters
  // HSHMAT: on-demand VLM bookkeeping. Any adjacent blocked edge may be
  // observed, but each edge gets at most ONE VLM call per encounter (so a VLM
  // that answers "unknown" cannot be looped on). pending_observe_edge_ is the
  // edge the last Observe decision asked about: the label in the next
  // computeWaitTime call is routed to it. Both reset when a fresh encounter
  // starts (obstacle_t_first == 0).
  std::set<sparrow::SEdge> observed_edges_;
  std::optional<sparrow::SEdge> pending_observe_edge_;
};

}  // namespace navigation
}  // namespace vtr
