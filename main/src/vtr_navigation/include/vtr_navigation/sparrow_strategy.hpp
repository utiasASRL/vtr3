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
 * ("POMCP" strategy in the sim, SPARROW in the paper). At every obstacle
 * encounter (and, unlike other strategies, again at every wait timeout) it:
 *
 *  1. Builds a graph context from the privileged (teach) graph via the same
 *     graph-access lambdas the Learned strategy uses.
 *  2. Assembles the robot's local observation: which adjacent edges are
 *     blocked / free / unknown (from the obstacle detector + costmap via the
 *     Navigator's adjacent-edge-status hook), the observed age of the current
 *     encounter, its VLM class label, and remembered past sightings.
 *  3. Draws N fresh particles conditioned on that observation from a frozen
 *     snapshot of the learned model (KM survival fits + obstacle stats).
 *  4. Runs cost-minimizing POMCP and maps the root action to a WaitDecision:
 *     MaxWait(W) -> wait(W), Traverse(detour) -> detour.
 *
 * Differences from the simulation, by design of the Navigator FSM:
 *  - Observe is never offered at the root: the VLM classification already
 *    happened before computeWaitTime is called (Observe still exists inside
 *    the search for hypothetical future encounters).
 *  - A root Traverse decision hands route choice to the Navigator's reroute
 *    (TDSP with banned blocked edges + uniform expected-wait delay), same as
 *    the Learned strategy.
 *  - Adjacent edges out of sensor range are "unknown": the belief samples
 *    them from the steady-state prior and a root Traverse onto them is
 *    allowed (the detector will re-trigger if they turn out blocked).
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

  /** \brief Persist survival model + obstacle stats. */
  void saveData();

 private:
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
    double t_first = 0.0;          // first sighting of this encounter
    double t_last = 0.0;           // last time confirmed still blocked
    std::string label;             // VLM label if known
  };
  std::map<sparrow::SEdge, MemEntry> memory_;

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
  // HSHMAT: one on-demand VLM call per encounter. Reset when a fresh
  // encounter starts (obstacle_t_first == 0), so a re-plan after Observe (or
  // after a MaxWait expires) cannot loop the VLM if it keeps saying unknown.
  bool observe_used_ = false;
};

}  // namespace navigation
}  // namespace vtr
