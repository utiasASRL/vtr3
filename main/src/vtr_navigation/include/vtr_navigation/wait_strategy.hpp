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
 * \file wait_strategy.hpp
 * \brief Strategy pattern for obstacle wait time decisions.
 * 
 * HSHMAT: Implements different strategies for deciding how long to wait
 * when an obstacle is encountered. The Learned strategy uses time-dependent
 * TDSP exactly matching the Python simulation.
 */
#pragma once

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <functional>

#include "vtr_navigation/survival_model.hpp"
#include "vtr_navigation/obstacle_memory.hpp"
#include "vtr_navigation/obstacle_stats.hpp"
#include "vtr_route_planning/ew_tdsp_planner.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace navigation {

// Forward declarations
class Navigator;

/**
 * \brief Result of computing optimal wait time.
 */
struct WaitDecision {
  double W_star;           // Optimal wait time in seconds (INFINITY = wait forever, 0 = detour now)
  bool should_wait;        // true if W_star > 0 (wait), false if should detour immediately
  std::string speech;      // What to announce
  
  static WaitDecision wait(double duration, const std::string& msg) {
    return {duration, true, msg};
  }
  
  static WaitDecision detour(const std::string& msg) {
    return {0.0, false, msg};
  }
  
  static WaitDecision waitForever(const std::string& msg) {
    return {std::numeric_limits<double>::infinity(), true, msg};
  }
};

/**
 * \brief Configuration for wait strategies.
 */
struct WaitStrategyConfig {
  // General
  double default_W_max = 120.0;  // Default max wait time (seconds)
  std::map<std::string, double> W_max_per_type;  // Per-type max wait
  
  // Rule-based
  std::set<std::string> wait_types = {"person"};  // Types to wait for
  
  // Learned
  int W_grid_points = 100;   // Number of W candidates for grid search
  int T_grid_points = 50;    // Number of time points for integration
  std::string survival_data_file;  // Path to survival data
  std::string obstacle_stats_dir;  // Directory for persistent obstacle statistics
  std::map<std::string, std::vector<double>> seed_samples;  // Initial samples per type
  
  // Obstacle parameters (defaults, overridden by learned stats when available)
  double p_block = 0.05;     // Probability of obstacle on any edge
  std::map<std::string, double> type_weights;  // Probability distribution over types
  
  // Robot speed (for computing edge travel times)
  double robot_speed_mps = 1.0;
  
  double getWMax(const std::string& obs_type) const {
    auto it = W_max_per_type.find(obs_type);
    return (it != W_max_per_type.end()) ? it->second : default_W_max;
  }
  
  double getTypeWeight(const std::string& obs_type) const {
    auto it = type_weights.find(obs_type);
    return (it != type_weights.end()) ? it->second : 1.0 / std::max(1.0, static_cast<double>(type_weights.size()));
  }
};

/**
 * \brief Strategy type enumeration.
 */
enum class StrategyType {
  ALWAYS_WAIT,
  ALWAYS_DETOUR,
  RULE_BASED,
  GREEDY_CTP,
  LEARNED
};

/**
 * \brief Convert string to StrategyType.
 */
inline StrategyType parseStrategyType(const std::string& s) {
  if (s == "always_wait") return StrategyType::ALWAYS_WAIT;
  if (s == "always_detour") return StrategyType::ALWAYS_DETOUR;
  if (s == "rule_based") return StrategyType::RULE_BASED;
  if (s == "greedy_ctp") return StrategyType::GREEDY_CTP;
  if (s == "learned") return StrategyType::LEARNED;
  throw std::invalid_argument("Unknown strategy type: " + s);
}

/**
 * \brief Convert StrategyType to string.
 */
inline std::string strategyTypeToString(StrategyType t) {
  switch (t) {
    case StrategyType::ALWAYS_WAIT: return "always_wait";
    case StrategyType::ALWAYS_DETOUR: return "always_detour";
    case StrategyType::RULE_BASED: return "rule_based";
    case StrategyType::GREEDY_CTP: return "greedy_ctp";
    case StrategyType::LEARNED: return "learned";
  }
  return "unknown";
}

/**
 * \brief Base class for wait time strategies.
 */
class WaitStrategy {
 public:
  virtual ~WaitStrategy() = default;
  
  /**
   * \brief Compute optimal wait time for an obstacle.
   * 
   * For Learned strategy, this needs context about the graph and current position.
   * Simple strategies (always_wait, etc.) ignore these parameters.
   * 
   * \param obs_type Obstacle type (e.g., "person", "chair")
   * \param blocked_edges Set of edges blocked by current obstacle
   * \param current_vertex Current robot position
   * \param goal_vertex Goal position
   * \param t_now Current global time
   * \param obstacle_t_first When obstacle was first observed (for conditional survival)
   * \return WaitDecision with W* and speech
   */
  virtual WaitDecision computeWaitTime(
      const std::string& obs_type,
      const EdgeIdSet& blocked_edges,
      const tactic::VertexId& current_vertex,
      const tactic::VertexId& goal_vertex,
      double t_now,
      double obstacle_t_first = 0.0) = 0;
  
  /**
   * \brief Called when obstacle clears (for updating models).
   * \param obs_type Obstacle type
   * \param wait_duration How long we actually waited
   */
  virtual void onObstacleCleared(const std::string& obs_type, double wait_duration) {}
  
  /**
   * \brief Called when we timeout and reroute (for updating models).
   * \param obs_type Obstacle type
   * \param wait_duration How long we waited before rerouting (censored sample)
   */
  virtual void onRerouteTimeout(const std::string& obs_type, double wait_duration) {}
  
  /**
   * \brief Update memory after a censored wait (for Learned strategy).
   */
  virtual void updateMemoryAfterCensoredWait(const EdgeIdSet& blocked_edges, double t_after_wait) {}
  
  /**
   * \brief Clear memory for an edge (obstacle actually cleared).
   */
  virtual void clearMemoryForEdge(const EdgeId& edge) {}
  
  /**
   * \brief Reset all memory (called at start of new Repeat).
   */
  virtual void resetMemory() {}
  
  /**
   * \brief For greedy CTP: check if an edge is permanently banned.
   */
  virtual bool isEdgePermanentlyBanned(const EdgeId& edge) const { return false; }
  
  /**
   * \brief For greedy CTP: mark edge as permanently banned.
   */
  virtual void banEdgePermanently(const EdgeId& edge) {}
  
  /**
   * \brief Clear all permanently banned edges (for new run).
   */
  virtual void clearPermanentBans() {}
  
  /**
   * \brief Get strategy type.
   */
  virtual StrategyType type() const = 0;
  
  /**
   * \brief Get the survival model (if any).
   */
  virtual SurvivalModel* survivalModel() { return nullptr; }
  
  /**
   * \brief Set graph access functions (for Learned strategy TDSP).
   */
  virtual void setGraphAccess(
      route_planning::NeighborsFn get_neighbors,
      route_planning::TravelTimeFn get_travel_time) {}
};

/**
 * \brief Always wait until obstacle clears.
 */
class AlwaysWaitStrategy : public WaitStrategy {
 public:
  WaitDecision computeWaitTime(
      const std::string& obs_type,
      const EdgeIdSet& blocked_edges,
      const tactic::VertexId& current_vertex,
      const tactic::VertexId& goal_vertex,
      double t_now,
      double obstacle_t_first) override {
    return WaitDecision::waitForever("Obstacle detected. Waiting.");
  }
  
  StrategyType type() const override { return StrategyType::ALWAYS_WAIT; }
};

/**
 * \brief Always reroute immediately.
 */
class AlwaysDetourStrategy : public WaitStrategy {
 public:
  WaitDecision computeWaitTime(
      const std::string& obs_type,
      const EdgeIdSet& blocked_edges,
      const tactic::VertexId& current_vertex,
      const tactic::VertexId& goal_vertex,
      double t_now,
      double obstacle_t_first) override {
    return WaitDecision::detour("Obstacle detected. Rerouting.");
  }
  
  StrategyType type() const override { return StrategyType::ALWAYS_DETOUR; }
};

/**
 * \brief Wait for certain obstacle types, reroute for others.
 */
class RuleBasedStrategy : public WaitStrategy {
 public:
  explicit RuleBasedStrategy(const std::set<std::string>& wait_types)
      : wait_types_(wait_types) {}
  
  WaitDecision computeWaitTime(
      const std::string& obs_type,
      const EdgeIdSet& blocked_edges,
      const tactic::VertexId& current_vertex,
      const tactic::VertexId& goal_vertex,
      double t_now,
      double obstacle_t_first) override {
    // Speech format: "[type]. Waiting/Rerouting." (Navigator already said "Obstacle detected.")
    if (wait_types_.count(obs_type) > 0) {
      return WaitDecision::waitForever(obs_type + ". Waiting.");
    } else {
      return WaitDecision::detour(obs_type + ". Rerouting.");
    }
  }
  
  StrategyType type() const override { return StrategyType::RULE_BASED; }
  
 private:
  std::set<std::string> wait_types_;
};

/**
 * \brief Greedy CTP: reroute immediately, permanently ban blocked edges.
 * 
 * When all alternate paths are exhausted (no valid path to goal), falls back
 * to waiting mode. Tracks this state so subsequent detections on the last
 * valid path just wait silently.
 */
class GreedyCTPStrategy : public WaitStrategy {
 public:
  WaitDecision computeWaitTime(
      const std::string& obs_type,
      const EdgeIdSet& blocked_edges,
      const tactic::VertexId& current_vertex,
      const tactic::VertexId& goal_vertex,
      double t_now,
      double obstacle_t_first) override {
    // NOTE: Don't ban edges here! Edges are banned in triggerReroute() after
    // we recompute with accurate localization. Banning here with potentially
    // stale localization causes extra edges to be permanently banned.
    
    // If we've already exhausted all paths, just wait
    if (no_valid_path_) {
      return WaitDecision::waitForever("Obstacle detected. Waiting.");
    }
    
    return WaitDecision::detour("Obstacle detected. Rerouting.");
  }
  
  bool isEdgePermanentlyBanned(const EdgeId& edge) const override {
    return banned_edges_.count(edge) > 0;
  }
  
  void banEdgePermanently(const EdgeId& edge) override {
    banned_edges_.insert(edge);
  }
  
  void clearPermanentBans() override { 
    banned_edges_.clear(); 
    no_valid_path_ = false;
  }
  
  // Called by Navigator when reroute fails (no alternate path found)
  void onNoValidPath() {
    no_valid_path_ = true;
  }
  
  bool hasNoValidPath() const { return no_valid_path_; }
  
  // Get all permanently banned edges (for applying to route planner)
  const EdgeIdSet& getBannedEdges() const { return banned_edges_; }
  
  StrategyType type() const override { return StrategyType::GREEDY_CTP; }
  
 private:
  EdgeIdSet banned_edges_;
  bool no_valid_path_ = false;  // True when all paths exhausted
};

/**
 * \brief Learned strategy: compute optimal W* using survival model and time-dependent TDSP.
 * 
 * HSHMAT: This exactly matches the Python simulation algorithm:
 * 
 * J(W) = clear_term + avoid_term
 * 
 * clear_term = sum_{t < W} p(t) * A_goal(t0 + t)
 *   - p(t) = probability obstacle clears at time t (from survival model)
 *   - A_goal(t0 + t) = earliest arrival at goal if obstacle clears at t0+t
 *   - Computed via TDSP with override_availability = t0 + t for the blocked edge
 * 
 * avoid_term = S(W) * A_avoid(t0 + W)
 *   - S(W) = probability obstacle still blocked at time W
 *   - A_avoid(t0 + W) = earliest arrival at goal if we reroute at time t0+W
 *   - Computed via TDSP with blocked edge forbidden, start_time = t0 + W
 * 
 * Expected wait on edges WITHOUT memory (potential new obstacles):
 *   E[wait] = p_block * sum_types(prob_type * mean_duration_type)
 * 
 * Expected wait on edges WITH memory (we saw it, gave up):
 *   E[wait] = P(still blocked) * E[remaining | blocked] + P(cleared) * E[wait | new]
 */
class LearnedStrategy : public WaitStrategy {
 public:
  explicit LearnedStrategy(const WaitStrategyConfig& config);
  
  WaitDecision computeWaitTime(
      const std::string& obs_type,
      const EdgeIdSet& blocked_edges,
      const tactic::VertexId& current_vertex,
      const tactic::VertexId& goal_vertex,
      double t_now,
      double obstacle_t_first) override;
  
  void onObstacleCleared(const std::string& obs_type, double wait_duration) override;
  void onRerouteTimeout(const std::string& obs_type, double wait_duration) override;
  
  void updateMemoryAfterCensoredWait(const EdgeIdSet& blocked_edges, double t_after_wait) override;
  void clearMemoryForEdge(const EdgeId& edge) override;
  void resetMemory() override;
  
  StrategyType type() const override { return StrategyType::LEARNED; }
  SurvivalModel* survivalModel() override { return &survival_model_; }
  
  /**
   * \brief Get global obstacle statistics (for Navigator to record edge traversals).
   */
  GlobalObstacleStats* obstacleStats() { return &obstacle_stats_; }
  
  /**
   * \brief Save all persistent data (survival model + obstacle stats).
   */
  void saveData();
  
  void setGraphAccess(
      route_planning::NeighborsFn get_neighbors,
      route_planning::TravelTimeFn get_travel_time) override {
    get_neighbors_ = get_neighbors;
    get_travel_time_ = get_travel_time;
  }
  
 private:
  /**
   * \brief Compute expected wait time for a fresh edge (no memory).
   * Uses learned p_block and p_obs_type from obstacle_stats_.
   */
  double computeExpectedWaitForNewObstacle() const;
  
  /**
   * \brief Compute expected wait time for an edge.
   * Accounts for edges with memory (saw obstacle, gave up) vs fresh edges.
   */
  double computeExpectedWaitForEdge(const EdgeId& edge, double planned_arrival_time) const;
  
  /**
   * \brief Create expected wait function for TDSP.
   */
  route_planning::ExpectedWaitFn createExpectedWaitFn(
      double t0,
      const EdgeId& exclude_edge = EdgeId::Invalid()) const;
  
  /**
   * \brief Build speech for waiting.
   */
  std::string buildWaitSpeech(const std::string& obs_type, double W_star) const;
  
  WaitStrategyConfig config_;
  SurvivalModel survival_model_;
  ObstacleMemoryManager memory_;
  GlobalObstacleStats obstacle_stats_;
  route_planning::EWTDSPPlanner tdsp_planner_;
  
  // Graph access functions
  route_planning::NeighborsFn get_neighbors_;
  route_planning::TravelTimeFn get_travel_time_;
};

/**
 * \brief Factory function to create strategy from config.
 */
std::unique_ptr<WaitStrategy> createWaitStrategy(
    StrategyType type,
    const WaitStrategyConfig& config);

}  // namespace navigation
}  // namespace vtr
