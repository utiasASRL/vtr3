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
 * when an obstacle is encountered:
 * - AlwaysWait: Wait indefinitely until obstacle clears
 * - AlwaysDetour: Immediately reroute (W* = 0)
 * - RuleBased: Wait for certain obstacle types, detour for others
 * - GreedyCTP: Immediately reroute, permanently ban blocked edges
 * - Learned: Compute optimal W* using survival model and TDSP
 */
#pragma once

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <functional>

#include "vtr_navigation/survival_model.hpp"

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
  std::map<std::string, std::vector<double>> seed_samples;  // Initial samples per type
  
  // Travel time function (provided by Navigator)
  // Returns time to reach goal from current position, optionally with edge banned
  std::function<double(bool ban_blocked_edge)> get_travel_time;
  
  double getWMax(const std::string& obs_type) const {
    auto it = W_max_per_type.find(obs_type);
    return (it != W_max_per_type.end()) ? it->second : default_W_max;
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
   * \param obs_type Obstacle type (e.g., "person", "chair")
   * \param elapsed Time already waited (for re-evaluation)
   * \return WaitDecision with W* and speech
   */
  virtual WaitDecision computeWaitTime(
      const std::string& obs_type,
      double elapsed = 0.0) = 0;
  
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
   * \brief For greedy CTP: check if an edge is permanently banned.
   */
  virtual bool isEdgePermanentlyBanned(uint64_t edge_id) const { return false; }
  
  /**
   * \brief For greedy CTP: mark edge as permanently banned.
   */
  virtual void banEdgePermanently(uint64_t edge_id) {}
  
  /**
   * \brief Get strategy type.
   */
  virtual StrategyType type() const = 0;
  
  /**
   * \brief Get the survival model (if any).
   */
  virtual SurvivalModel* survivalModel() { return nullptr; }
};

/**
 * \brief Always wait until obstacle clears.
 */
class AlwaysWaitStrategy : public WaitStrategy {
 public:
  WaitDecision computeWaitTime(const std::string& obs_type, double elapsed) override {
    return WaitDecision::waitForever("Obstacle detected. Waiting.");
  }
  
  StrategyType type() const override { return StrategyType::ALWAYS_WAIT; }
};

/**
 * \brief Always reroute immediately.
 */
class AlwaysDetourStrategy : public WaitStrategy {
 public:
  WaitDecision computeWaitTime(const std::string& obs_type, double elapsed) override {
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
  
  WaitDecision computeWaitTime(const std::string& obs_type, double elapsed) override {
    if (wait_types_.count(obs_type) > 0) {
      return WaitDecision::waitForever("Obstacle " + obs_type + ". Waiting.");
    } else {
      return WaitDecision::detour("Obstacle " + obs_type + ". Rerouting.");
    }
  }
  
  StrategyType type() const override { return StrategyType::RULE_BASED; }
  
 private:
  std::set<std::string> wait_types_;
};

/**
 * \brief Greedy CTP: reroute immediately, permanently ban blocked edges.
 */
class GreedyCTPStrategy : public WaitStrategy {
 public:
  WaitDecision computeWaitTime(const std::string& obs_type, double elapsed) override {
    return WaitDecision::detour("Obstacle detected. Rerouting.");
  }
  
  bool isEdgePermanentlyBanned(uint64_t edge_id) const override {
    return banned_edges_.count(edge_id) > 0;
  }
  
  void banEdgePermanently(uint64_t edge_id) override {
    banned_edges_.insert(edge_id);
  }
  
  StrategyType type() const override { return StrategyType::GREEDY_CTP; }
  
  void clearBannedEdges() { banned_edges_.clear(); }
  
 private:
  std::set<uint64_t> banned_edges_;
};

/**
 * \brief Learned strategy: compute optimal W* using survival model.
 * 
 * Minimizes expected time to goal:
 *   J(W) = clear_term + avoid_term
 * where:
 *   clear_term = sum over t < W of: p(clear at t) * A_goal(t)
 *   avoid_term = S(W) * A_avoid(W)
 */
class LearnedStrategy : public WaitStrategy {
 public:
  explicit LearnedStrategy(const WaitStrategyConfig& config);
  
  WaitDecision computeWaitTime(const std::string& obs_type, double elapsed) override;
  
  void onObstacleCleared(const std::string& obs_type, double wait_duration) override;
  void onRerouteTimeout(const std::string& obs_type, double wait_duration) override;
  
  StrategyType type() const override { return StrategyType::LEARNED; }
  SurvivalModel* survivalModel() override { return &survival_model_; }
  
  /**
   * \brief Set the travel time function (called by Navigator).
   */
  void setTravelTimeFunction(std::function<double(bool)> func) {
    get_travel_time_ = func;
  }
  
 private:
  /**
   * \brief Compute J(W) for a given wait time.
   * \param obs_type Obstacle type
   * \param W Wait time candidate
   * \param elapsed Time already waited
   * \param A_goal Travel time if obstacle clears
   * \param A_avoid Travel time if we reroute
   * \return Expected total time
   */
  double computeJ(const std::string& obs_type, double W, double elapsed,
                  double A_goal, double A_avoid) const;
  
  /**
   * \brief Build speech for waiting.
   */
  std::string buildWaitSpeech(const std::string& obs_type, double W_star) const;
  
  WaitStrategyConfig config_;
  SurvivalModel survival_model_;
  std::function<double(bool)> get_travel_time_;
};

/**
 * \brief Factory function to create strategy from config.
 */
std::unique_ptr<WaitStrategy> createWaitStrategy(
    StrategyType type,
    const WaitStrategyConfig& config);

}  // namespace navigation
}  // namespace vtr
