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
 * \file wait_strategy.cpp
 * \brief Wait strategy implementations.
 */
#include "vtr_navigation/wait_strategy.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace navigation {

// ============================================================================
// LearnedStrategy
// ============================================================================

LearnedStrategy::LearnedStrategy(const WaitStrategyConfig& config)
    : config_(config) {
  // Load existing survival data
  if (!config_.survival_data_file.empty()) {
    survival_model_.loadFromFile(config_.survival_data_file);
  }
  
  // Add seed samples
  for (const auto& kv : config_.seed_samples) {
    // Only add seeds if we don't have enough data yet
    if (survival_model_.sampleCount(kv.first) < kv.second.size()) {
      survival_model_.addSeedSamples(kv.first, kv.second);
    }
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Initialized with "
                           << config_.W_grid_points << " W grid points, "
                           << config_.T_grid_points << " T grid points";
}

WaitDecision LearnedStrategy::computeWaitTime(const std::string& obs_type, double elapsed) {
  double W_max = config_.getWMax(obs_type);
  
  // If no travel time function, fall back to always wait
  if (!get_travel_time_) {
    CLOG(WARNING, "navigation") << "HSHMAT LearnedStrategy: No travel time function, defaulting to wait";
    return WaitDecision::waitForever("Obstacle " + obs_type + ". Waiting.");
  }
  
  // Get travel times (simplified: assume constant A_goal and A_avoid)
  // In simulation we compute A_goal(t) for each t, but on real robot that's expensive
  // A_goal = travel time if obstacle clears now (we proceed on original path)
  // A_avoid = travel time if we reroute now (banned edge)
  double A_goal = get_travel_time_(false);
  double A_avoid = get_travel_time_(true);
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: A_goal=" << A_goal
                            << ", A_avoid=" << A_avoid << ", W_max=" << W_max
                            << ", elapsed=" << elapsed;
  
  // If no alternative route, must wait
  if (A_avoid >= 1e9 || std::isinf(A_avoid)) {
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: No alternative route, must wait";
    return WaitDecision::waitForever("Obstacle " + obs_type + ". No alternate route. Waiting.");
  }
  
  // If detour is faster than going straight (even without obstacle), always detour
  if (A_avoid <= A_goal) {
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Detour is faster, rerouting immediately";
    return WaitDecision::detour("Obstacle " + obs_type + ". Rerouting.");
  }
  
  // ========================================================================
  // HSHMAT: J(W) computation matching Python simulation algorithm
  //
  // The algorithm:
  // 1. Create T_grid of clearance times: t_1, t_2, ..., t_n over [0, W_max]
  // 2. For each t_i, compute probability mass p(t_i) = S(t_{i-1}) - S(t_i)
  // 3. Compute contribution: contrib[i] = p(t_i) * (t_i + A_goal)
  // 4. Compute prefix sum: cum[i] = sum_{j<=i} contrib[j]
  // 5. For each W in W_grid:
  //    - clear_term = cum[k] where k = largest index with t_k <= W
  //    - avoid_term = S(W) * (W + A_avoid)
  //    - J(W) = clear_term + avoid_term
  // 6. W* = argmin J(W)
  //
  // Note: On real robot, A_goal is constant (no time-dependent TDSP).
  // This is equivalent to A_goal(t) = A_goal for all t.
  // ========================================================================
  
  const int n_T = std::max(16, config_.T_grid_points);
  const int n_W = std::max(16, config_.W_grid_points);
  
  // Step 1: Create T_grid of clearance times
  std::vector<double> t_clear(n_T);
  double dt = W_max / n_T;
  for (int i = 0; i < n_T; ++i) {
    t_clear[i] = (i + 1) * dt;  // Right endpoints: dt, 2*dt, ..., W_max
  }
  
  // Step 2-3: Compute probability masses and contributions
  std::vector<double> contrib(n_T);
  for (int i = 0; i < n_T; ++i) {
    double t_left = i * dt;
    double t_right = (i + 1) * dt;
    
    double S_left, S_right;
    if (elapsed > 0.0) {
      S_left = survival_model_.conditionalSurvival(obs_type, t_left, elapsed);
      S_right = survival_model_.conditionalSurvival(obs_type, t_right, elapsed);
    } else {
      S_left = survival_model_.survival(obs_type, t_left);
      S_right = survival_model_.survival(obs_type, t_right);
    }
    
    double prob_mass = std::max(0.0, S_left - S_right);
    // If obstacle clears at t_right, total time = t_right (wait) + A_goal (travel)
    contrib[i] = prob_mass * (t_right + A_goal);
  }
  
  // Step 4: Compute prefix sums
  std::vector<double> cum(n_T);
  cum[0] = contrib[0];
  for (int i = 1; i < n_T; ++i) {
    cum[i] = cum[i - 1] + contrib[i];
  }
  
  // Step 5-6: Find optimal W*
  double best_W = 0.0;
  double best_J = A_avoid;  // J(0) = A_avoid (immediate reroute)
  
  for (int i = 0; i <= n_W; ++i) {
    double W = (static_cast<double>(i) / n_W) * W_max;
    
    // Find clear_term using prefix sum
    // k = largest index such that t_clear[k] <= W
    double clear_term = 0.0;
    if (W > 0.0) {
      // Binary search for k
      auto it = std::upper_bound(t_clear.begin(), t_clear.end(), W);
      if (it != t_clear.begin()) {
        int k = static_cast<int>(std::distance(t_clear.begin(), it)) - 1;
        clear_term = cum[k];
      }
    }
    
    // Compute S(W) for avoid term
    double S_W;
    if (elapsed > 0.0) {
      S_W = survival_model_.conditionalSurvival(obs_type, W, elapsed);
    } else {
      S_W = survival_model_.survival(obs_type, W);
    }
    
    double avoid_term = S_W * (W + A_avoid);
    double J = clear_term + avoid_term;
    
    if (J < best_J) {
      best_J = J;
      best_W = W;
    }
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Optimal W*=" << best_W
                           << "s with J=" << best_J << " for " << obs_type;
  
  // Check if we've already waited past W*
  double remaining_wait = best_W - elapsed;
  if (remaining_wait <= 0.0) {
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Already waited " << elapsed
                             << "s >= W*=" << best_W << ", rerouting";
    return WaitDecision::detour("Time limit exceeded. Rerouting.");
  }
  
  // Round to nice number for speech
  int wait_seconds = static_cast<int>(std::ceil(remaining_wait));
  
  return WaitDecision::wait(remaining_wait, buildWaitSpeech(obs_type, wait_seconds));
}

double LearnedStrategy::computeJ(const std::string& obs_type, double W, double elapsed,
                                  double A_goal, double A_avoid) const {
  // This function is now unused - J(W) is computed inline in computeWaitTime
  // using prefix sums for efficiency. Kept for interface compatibility.
  (void)obs_type; (void)W; (void)elapsed; (void)A_goal; (void)A_avoid;
  return 0.0;
}

std::string LearnedStrategy::buildWaitSpeech(const std::string& obs_type, double W_star) const {
  std::ostringstream ss;
  ss << "Obstacle " << obs_type << ". Waiting ";
  
  int seconds = static_cast<int>(W_star);
  if (seconds >= 60) {
    int minutes = seconds / 60;
    int secs = seconds % 60;
    if (secs == 0) {
      ss << minutes << " minute" << (minutes > 1 ? "s" : "");
    } else {
      ss << minutes << " minute" << (minutes > 1 ? "s" : "") << " " << secs << " seconds";
    }
  } else {
    ss << seconds << " seconds";
  }
  ss << ".";
  
  return ss.str();
}

void LearnedStrategy::onObstacleCleared(const std::string& obs_type, double wait_duration) {
  // Uncensored sample: we observed the actual clearance time
  survival_model_.addSample(obs_type, wait_duration, false);
  
  // Save to file
  if (!config_.survival_data_file.empty()) {
    survival_model_.saveToFile(config_.survival_data_file);
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Recorded uncensored sample for "
                           << obs_type << ": " << wait_duration << "s";
}

void LearnedStrategy::onRerouteTimeout(const std::string& obs_type, double wait_duration) {
  // Censored sample: we rerouted before obstacle cleared
  survival_model_.addSample(obs_type, wait_duration, true);
  
  // Save to file
  if (!config_.survival_data_file.empty()) {
    survival_model_.saveToFile(config_.survival_data_file);
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Recorded censored sample for "
                           << obs_type << ": " << wait_duration << "s";
}

// ============================================================================
// Factory
// ============================================================================

std::unique_ptr<WaitStrategy> createWaitStrategy(
    StrategyType type,
    const WaitStrategyConfig& config) {
  
  switch (type) {
    case StrategyType::ALWAYS_WAIT:
      return std::make_unique<AlwaysWaitStrategy>();
      
    case StrategyType::ALWAYS_DETOUR:
      return std::make_unique<AlwaysDetourStrategy>();
      
    case StrategyType::RULE_BASED:
      return std::make_unique<RuleBasedStrategy>(config.wait_types);
      
    case StrategyType::GREEDY_CTP:
      return std::make_unique<GreedyCTPStrategy>();
      
    case StrategyType::LEARNED:
      return std::make_unique<LearnedStrategy>(config);
      
    default:
      CLOG(ERROR, "navigation") << "HSHMAT: Unknown strategy type, defaulting to always_wait";
      return std::make_unique<AlwaysWaitStrategy>();
  }
}

}  // namespace navigation
}  // namespace vtr
