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
  
  // Get travel times
  double A_goal = get_travel_time_(false);   // Time if obstacle clears (no ban)
  double A_avoid = get_travel_time_(true);   // Time if we reroute (ban blocked edge)
  
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: A_goal=" << A_goal
                            << ", A_avoid=" << A_avoid << ", W_max=" << W_max;
  
  // If no alternative route, must wait
  if (A_avoid >= 1e9 || std::isinf(A_avoid)) {
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: No alternative route, must wait";
    return WaitDecision::waitForever("Obstacle " + obs_type + ". No alternate route. Waiting.");
  }
  
  // If detour is actually faster, always detour
  if (A_avoid <= A_goal) {
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Detour is faster, rerouting immediately";
    return WaitDecision::detour("Obstacle " + obs_type + ". Rerouting.");
  }
  
  // Grid search for optimal W*
  double best_W = 0.0;
  double best_J = std::numeric_limits<double>::infinity();
  
  for (int i = 0; i <= config_.W_grid_points; ++i) {
    double W = (static_cast<double>(i) / config_.W_grid_points) * W_max;
    double J = computeJ(obs_type, W, elapsed, A_goal, A_avoid);
    
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
  // J(W) = clear_term + avoid_term
  // 
  // clear_term = integral from 0 to W of: p(t) * (t + A_goal) dt
  //            = sum over t_i of: (S(t_i-1) - S(t_i)) * (t_i + A_goal)
  //
  // avoid_term = S(W) * (W + A_avoid)
  //
  // For conditional (elapsed > 0), use conditional survival S(t | T > elapsed)
  
  if (W <= 0.0) {
    // Immediate detour: cost = A_avoid
    return A_avoid;
  }
  
  double clear_term = 0.0;
  double dt = W / config_.T_grid_points;
  
  for (int i = 0; i < config_.T_grid_points; ++i) {
    double t_left = i * dt;
    double t_right = (i + 1) * dt;
    double t_mid = (t_left + t_right) / 2.0;
    
    // Probability mass in this interval
    double S_left, S_right;
    if (elapsed > 0.0) {
      S_left = survival_model_.conditionalSurvival(obs_type, t_left, elapsed);
      S_right = survival_model_.conditionalSurvival(obs_type, t_right, elapsed);
    } else {
      S_left = survival_model_.survival(obs_type, t_left);
      S_right = survival_model_.survival(obs_type, t_right);
    }
    
    double prob_mass = S_left - S_right;
    if (prob_mass > 0.0) {
      // If clears at t_mid, total time = t_mid (wait) + A_goal (travel)
      clear_term += prob_mass * (t_mid + A_goal);
    }
  }
  
  // Avoid term: probability still blocked at W, then reroute
  double S_W;
  if (elapsed > 0.0) {
    S_W = survival_model_.conditionalSurvival(obs_type, W, elapsed);
  } else {
    S_W = survival_model_.survival(obs_type, W);
  }
  
  double avoid_term = S_W * (W + A_avoid);
  
  return clear_term + avoid_term;
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
