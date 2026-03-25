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
 * 
 * HSHMAT: LearnedStrategy uses time-dependent TDSP exactly matching Python simulation.
 */
#include "vtr_navigation/wait_strategy.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace navigation {

// ============================================================================
// LearnedStrategy
// ============================================================================

LearnedStrategy::LearnedStrategy(const WaitStrategyConfig& config)
    : config_(config) {
  // Derive file paths from learned_data_dir
  // Structure: learned_data_dir/survival_stats.yaml, learned_data_dir/obstacle_stats.yaml
  if (!config_.learned_data_dir.empty()) {
    survival_stats_file_ = config_.learned_data_dir + "/survival_stats.yaml";
    obstacle_stats_file_ = config_.learned_data_dir + "/obstacle_stats.yaml";
    
    // Load existing data
    survival_model_.loadFromFile(survival_stats_file_);
    obstacle_stats_.loadFromFile(obstacle_stats_file_);
  }
  
  // Set defaults for obstacle stats (used when not enough data)
  obstacle_stats_.setDefaultPBlock(config_.p_block);
  obstacle_stats_.setDefaultTypeWeights(config_.type_weights);
  
  // Add seed samples
  for (const auto& kv : config_.seed_samples) {
    // Only add seeds if we don't have enough data yet
    if (survival_model_.sampleCount(kv.first) < kv.second.size()) {
      survival_model_.addSeedSamples(kv.first, kv.second);
    }
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Initialized with "
                           << config_.W_grid_points << " W grid points, "
                           << config_.T_grid_points << " T grid points, "
                           << "p_block=" << obstacle_stats_.p_block()
                           << " (default=" << config_.p_block << ")"
                           << ", edges_traversed=" << obstacle_stats_.totalEdgesTraversed()
                           << ", episodes=" << obstacle_stats_.totalObstacleEpisodes()
                           << ", data_dir=" << config_.learned_data_dir;
}

void LearnedStrategy::saveData() {
  if (config_.learned_data_dir.empty()) return;
  
  // Save survival model
  survival_model_.saveToFile(survival_stats_file_);
  
  // Save obstacle statistics
  obstacle_stats_.saveToFile(obstacle_stats_file_);
}

double LearnedStrategy::computeExpectedWaitForNewObstacle() const {
  // E[wait | new obstacle] = p_block * sum_types(p_obs_type * mean_duration_type)
  // Uses learned statistics when available, falls back to config defaults
  double expected_wait = 0.0;
  
  // Get type distribution (learned or default)
  auto type_dist = obstacle_stats_.getTypeDistribution();
  if (type_dist.empty()) {
    // Fall back to config
    type_dist = config_.type_weights;
  }
  
  for (const auto& kv : type_dist) {
    const std::string& obs_type = kv.first;
    double prob_type = kv.second;
    double mean_duration = survival_model_.meanSurvivalTime(obs_type);
    expected_wait += prob_type * mean_duration;
  }
  
  // Use learned p_block (falls back to default if not enough data)
  return obstacle_stats_.p_block() * expected_wait;
}

double LearnedStrategy::computeExpectedWaitForEdge(
    const EdgeId& edge, 
    double planned_arrival_time) const {
  
  // Check if edge has memory
  auto mem_opt = memory_.getMemory(edge);
  if (!mem_opt.has_value()) {
    // Fresh edge - use expected wait for new obstacle
    return computeExpectedWaitForNewObstacle();
  }
  
  // Edge has memory - we saw an obstacle and gave up
  const auto& mem = mem_opt.value();
  const std::string& obs_type = mem.obs_type;
  double t_first = mem.t_first;
  double c = mem.c();  // Confirmed duration
  
  // Time since obstacle first appeared, at planned arrival
  double elapsed_at_arrival = planned_arrival_time - t_first;
  
  // If planned arrival is before last confirmation, obstacle certainly still there
  double prob_still_blocked;
  if (elapsed_at_arrival <= c) {
    prob_still_blocked = 1.0;
  } else {
    // P(still blocked | survived c) = S(elapsed) / S(c)
    double S_elapsed = survival_model_.survival(obs_type, elapsed_at_arrival);
    double S_c = survival_model_.survival(obs_type, c);
    
    if (S_c < 1e-15) {
      prob_still_blocked = 0.0;
    } else {
      prob_still_blocked = std::min(1.0, S_elapsed / S_c);
    }
  }
  
  double prob_cleared = 1.0 - prob_still_blocked;
  
  // E[wait | still blocked] = E[T | T > elapsed] - elapsed
  double expected_wait_if_blocked = 0.0;
  if (prob_still_blocked > 1e-10) {
    double cond_expected = survival_model_.conditionalExpectedTime(obs_type, elapsed_at_arrival);
    expected_wait_if_blocked = std::max(0.0, cond_expected - elapsed_at_arrival);
  }
  
  // E[wait | cleared] = expected wait from new obstacle
  double expected_wait_if_cleared = computeExpectedWaitForNewObstacle();
  
  return prob_still_blocked * expected_wait_if_blocked +
         prob_cleared * expected_wait_if_cleared;
}

route_planning::ExpectedWaitFn LearnedStrategy::createExpectedWaitFn(
    double t0,
    const EdgeId& exclude_edge) const {
  
  // Capture necessary data for the lambda
  double ew_fresh = computeExpectedWaitForNewObstacle();
  
  return [this, t0, exclude_edge, ew_fresh](const EdgeId& edge, double arrival_at_u) -> double {
    // If this is the exclude_edge (for A_goal), use no expected wait
    // (the override mechanism handles when the blocked edge clears)
    if (exclude_edge.isValid() && edge == exclude_edge) {
      return 0.0;
    }
    
    // Check memory
    auto mem_opt = memory_.getMemory(edge);
    if (!mem_opt.has_value()) {
      // Fresh edge
      return ew_fresh;
    }
    
    // Edge with memory
    return computeExpectedWaitForEdge(edge, arrival_at_u);
  };
}

WaitDecision LearnedStrategy::computeWaitTime(
    const std::string& obs_type,
    const EdgeIdSet& blocked_edges,
    const tactic::VertexId& current_vertex,
    const tactic::VertexId& goal_vertex,
    double t_now,
    double obstacle_t_first) {
  
  double W_max = config_.getWMax(obs_type);
  
  // Check if graph access is available
  if (!get_neighbors_ || !get_travel_time_) {
    CLOG(WARNING, "navigation") << "HSHMAT LearnedStrategy: No graph access, defaulting to wait";
    return WaitDecision::waitForever(obs_type + ". Waiting.");
  }
  
  // Record this obstacle in memory
  memory_.recordObstacle(blocked_edges, obs_type, t_now);
  
  // Check for conditional survival (old obstacle we've seen before)
  double elapsed = 0.0;
  bool use_conditional = false;
  if (obstacle_t_first > 0.0 && obstacle_t_first < t_now) {
    elapsed = t_now - obstacle_t_first;
    use_conditional = true;
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Computing W* for " << obs_type
                           << ", blocked_edges=" << blocked_edges.size()
                           << ", W_max=" << W_max
                           << ", elapsed=" << elapsed
                           << ", conditional=" << use_conditional
                           << ", current_v=" << current_vertex
                           << ", goal_v=" << goal_vertex;
  
  // Get one edge to use as the "blocked edge" for TDSP
  // All edges in blocked_edges are treated as one obstacle group
  EdgeId blocked_edge = EdgeId::Invalid();
  if (!blocked_edges.empty()) {
    blocked_edge = *blocked_edges.begin();
    CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Using blocked_edge=" << blocked_edge;
  }
  
  // Convert blocked_edges to forbidden set
  std::unordered_set<EdgeId, route_planning::EdgeIdHash> forbidden_for_avoid;
  for (const auto& e : blocked_edges) {
    forbidden_for_avoid.insert(e);
  }
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: forbidden_for_avoid has " << forbidden_for_avoid.size() << " edges";
  
  // ========================================================================
  // Step 1: Check if detour exists at all (A_avoid at t0)
  // ========================================================================
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Step 1 - checking detour existence...";
  auto ew_fn = createExpectedWaitFn(t_now, EdgeId::Invalid());
  auto result_avoid0 = tdsp_planner_.earliestArrival(
      current_vertex, goal_vertex, t_now,
      get_neighbors_, get_travel_time_, ew_fn,
      forbidden_for_avoid);
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Step 1 complete - detour check done";
  
  if (!result_avoid0.success || !std::isfinite(result_avoid0.arrival_time)) {
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: No alternate route, must wait";
    return WaitDecision::waitForever(obs_type + ". Waiting.");
  }
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Detour exists, arrival=" << result_avoid0.arrival_time;
  
  // ========================================================================
  // Step 2: Compute A_goal(t0 + t_clear[i]) for T_grid_points using TDSP
  //         with override_availability = t0 + t_clear[i] for blocked_edge
  // ========================================================================
  const int n_T = std::max(16, config_.T_grid_points);
  double dt = W_max / n_T;
  
  std::vector<double> t_clear(n_T);
  std::vector<double> override_times(n_T);
  for (int i = 0; i < n_T; ++i) {
    t_clear[i] = (i + 1) * dt;  // Right endpoints: dt, 2*dt, ..., W_max
    override_times[i] = t_now + t_clear[i];  // Global availability time
  }
  
  // Expected wait function for A_goal: exclude the blocked edge
  auto ew_fn_goal = createExpectedWaitFn(t_now, blocked_edge);
  
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Step 2 - computing A_goal batch (" << n_T << " points)...";
  // Batch compute A_goal for all override times
  std::vector<double> A_goal_arrivals = tdsp_planner_.batchOverrideArrivals(
      current_vertex, goal_vertex, t_now,
      get_neighbors_, get_travel_time_, ew_fn_goal,
      blocked_edge, override_times,
      {});  // No forbidden edges for A_goal
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Step 2 complete - A_goal batch done";
  
  // ========================================================================
  // Step 3: Compute A_avoid(t0 + W[j]) for W_grid_points using TDSP
  //         with start_time = t0 + W[j] and blocked_edge forbidden
  // ========================================================================
  const int n_W = std::max(16, config_.W_grid_points);
  
  std::vector<double> W_candidates(n_W + 1);
  std::vector<double> avoid_start_times(n_W + 1);
  for (int i = 0; i <= n_W; ++i) {
    W_candidates[i] = (static_cast<double>(i) / n_W) * W_max;
    avoid_start_times[i] = t_now + W_candidates[i];
  }
  
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Step 3 - computing A_avoid batch (" << (n_W+1) << " points)...";
  // Batch compute A_avoid for all start times
  std::vector<double> A_avoid_arrivals = tdsp_planner_.batchStartTimeArrivals(
      current_vertex, goal_vertex, avoid_start_times,
      get_neighbors_, get_travel_time_, ew_fn,
      forbidden_for_avoid);
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Step 3 complete - A_avoid batch done";
  
  // ========================================================================
  // Step 4: Compute probability masses p(t) = S(t_{i-1}) - S(t_i)
  // ========================================================================
  CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: Step 4 - computing probability masses...";
  std::vector<double> prob_masses(n_T);
  for (int i = 0; i < n_T; ++i) {
    double t_left = i * dt;
    double t_right = (i + 1) * dt;
    
    double S_left, S_right;
    if (use_conditional && elapsed > 0.0) {
      // Conditional survival: S(t | T > elapsed) = S(t + elapsed) / S(elapsed)
      double S_elapsed = survival_model_.survival(obs_type, elapsed);
      if (S_elapsed < 1e-15) {
        prob_masses[i] = 0.0;
        continue;
      }
      S_left = survival_model_.survival(obs_type, t_left + elapsed) / S_elapsed;
      S_right = survival_model_.survival(obs_type, t_right + elapsed) / S_elapsed;
    } else {
      S_left = survival_model_.survival(obs_type, t_left);
      S_right = survival_model_.survival(obs_type, t_right);
    }
    
    prob_masses[i] = std::max(0.0, S_left - S_right);
  }
  
  // ========================================================================
  // Step 5: Compute contributions and prefix sums for clear_term
  // ========================================================================
  std::vector<double> contrib(n_T);
  for (int i = 0; i < n_T; ++i) {
    // A_goal is arrival time; convert to elapsed time from t_now
    double A_goal_elapsed = A_goal_arrivals[i] - t_now;
    contrib[i] = prob_masses[i] * A_goal_elapsed;
  }
  
  std::vector<double> cum(n_T);
  cum[0] = contrib[0];
  for (int i = 1; i < n_T; ++i) {
    cum[i] = cum[i - 1] + contrib[i];
  }
  
  // ========================================================================
  // Step 6: Find optimal W* by minimizing J(W)
  // ========================================================================
  double best_W = 0.0;
  double best_J = A_avoid_arrivals[0] - t_now;  // J(0) = A_avoid(t0) - t0 = immediate detour time
  
  for (int i = 0; i <= n_W; ++i) {
    double W = W_candidates[i];
    
    // clear_term: sum of p(t) * A_goal(t) for t <= W
    double clear_term = 0.0;
    if (W > 0.0) {
      auto it = std::upper_bound(t_clear.begin(), t_clear.end(), W);
      if (it != t_clear.begin()) {
        int k = static_cast<int>(std::distance(t_clear.begin(), it)) - 1;
        clear_term = cum[k];
      }
    }
    
    // S(W) for avoid term
    double S_W;
    if (use_conditional && elapsed > 0.0) {
      double S_elapsed = survival_model_.survival(obs_type, elapsed);
      if (S_elapsed < 1e-15) {
        S_W = 0.0;
      } else {
        S_W = survival_model_.survival(obs_type, W + elapsed) / S_elapsed;
      }
    } else {
      S_W = survival_model_.survival(obs_type, W);
    }
    
    // A_avoid(t0 + W) - t0 = time to goal if we reroute at W
    double A_avoid_elapsed = A_avoid_arrivals[i] - t_now;
    double avoid_term = S_W * A_avoid_elapsed;
    
    double J = clear_term + avoid_term;
    
    if (J < best_J) {
      best_J = J;
      best_W = W;
    }
  }
  
  // ========================================================================
  // EPISODE SUMMARY - structured logging for debugging learned policy
  // ========================================================================
  {
    double A_avoid_0 = result_avoid0.arrival_time - t_now;
    double S_Wmax = use_conditional ? 
        (survival_model_.survival(obs_type, elapsed) > 1e-15 ? 
         survival_model_.survival(obs_type, W_max + elapsed) / survival_model_.survival(obs_type, elapsed) : 0.0)
        : survival_model_.survival(obs_type, W_max);
    
    CLOG(INFO, "navigation") << "========== LEARNED POLICY EPISODE SUMMARY ==========";
    CLOG(INFO, "navigation") << "  Obstacle type:     " << obs_type;
    CLOG(INFO, "navigation") << "  Blocked edges:     " << blocked_edges.size();
    CLOG(INFO, "navigation") << "  Position:          v=" << current_vertex << " -> goal=" << goal_vertex;
    CLOG(INFO, "navigation") << "  W_max:             " << W_max << "s";
    CLOG(INFO, "navigation") << "  Elapsed (if cont): " << elapsed << "s";
    CLOG(INFO, "navigation") << "  A_avoid(t0):       " << A_avoid_0 << "s (immediate detour time)";
    CLOG(INFO, "navigation") << "  S(W_max):          " << S_Wmax << " (prob still blocked after max wait)";
    CLOG(INFO, "navigation") << "  --> OPTIMAL W*:    " << best_W << "s";
    CLOG(INFO, "navigation") << "  --> J(W*):         " << best_J << "s (expected arrival time)";
    CLOG(INFO, "navigation") << "  --> DECISION:      " << (best_W < 1e-3 ? "REROUTE NOW" : ("WAIT " + std::to_string(static_cast<int>(std::ceil(best_W))) + "s"));
    CLOG(INFO, "navigation") << "=====================================================";
  }
  
  // Check if W* is effectively 0 (should detour immediately)
  if (best_W < 1e-3) {
    return WaitDecision::detour(obs_type + ". Rerouting.");
  }
  
  // Round to nice number for speech
  int wait_seconds = static_cast<int>(std::ceil(best_W));
  
  return WaitDecision::wait(best_W, buildWaitSpeech(obs_type, wait_seconds));
}

std::string LearnedStrategy::buildWaitSpeech(const std::string& obs_type, double W_star) const {
  // Speech format: "[type]. Waiting [time]." (Navigator already said "Obstacle detected.")
  std::ostringstream ss;
  ss << obs_type << ". Waiting ";
  
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
  
  // Record episode in global stats
  obstacle_stats_.recordObstacleEpisode(obs_type);
  
  // Save all data
  saveData();
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Recorded uncensored sample for "
                           << obs_type << ": " << wait_duration << "s"
                           << " (total episodes: " << obstacle_stats_.totalObstacleEpisodes() << ")";
}

void LearnedStrategy::onRerouteTimeout(const std::string& obs_type, double wait_duration) {
  // Censored sample: we rerouted before obstacle cleared
  survival_model_.addSample(obs_type, wait_duration, true);
  
  // Record episode in global stats
  obstacle_stats_.recordObstacleEpisode(obs_type);
  
  // Save all data
  saveData();
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Recorded censored sample for "
                           << obs_type << ": " << wait_duration << "s"
                           << " (total episodes: " << obstacle_stats_.totalObstacleEpisodes() << ")";
}

void LearnedStrategy::updateMemoryAfterCensoredWait(
    const EdgeIdSet& blocked_edges, 
    double t_after_wait) {
  // Update memory for all edges in the group
  for (const auto& edge : blocked_edges) {
    memory_.updateAfterCensoredWait(edge, t_after_wait);
  }
}

void LearnedStrategy::clearMemoryForEdge(const EdgeId& edge) {
  memory_.clearEdge(edge);
}

void LearnedStrategy::resetMemory() {
  memory_.reset();
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
