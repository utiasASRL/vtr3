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
#include <fstream>
#include <filesystem>
#include <regex>

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
  
  // p_block is computed from data (no default). Type weights still have a default
  // distribution for low-data regimes.
  obstacle_stats_.setDefaultTypeWeights(config_.type_weights);
  
  // Add seed samples
  for (const auto& kv : config_.seed_samples) {
    // Only add seeds if we don't have enough data yet
    if (survival_model_.sampleCount(kv.first) < kv.second.size()) {
      survival_model_.addSeedSamples(kv.first, kv.second);
    }
  }
  
  // Initialize debug plotting if enabled
  if (config_.debug_plot_policy) {
    debug_plot_dir_ = config_.debug_plot_dir.empty() 
        ? config_.learned_data_dir + "/debug_plots"
        : config_.debug_plot_dir;
    
    // Create directory if it doesn't exist
    try {
      std::filesystem::create_directories(debug_plot_dir_);
    } catch (const std::exception& e) {
      CLOG(WARNING, "navigation") << "HSHMAT LearnedStrategy: Failed to create debug_plot_dir: " << e.what();
    }
    
    // Use run_idx from config; the starting episode is auto-detected from
    // debug_plot_dir so resumed runs continue numbering correctly (mirrors how
    // RealWorldLogger scans episode_stats.csv). Navigator will still call
    // notifyEpisodeStart() with the authoritative mission episode once the
    // goal starts.
    run_number_ = config_.run_idx;
    episode_idx_ = detectStartingEpisode();
    encounter_in_episode_ = 0;

    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Debug plotting enabled, dir="
                             << debug_plot_dir_ << ", run=" << run_number_
                             << ", starting_episode=" << episode_idx_
                             << " (config.starting_episode=" << config_.starting_episode << ")";
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Initialized with "
                           << config_.W_grid_points << " W grid points, "
                           << config_.T_grid_points << " T grid points, "
                           << "p_block=" << obstacle_stats_.p_block()
                           << ", edges_traversed=" << obstacle_stats_.totalEdgesTraversed()
                           << ", episodes=" << obstacle_stats_.totalObstacleEpisodes()
                           << ", data_dir=" << config_.learned_data_dir;
}

void LearnedStrategy::saveData() {
  if (config_.learned_data_dir.empty()) return;
  
  // Save survival model
  survival_model_.saveToFile(survival_stats_file_);
  
  // Save obstacle statistics (includes edge count and episode counts)
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
    // Pass per-type W_max so the tail integration horizon matches the sim
    // (vtr3_sim agent.py:1582-1592, km.py mean_survival_time uses W_max_per_type).
    double mean_duration =
        survival_model_.meanSurvivalTime(obs_type, config_.getWMax(obs_type));
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
    // Pass per-type W_max so the conditional-expectation integration horizon
    // matches the sim (km.py conditional_expected_km uses W_max_per_type).
    double cond_expected = survival_model_.conditionalExpectedTime(
        obs_type, elapsed_at_arrival, config_.getWMax(obs_type));
    expected_wait_if_blocked = std::max(0.0, cond_expected - elapsed_at_arrival);
  }
  
  // E[wait | cleared] = expected wait from new obstacle
  double expected_wait_if_cleared = computeExpectedWaitForNewObstacle();
  
  return prob_still_blocked * expected_wait_if_blocked +
         prob_cleared * expected_wait_if_cleared;
}

route_planning::ExpectedWaitFn LearnedStrategy::createExpectedWaitFn(
    double t0,
    const EdgeIdSet& exclude_edges) const {
  
  // Capture necessary data for the lambda
  double ew_fresh = computeExpectedWaitForNewObstacle();
  
  return [this, t0, exclude_edges, ew_fresh](const EdgeId& edge, double arrival_at_u) -> double {
    // If this edge is in the exclude set (for A_goal), use no expected wait
    // (the override mechanism handles when the blocked edge clears)
    if (exclude_edges.count(edge) > 0) {
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
  auto ew_fn = createExpectedWaitFn(t_now, {});  // No exclusions for avoid path
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
    // NOTE: use the SAME formula as W_candidates below so that t_clear[i]
    // and any W_candidates[j] that represent the same point are bitwise
    // equal. Otherwise upper_bound() in the J(W) loop can drop a whole
    // KM-event mass from clear_term at exactly that boundary, leaving S(W)
    // and clear_term inconsistent and producing a single-row spurious dip
    // in J(W) (which can then be selected as a bogus W*).
    t_clear[i] = (static_cast<double>(i + 1) / n_T) * W_max;
    override_times[i] = t_now + t_clear[i];  // Global availability time
  }
  
  // Expected wait function for A_goal: exclude ALL blocked edges from expected wait
  // (the override mechanism handles when the primary blocked edge clears, and all
  // blocked edges are assumed to clear together since they're one obstacle)
  auto ew_fn_goal = createExpectedWaitFn(t_now, blocked_edges);
  
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
  // Collect J values for debug plotting
  std::vector<double> J_values(n_W + 1);
  
  for (int i = 0; i <= n_W; ++i) {
    double W = W_candidates[i];
    
    // clear_term: sum of p(t) * A_goal(t) for t <= W
    // Use a small tolerance so that bins whose right-endpoint equals W
    // (to within fp rounding) are INCLUDED. Must mirror how S(W) sees the
    // KM step at that t; otherwise clear_term and S_W disagree by one bin's
    // mass and J(W) gets a single-row spurious dip.
    double clear_term = 0.0;
    if (W > 0.0) {
      constexpr double kBoundaryEps = 1e-9;
      auto it = std::upper_bound(t_clear.begin(), t_clear.end(),
                                 W + kBoundaryEps);
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
    
    J_values[i] = clear_term + avoid_term;
  }

  // Argmin J(W) with tie-break matching Python sim (_w_star_min_idx_prefer_wmax):
  // if J(W_max) is within tolerance of min J, prefer W_max over earlier grid points.
  double best_J = J_values[0];
  for (double J : J_values) {
    best_J = std::min(best_J, J);
  }
  const double tol = 1e-6 * std::max(1.0, std::abs(best_J));
  int best_idx = 0;
  for (int i = 0; i <= n_W; ++i) {
    if (std::abs(J_values[i] - best_J) <= tol) {
      best_idx = i;
      break;
    }
  }
  if (std::abs(J_values[n_W] - best_J) <= tol) {
    best_idx = n_W;
  }
  double best_W = W_candidates[best_idx];

  // Cold-start: no KM samples for this type — S(t)=0 degenerates J(W); use W*=W_max,
  // J*=A_goal(immediate clear)-t_now. Still run full grid above so debug plots are saved.
  const bool cold_start_no_km = !survival_model_.hasData(obs_type);
  if (cold_start_no_km) {
    auto ew_fn_goal_cold = createExpectedWaitFn(t_now, blocked_edges);
    auto result_goal0 = tdsp_planner_.earliestArrival(
        current_vertex, goal_vertex, t_now,
        get_neighbors_, get_travel_time_, ew_fn_goal_cold,
        {});
    double A_goal_0 = result_goal0.success ? result_goal0.arrival_time : (t_now + W_max);
    best_W = W_max;
    best_J = A_goal_0 - t_now;
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: No KM data for '" << obs_type
                             << "' — cold-start override W*=" << best_W
                             << ", J*=A_goal(0)-t_now=" << best_J;
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
    
    // Compute detailed statistics for the avoid path
    double p_block_val = obstacle_stats_.p_block();
    double ew_fresh = computeExpectedWaitForNewObstacle();
    
    // Compute pure travel time (no expected waits) along the avoid path
    // by calling TDSP with zero expected-wait function
    auto zero_ew_fn = [](const EdgeId&, double) -> double { return 0.0; };
    auto pure_travel_result = tdsp_planner_.earliestArrival(
        current_vertex, goal_vertex, t_now,
        get_neighbors_, get_travel_time_, zero_ew_fn,
        forbidden_for_avoid);
    double pure_travel_time = pure_travel_result.success ? 
        (pure_travel_result.arrival_time - t_now) : -1.0;
    int avoid_path_length = pure_travel_result.success ? 
        static_cast<int>(pure_travel_result.path.size()) - 1 : -1;  // edges = vertices - 1
    
    // Expected wait added by TDSP = A_avoid - pure_travel
    double total_expected_wait = A_avoid_0 - pure_travel_time;
    double avg_ew_per_edge = (avoid_path_length > 0) ? 
        (total_expected_wait / avoid_path_length) : 0.0;
    
    // Get type distribution info
    auto type_dist = obstacle_stats_.getTypeDistribution();
    double mean_survival_this_type = survival_model_.meanSurvivalTime(obs_type);
    
    CLOG(INFO, "navigation") << "========== LEARNED POLICY EPISODE SUMMARY ==========";
    CLOG(INFO, "navigation") << "  Obstacle type:     " << obs_type;
    CLOG(INFO, "navigation") << "  Blocked edges:     " << blocked_edges.size();
    CLOG(INFO, "navigation") << "  Position:          v=" << current_vertex << " -> goal=" << goal_vertex;
    CLOG(INFO, "navigation") << "  W_max:             " << W_max << "s";
    CLOG(INFO, "navigation") << "  Elapsed (if cont): " << elapsed << "s";
    CLOG(INFO, "navigation") << "  ---------- AVOID PATH STATS ----------";
    CLOG(INFO, "navigation") << "  A_avoid(t0):       " << A_avoid_0 << "s (immediate detour time)";
    CLOG(INFO, "navigation") << "  Avoid path length: " << avoid_path_length << " edges";
    CLOG(INFO, "navigation") << "  Pure travel time:  " << pure_travel_time << "s (no expected waits)";
    CLOG(INFO, "navigation") << "  Total E[wait]:     " << total_expected_wait << "s (added by TDSP)";
    CLOG(INFO, "navigation") << "  Avg E[wait]/edge:  " << avg_ew_per_edge << "s";
    CLOG(INFO, "navigation") << "  ---------- LEARNED PARAMS ----------";
    CLOG(INFO, "navigation") << "  p_block:           " << p_block_val
                             << " (episodes/edges = " << obstacle_stats_.totalObstacleEpisodes()
                             << "/" << obstacle_stats_.totalEdgesTraversed() << ")";
    CLOG(INFO, "navigation") << "  E[wait|new obs]:   " << ew_fresh << "s (p_block * E[type * duration])";
    CLOG(INFO, "navigation") << "  Mean survival("<<obs_type<<"): " << mean_survival_this_type << "s";
    CLOG(INFO, "navigation") << "  Edges traversed:   " << obstacle_stats_.totalEdgesTraversed();
    CLOG(INFO, "navigation") << "  Obstacle episodes: " << obstacle_stats_.totalObstacleEpisodes();
    CLOG(INFO, "navigation") << "  ---------- DECISION ----------";
    CLOG(INFO, "navigation") << "  S(W_max):          " << S_Wmax << " (prob still blocked after max wait)";
    CLOG(INFO, "navigation") << "  --> OPTIMAL W*:    " << best_W << "s";
    CLOG(INFO, "navigation") << "  --> J(W*):         " << best_J << "s (expected arrival time)";
    CLOG(INFO, "navigation") << "  --> DECISION:      " << (best_W < 1e-3 ? "REROUTE NOW" : ("WAIT " + std::to_string(static_cast<int>(std::ceil(best_W))) + "s"));
    CLOG(INFO, "navigation") << "=====================================================";

    // ======================================================================
    // REAL PARITY: dump the chosen A_avoid(W=0) path edge-by-edge so we can
    // compare vertex sequences and per-edge (travel, EW) with the simulator's
    // [SIM PARITY] log. Keeps formatting symmetric with agent.py:
    //   _log_avoid_path_for_parity.
    // ======================================================================
    {
      const auto& path = result_avoid0.path;
      std::ostringstream blocked_ss;
      blocked_ss << "[";
      bool first = true;
      for (const auto& e : forbidden_for_avoid) {
        if (!first) blocked_ss << ", ";
        blocked_ss << "((" << e.majorId1() << "," << e.minorId1() << "),("
                   << e.majorId2() << "," << e.minorId2() << "))";
        first = false;
      }
      blocked_ss << "]";

      CLOG(INFO, "navigation") << "[REAL PARITY] LearnedStrategy A_avoid(W=0) path";
      CLOG(INFO, "navigation") << "  obs_type=" << obs_type
                               << "  t0=" << std::fixed << std::setprecision(3) << t_now;
      CLOG(INFO, "navigation") << "  start_vertex=" << current_vertex
                               << "  goal_vertex=" << goal_vertex;
      CLOG(INFO, "navigation") << "  blocked_edges=" << blocked_ss.str();
      CLOG(INFO, "navigation") << "  p_block(frozen)=" << std::fixed << std::setprecision(6)
                               << p_block_val;
      CLOG(INFO, "navigation") << "  ew_fresh(p_block * E[type*duration])="
                               << std::fixed << std::setprecision(4) << ew_fresh << "s";
      CLOG(INFO, "navigation") << "  A_avoid_elapsed = A_avoid - t0 = "
                               << std::fixed << std::setprecision(4) << A_avoid_0 << "s";
      CLOG(INFO, "navigation") << "  path_len_nodes=" << path.size()
                               << "  path_edges=" << (path.empty() ? 0 : path.size() - 1);

      // Dump the full vertex sequence (parity with sim)
      {
        std::ostringstream ps;
        ps << "  path=[";
        for (size_t i = 0; i < path.size(); ++i) {
          if (i) ps << ", ";
          ps << "(" << path[i].majorId() << "," << path[i].minorId() << ")";
        }
        ps << "]";
        CLOG(INFO, "navigation") << ps.str();
      }

      // Per-edge breakdown using the SAME ew_fn that drove the TDSP
      CLOG(INFO, "navigation") << "  per-edge breakdown:";
      CLOG(INFO, "navigation") << "    idx          edge                          travel   E[wait]  arrival";
      double t_cursor = t_now;
      double sum_travel = 0.0;
      double sum_ew = 0.0;
      const int n_edges = static_cast<int>(path.empty() ? 0 : path.size() - 1);
      const int print_head = 5;
      const int print_tail = 5;
      for (int i = 0; i < n_edges; ++i) {
        EdgeId edge(path[i], path[i + 1]);
        double travel = get_travel_time_(edge);
        double ew = ew_fn(edge, t_cursor);
        sum_travel += travel;
        sum_ew += ew;
        t_cursor += travel + ew;
        if (i < print_head || i >= n_edges - print_tail) {
          std::ostringstream line;
          line << "    " << std::setw(4) << i << "  "
               << "(" << path[i].majorId() << "," << path[i].minorId() << ")"
               << " -> "
               << "(" << path[i + 1].majorId() << "," << path[i + 1].minorId() << ")"
               << "  " << std::fixed << std::setprecision(3) << std::setw(8) << travel
               << "  " << std::fixed << std::setprecision(3) << std::setw(8) << ew
               << "  " << std::fixed << std::setprecision(3) << std::setw(9) << (t_cursor - t_now);
          CLOG(INFO, "navigation") << line.str();
        } else if (i == print_head) {
          CLOG(INFO, "navigation") << "    ... (" << (n_edges - print_head - print_tail)
                                   << " interior edges omitted) ...";
        }
      }
      CLOG(INFO, "navigation") << "  totals: travel=" << std::fixed << std::setprecision(3)
                               << sum_travel << "s  ew_sum=" << sum_ew
                               << "s  total_elapsed=" << (sum_travel + sum_ew) << "s";
    }
  }
  
  // ========================================================================
  // Debug plotting (if enabled)
  // ========================================================================
  if (config_.debug_plot_policy) {
    saveDebugPlots(obs_type, W_max, elapsed, use_conditional,
                   t_clear, A_goal_arrivals, W_candidates, A_avoid_arrivals,
                   prob_masses, J_values, t_now, best_W, best_J);
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

void LearnedStrategy::onObstacleCleared(const std::string& obs_type, double wait_duration, int episode) {
  // Buffer uncensored sample (will be added to KM at episode end to match sim behavior)
  // Also buffer the obstacle type for stats update at episode end
  pending_samples_.push_back({obs_type, wait_duration, false, episode});
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Buffered uncensored sample for "
                           << obs_type << ": " << wait_duration << "s, episode=" << episode
                           << " (pending=" << pending_samples_.size() << ")";
}

void LearnedStrategy::onRerouteTimeout(const std::string& obs_type, double wait_duration, int episode) {
  // Buffer censored sample (will be added to KM at episode end to match sim behavior)
  // Also buffer the obstacle type for stats update at episode end
  pending_samples_.push_back({obs_type, wait_duration, true, episode});
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Buffered censored sample for "
                           << obs_type << ": " << wait_duration << "s, episode=" << episode
                           << " (pending=" << pending_samples_.size() << ")";
}

void LearnedStrategy::flushPendingSamplesToKM() {
  if (pending_samples_.empty()) {
    CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: No pending samples to flush";
    return;
  }
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Flushing " << pending_samples_.size() 
                           << " pending samples to KM and obstacle stats (per-episode update)";
  
  for (const auto& sample : pending_samples_) {
    // Add to KM survival model
    survival_model_.addSample(sample.obs_type, sample.duration, sample.censored, sample.episode);
    // Update obstacle stats (p_block numerator and type counts)
    obstacle_stats_.recordObstacleEpisode(sample.obs_type);
    CLOG(DEBUG, "navigation") << "  -> Added " << (sample.censored ? "censored" : "uncensored")
                              << " sample: " << sample.obs_type << " " << sample.duration << "s"
                              << ", episode=" << sample.episode;
  }
  pending_samples_.clear();
  
  // Save all data after batch update
  saveData();
  
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: KM model and obstacle stats updated and saved"
                           << " (total_obstacle_episodes=" << obstacle_stats_.totalObstacleEpisodes() << ")";
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

void LearnedStrategy::seedMemoryForEdge(const EdgeId& edge,
                                        const std::string& obs_type,
                                        double t_first,
                                        double t_last_confirmed) {
  EdgeIdSet edges;
  edges.insert(edge);
  memory_.recordObstacle(edges, obs_type, t_first);
  if (t_last_confirmed > t_first) {
    memory_.updateAfterCensoredWait(edge, t_last_confirmed);
  }
}

void LearnedStrategy::resetMemory() {
  memory_.reset();
}

void LearnedStrategy::notifyEpisodeStart(int episode_idx) {
  // Use the navigator's mission episode number directly so debug-plot filenames
  // align with the episode counter in user-facing logs. encounter_in_episode_
  // resets so the next saveDebugPlots() call writes ...encounter1_*.
  if (episode_idx > 0) {
    episode_idx_ = episode_idx;
  }
  encounter_in_episode_ = 0;
  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: notifyEpisodeStart -> episode "
                           << episode_idx_ << " (encounter counter reset)";
}

int LearnedStrategy::detectRunNumber() const {
  int max_run = 0;
  
  if (!std::filesystem::exists(debug_plot_dir_)) {
    return 1;
  }
  
  // Scan for files matching pattern run{N}_episode{M}_* (with or without
  // _encounter{K} suffix). Older outputs used run{N}_episode{M} where M was
  // actually an encounter counter; new outputs use run{N}_episode{M}_encounter{K}.
  std::regex run_pattern(R"(run(\d+)_episode\d+)");
  
  try {
    for (const auto& entry : std::filesystem::directory_iterator(debug_plot_dir_)) {
      if (!entry.is_regular_file()) continue;
      
      std::string filename = entry.path().filename().string();
      std::smatch match;
      if (std::regex_search(filename, match, run_pattern)) {
        int run_num = std::stoi(match[1].str());
        max_run = std::max(max_run, run_num);
      }
    }
  } catch (const std::exception& e) {
    CLOG(WARNING, "navigation") << "HSHMAT LearnedStrategy: Error scanning debug_plot_dir: " << e.what();
  }
  
  return max_run + 1;  // Next run number
}

int LearnedStrategy::detectStartingEpisode() const {
  // Default fallback if we cannot scan: trust the config value.
  const int fallback = std::max(1, config_.starting_episode);

  if (debug_plot_dir_.empty() || !std::filesystem::exists(debug_plot_dir_)) {
    return fallback;
  }

  // Match the NEW naming format only: run{run_number_}_episode{M}_encounter{K}_*.
  // Old-format files (run{N}_episode{M} without _encounter) are ignored so we
  // do not confuse the legacy encounter counter with an episode index.
  const std::string run_prefix = "run" + std::to_string(run_number_);
  std::regex episode_pattern(run_prefix + R"(_episode(\d+)_encounter\d+)");

  int max_ep = 0;
  try {
    for (const auto& entry : std::filesystem::directory_iterator(debug_plot_dir_)) {
      if (!entry.is_regular_file()) continue;
      const std::string filename = entry.path().filename().string();
      std::smatch match;
      if (std::regex_search(filename, match, episode_pattern)) {
        int ep = std::stoi(match[1].str());
        max_ep = std::max(max_ep, ep);
      }
    }
  } catch (const std::exception& e) {
    CLOG(WARNING, "navigation")
        << "HSHMAT LearnedStrategy: Error scanning debug_plot_dir for episode: " << e.what();
    return fallback;
  }

  // Next episode to log = max(M) + 1, but never below the config-provided value
  // so resume logic in RealWorldLogger remains the lower bound.
  return std::max(fallback, max_ep + 1);
}

void LearnedStrategy::saveDebugPlots(
    const std::string& obs_type,
    double W_max,
    double elapsed,
    bool use_conditional,
    const std::vector<double>& t_clear,
    const std::vector<double>& A_goal_arrivals,
    const std::vector<double>& W_candidates,
    const std::vector<double>& A_avoid_arrivals,
    const std::vector<double>& prob_masses,
    const std::vector<double>& J_values,
    double t_now,
    double best_W,
    double best_J) {
  
  // Each call to saveDebugPlots corresponds to one obstacle encounter within the
  // current episode. encounter_in_episode_ resets when Navigator notifies us of
  // a new episode (see notifyEpisodeStart).
  ++encounter_in_episode_;

  std::string prefix = debug_plot_dir_ + "/run" + std::to_string(run_number_)
                     + "_episode" + std::to_string(episode_idx_)
                     + "_encounter" + std::to_string(encounter_in_episode_);

  CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Saving debug plots to " << prefix << "_*.csv";
  
  // === Save data to CSV files for external plotting ===
  
  // 1. Survival function S(t)
  {
    std::ofstream f(prefix + "_survival.csv");
    // Prefix header with '#' so gnuplot treats it as a comment.
    f << "# t,S_t,p_t\n";
    f << std::fixed << std::setprecision(6);
    
    const int n_pts = 200;
    for (int i = 0; i <= n_pts; ++i) {
      double t = (static_cast<double>(i) / n_pts) * W_max;
      double S_t;
      if (use_conditional && elapsed > 0.0) {
        double S_elapsed = survival_model_.survival(obs_type, elapsed);
        S_t = (S_elapsed > 1e-15) ? survival_model_.survival(obs_type, t + elapsed) / S_elapsed : 0.0;
      } else {
        S_t = survival_model_.survival(obs_type, t);
      }
      // PDF approximation: -dS/dt (finite difference)
      double p_t = 0.0;
      if (i > 0) {
        double t_prev = (static_cast<double>(i - 1) / n_pts) * W_max;
        double S_prev;
        if (use_conditional && elapsed > 0.0) {
          double S_elapsed = survival_model_.survival(obs_type, elapsed);
          S_prev = (S_elapsed > 1e-15) ? survival_model_.survival(obs_type, t_prev + elapsed) / S_elapsed : 0.0;
        } else {
          S_prev = survival_model_.survival(obs_type, t_prev);
        }
        double dt = W_max / n_pts;
        p_t = std::max(0.0, (S_prev - S_t) / dt);
      }
      f << t << "," << S_t << "," << p_t << "\n";
    }
    f.close();
  }
  
  // 2. A_goal(C) - arrival time at goal as function of clearance time C
  {
    std::ofstream f(prefix + "_Agoal.csv");
    f << "# C,A_goal_elapsed\n";
    f << std::fixed << std::setprecision(6);
    
    for (size_t i = 0; i < t_clear.size(); ++i) {
      double A_goal_elapsed = A_goal_arrivals[i] - t_now;
      f << t_clear[i] << "," << A_goal_elapsed << "\n";
    }
    f.close();
  }
  
  // 3. A_avoid(W) - arrival time at goal if rerouting at wait time W
  {
    std::ofstream f(prefix + "_Aavoid.csv");
    f << "# W,A_avoid_elapsed\n";
    f << std::fixed << std::setprecision(6);
    
    for (size_t i = 0; i < W_candidates.size(); ++i) {
      double A_avoid_elapsed = A_avoid_arrivals[i] - t_now;
      f << W_candidates[i] << "," << A_avoid_elapsed << "\n";
    }
    f.close();
  }
  
  // 4. Probability masses p(t) for each bin
  {
    std::ofstream f(prefix + "_prob_masses.csv");
    f << "# bin,t_left,t_right,p_mass\n";
    f << std::fixed << std::setprecision(6);
    
    double dt = W_max / t_clear.size();
    for (size_t i = 0; i < prob_masses.size(); ++i) {
      double t_left = i * dt;
      double t_right = (i + 1) * dt;
      f << i << "," << t_left << "," << t_right << "," << prob_masses[i] << "\n";
    }
    f.close();
  }
  
  // 5. J(W) - the objective function
  {
    std::ofstream f(prefix + "_J.csv");
    f << "# W,J,best\n";
    f << std::fixed << std::setprecision(6);
    
    for (size_t i = 0; i < W_candidates.size() && i < J_values.size(); ++i) {
      int is_best = (std::abs(W_candidates[i] - best_W) < 1e-6) ? 1 : 0;
      f << W_candidates[i] << "," << J_values[i] << "," << is_best << "\n";
    }
    f.close();
  }
  
  // 6. Summary metadata
  {
    std::ofstream f(prefix + "_summary.yaml");
    f << "# Debug plot summary for run " << run_number_
      << " episode " << episode_idx_
      << " encounter " << encounter_in_episode_ << "\n";
    f << "obs_type: " << obs_type << "\n";
    f << "W_max: " << W_max << "\n";
    f << "elapsed: " << elapsed << "\n";
    f << "use_conditional: " << (use_conditional ? "true" : "false") << "\n";
    f << "t_now: " << std::fixed << std::setprecision(3) << t_now << "\n";
    f << "best_W: " << best_W << "\n";
    f << "best_J: " << best_J << "\n";
    f << "n_T_points: " << t_clear.size() << "\n";
    f << "n_W_points: " << W_candidates.size() << "\n";
    f.close();
  }
  
  // === Generate gnuplot script for easy visualization ===
  {
    std::ofstream f(prefix + "_plot.gp");
    f << "# Gnuplot script for learned policy debug visualization\n";
    f << "# Run with: gnuplot " << prefix << "_plot.gp\n\n";
    f << "set terminal pngcairo size 1600,1200 enhanced font 'Arial,12'\n";
    f << "set output '" << prefix << "_combined.png'\n\n";
    // CSV files are comma-separated; tell gnuplot how to split columns.
    f << "set datafile separator ','\n\n";
    f << "set multiplot layout 2,2 title 'Learned Policy: " << obs_type
      << " (run " << run_number_
      << " ep " << episode_idx_
      << " enc " << encounter_in_episode_ << ")' font ',14'\n\n";
    
    // Plot 1: Survival function
    f << "set title 'Survival Function S(t)'\n";
    f << "set xlabel 'Time (s)'\n";
    f << "set ylabel 'Probability'\n";
    f << "set yrange [0:1.1]\n";
    f << "plot '" << prefix << "_survival.csv' using 1:2 with lines lw 2 title 'S(t)',\\\n";
    f << "     '' using 1:3 with lines lw 2 title 'p(t) (PDF)'\n\n";
    
    // Plot 2: A_goal(C)
    f << "set title 'A_{goal}(C) - Arrival if clearing at C'\n";
    f << "set xlabel 'Clearance time C (s)'\n";
    f << "set ylabel 'Arrival time (s from now)'\n";
    f << "set autoscale y\n";
    f << "plot '" << prefix << "_Agoal.csv' using 1:2 with linespoints lw 2 pt 7 ps 0.5 title 'A_{goal}(C)'\n\n";
    
    // Plot 3: A_avoid(W)
    f << "set title 'A_{avoid}(W) - Arrival if rerouting at W'\n";
    f << "set xlabel 'Wait time W (s)'\n";
    f << "set ylabel 'Arrival time (s from now)'\n";
    f << "plot '" << prefix << "_Aavoid.csv' using 1:2 with linespoints lw 2 pt 7 ps 0.5 title 'A_{avoid}(W)'\n\n";
    
    // Plot 4: J(W) objective
    f << "set title 'J(W) - Objective Function (W* = " << std::fixed << std::setprecision(1) << best_W << "s)'\n";
    f << "set xlabel 'Wait time W (s)'\n";
    f << "set ylabel 'Expected arrival time (s)'\n";
    f << "set arrow from " << best_W << ",graph 0 to " << best_W << ",graph 1 nohead lc rgb 'red' lw 2\n";
    f << "plot '" << prefix << "_J.csv' using 1:2 with linespoints lw 2 pt 7 ps 0.5 title 'J(W)'\n\n";
    
    f << "unset multiplot\n";
    f << "unset arrow\n";
    f.close();
  }
  
  // Try to run gnuplot if available
  std::string gp_cmd = "gnuplot " + prefix + "_plot.gp 2>/dev/null";
  int ret = std::system(gp_cmd.c_str());
  if (ret == 0) {
    CLOG(INFO, "navigation") << "HSHMAT LearnedStrategy: Generated plot: " << prefix << "_combined.png";
  } else {
    CLOG(DEBUG, "navigation") << "HSHMAT LearnedStrategy: gnuplot not available or failed, CSV files saved for manual plotting";
  }
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
