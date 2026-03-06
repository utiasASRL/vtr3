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
 * \file ew_tdsp_planner.hpp
 * \brief Expected-Wait TDSP planner for learned obstacle strategies.
 *
 * HSHMAT: This planner computes earliest arrival times using expected wait
 * times instead of deterministic blockage intervals. This matches the Python
 * simulation's TDSP with expected wait.
 *
 * Model:
 *   arrival(v) = arrival(u) + travel_time(u,v) + expected_wait(u,v, arrival(u))
 *
 * The expected_wait function is provided by the caller and depends on:
 * - Whether the edge has memory (we saw an obstacle and gave up)
 * - The survival model for that obstacle type
 * - p_block for new obstacles on edges without memory
 */

#pragma once

#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "vtr_tactic/types.hpp"

namespace vtr {
namespace route_planning {

// Hash for EdgeId in unordered containers - defined BEFORE use
struct EdgeIdHash {
  std::size_t operator()(const tactic::EdgeId& e) const {
    return e.hash();
  }
};

// Convenience type alias
using EdgeIdSet = std::unordered_set<tactic::EdgeId, EdgeIdHash>;

/**
 * \brief Result of a single TDSP query.
 */
struct TDSPResult {
  double arrival_time;               // Earliest arrival time at goal
  tactic::PathType path;             // Path from start to goal
  bool success;                      // Whether a path was found
  
  TDSPResult() : arrival_time(std::numeric_limits<double>::infinity()), 
                 success(false) {}
  TDSPResult(double t, tactic::PathType p, bool s) 
      : arrival_time(t), path(std::move(p)), success(s) {}
};

/**
 * \brief Function type for computing expected wait for an edge.
 * 
 * Parameters:
 *   edge: The edge (u, v)
 *   arrival_at_u: Planned arrival time at node u
 * 
 * Returns: Expected wait time in seconds
 */
using ExpectedWaitFn = std::function<double(const tactic::EdgeId&, double arrival_at_u)>;

/**
 * \brief Function type for getting edge travel time (edge length / speed).
 */
using TravelTimeFn = std::function<double(const tactic::EdgeId&)>;

/**
 * \brief Function type for getting neighbors of a vertex.
 */
using NeighborsFn = std::function<std::vector<tactic::VertexId>(const tactic::VertexId&)>;

/**
 * \brief Expected-Wait TDSP Planner.
 * 
 * Computes shortest paths where edge costs include expected wait times
 * based on survival models, rather than deterministic blockage intervals.
 */
class EWTDSPPlanner {
 public:
  using VertexId = tactic::VertexId;
  using EdgeId = tactic::EdgeId;
  using PathType = tactic::PathType;

  EWTDSPPlanner() = default;

  /**
   * \brief Compute earliest arrival from start to goal.
   * 
   * \param start Starting vertex
   * \param goal Goal vertex
   * \param start_time Departure time from start
   * \param get_neighbors Function to get neighbors of a vertex
   * \param get_travel_time Function to get base travel time for an edge
   * \param get_expected_wait Function to get expected wait for an edge
   * \param forbidden_edges Edges that cannot be traversed
   * \param override_edge Optional: treat this edge as having a specific availability time
   * \param override_availability If override_edge is set, the edge becomes available at this time
   * \return TDSPResult with arrival time and path
   */
  TDSPResult earliestArrival(
      const VertexId& start,
      const VertexId& goal,
      double start_time,
      const NeighborsFn& get_neighbors,
      const TravelTimeFn& get_travel_time,
      const ExpectedWaitFn& get_expected_wait,
      const EdgeIdSet& forbidden_edges = {},
      const std::optional<EdgeId>& override_edge = std::nullopt,
      const std::optional<double>& override_availability = std::nullopt) const;

  /**
   * \brief Batch compute arrivals for multiple override_availability values.
   * 
   * This is optimized for the W* computation where we need:
   * - T_grid_points A_goal calls (override_availability = t0 + t_clear[i])
   * - W_grid_points A_avoid calls (different start_time = t0 + W[j], forbidden_edge = blocked_edge)
   * 
   * \param start Starting vertex
   * \param goal Goal vertex
   * \param start_time Departure time from start
   * \param get_neighbors Function to get neighbors
   * \param get_travel_time Function to get travel time
   * \param get_expected_wait Function to get expected wait
   * \param override_edge Edge to override availability for
   * \param override_times Array of availability times for the override edge
   * \param forbidden_edges Edges that cannot be traversed
   * \return Vector of arrival times (same length as override_times)
   */
  std::vector<double> batchOverrideArrivals(
      const VertexId& start,
      const VertexId& goal,
      double start_time,
      const NeighborsFn& get_neighbors,
      const TravelTimeFn& get_travel_time,
      const ExpectedWaitFn& get_expected_wait,
      const EdgeId& override_edge,
      const std::vector<double>& override_times,
      const std::unordered_set<EdgeId, struct EdgeIdHash>& forbidden_edges = {}) const;

  /**
   * \brief Batch compute arrivals for multiple start times with forbidden edge.
   * 
   * For A_avoid computation: start at t0+W, block the obstacle edge.
   * 
   * \param start Starting vertex
   * \param goal Goal vertex
   * \param start_times Vector of departure times
   * \param get_neighbors Function to get neighbors
   * \param get_travel_time Function to get travel time
   * \param get_expected_wait Function to get expected wait
   * \param forbidden_edges Edges that cannot be traversed (includes blocked edge)
   * \return Vector of arrival times
   */
  std::vector<double> batchStartTimeArrivals(
      const VertexId& start,
      const VertexId& goal,
      const std::vector<double>& start_times,
      const NeighborsFn& get_neighbors,
      const TravelTimeFn& get_travel_time,
      const ExpectedWaitFn& get_expected_wait,
      const EdgeIdSet& forbidden_edges) const;

 private:
  struct QItem {
    double t;
    VertexId v;
    bool operator>(const QItem& other) const { return t > other.t; }
  };
};

}  // namespace route_planning
}  // namespace vtr
