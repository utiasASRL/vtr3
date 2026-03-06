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
 * \file ew_tdsp_planner.cpp
 * \brief Implementation of Expected-Wait TDSP planner.
 *
 * HSHMAT: Port of Python TDSP with expected wait to C++.
 */

#include "vtr_route_planning/ew_tdsp_planner.hpp"

#include <algorithm>
#include <cmath>

namespace vtr {
namespace route_planning {

TDSPResult EWTDSPPlanner::earliestArrival(
    const VertexId& start,
    const VertexId& goal,
    double start_time,
    const NeighborsFn& get_neighbors,
    const TravelTimeFn& get_travel_time,
    const ExpectedWaitFn& get_expected_wait,
    const EdgeIdSet& forbidden_edges,
    const std::optional<EdgeId>& override_edge,
    const std::optional<double>& override_availability) const {
  
  // Early exit: start == goal
  if (start == goal) {
    return TDSPResult(start_time, {start}, true);
  }

  // Best arrival times
  std::unordered_map<VertexId, double> best_arrival;
  best_arrival.reserve(1024);
  best_arrival[start] = start_time;

  // Predecessors for path reconstruction
  std::unordered_map<VertexId, VertexId> predecessor;
  predecessor.reserve(1024);

  // Priority queue: min-heap by arrival time
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
  pq.push({start_time, start});

  // Visited set
  std::unordered_set<VertexId> visited;
  visited.reserve(1024);

  while (!pq.empty()) {
    auto [arrival_u, u] = pq.top();
    pq.pop();

    // Skip if already visited
    if (visited.count(u)) continue;
    visited.insert(u);

    // Goal reached
    if (u == goal) {
      PathType path;
      VertexId cur = goal;
      path.push_back(cur);
      while (cur != start) {
        auto it = predecessor.find(cur);
        if (it == predecessor.end()) break;
        cur = it->second;
        path.push_back(cur);
      }
      std::reverse(path.begin(), path.end());
      return TDSPResult(arrival_u, path, true);
    }

    // Expand neighbors
    auto neighbors = get_neighbors(u);
    for (const auto& v : neighbors) {
      EdgeId edge(u, v);

      // Skip forbidden edges
      if (forbidden_edges.count(edge)) continue;

      // Get base travel time
      double travel_time = get_travel_time(edge);
      if (!std::isfinite(travel_time) || travel_time < 0) continue;

      // Compute expected wait or use override
      double expected_wait = 0.0;
      if (override_edge.has_value() && edge == override_edge.value()) {
        // Override mode: edge becomes available at override_availability
        // Wait until available, then traverse
        double t_available = override_availability.value_or(0.0);
        double wait_time = std::max(0.0, t_available - arrival_u);
        expected_wait = wait_time;
      } else {
        // Normal mode: use expected wait function
        expected_wait = get_expected_wait(edge, arrival_u);
      }

      // Arrival at v = arrival at u + expected_wait + travel_time
      double arrival_v = arrival_u + expected_wait + travel_time;

      // Update if better
      auto it = best_arrival.find(v);
      if (it == best_arrival.end() || arrival_v < it->second) {
        best_arrival[v] = arrival_v;
        predecessor[v] = u;
        pq.push({arrival_v, v});
      }
    }
  }

  // No path found
  return TDSPResult();
}

std::vector<double> EWTDSPPlanner::batchOverrideArrivals(
    const VertexId& start,
    const VertexId& goal,
    double start_time,
    const NeighborsFn& get_neighbors,
    const TravelTimeFn& get_travel_time,
    const ExpectedWaitFn& get_expected_wait,
    const EdgeId& override_edge,
    const std::vector<double>& override_times,
    const std::unordered_set<EdgeId, EdgeIdHash>& forbidden_edges) const {
  
  std::vector<double> results;
  results.reserve(override_times.size());

  for (double t_override : override_times) {
    auto result = earliestArrival(
        start, goal, start_time,
        get_neighbors, get_travel_time, get_expected_wait,
        forbidden_edges,
        override_edge, t_override);
    results.push_back(result.arrival_time);
  }

  return results;
}

std::vector<double> EWTDSPPlanner::batchStartTimeArrivals(
    const VertexId& start,
    const VertexId& goal,
    const std::vector<double>& start_times,
    const NeighborsFn& get_neighbors,
    const TravelTimeFn& get_travel_time,
    const ExpectedWaitFn& get_expected_wait,
    const std::unordered_set<EdgeId, EdgeIdHash>& forbidden_edges) const {
  
  std::vector<double> results;
  results.reserve(start_times.size());

  for (double t_start : start_times) {
    auto result = earliestArrival(
        start, goal, t_start,
        get_neighbors, get_travel_time, get_expected_wait,
        forbidden_edges,
        std::nullopt, std::nullopt);
    results.push_back(result.arrival_time);
  }

  return results;
}

}  // namespace route_planning
}  // namespace vtr
