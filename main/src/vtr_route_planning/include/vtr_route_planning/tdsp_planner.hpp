// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file tdsp_planner.hpp
 * \brief FIFO time-dependent shortest path (label-setting) route planner.
 *
 * Implements the label-setting (Dijkstra-like) algorithm on earliest-arrival
 * times using FIFO arc arrival functions:
 *
 *   A_ij(t) = depart(t, blockage_ij) + travel_time_ij + static_delay_ij
 *   depart(t, blockage) = (t < end && t >= start) ? end : t
 *
 * If an edge blockage interval is overwritten, the newest interval fully
 * replaces the old one (handled by the owner of the blockage map).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "vtr_route_planning/route_planner_interface.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace route_planning {

class TDSPPlanner : public RoutePlannerInterface {
 public:
  PTR_TYPEDEFS(TDSPPlanner);

  using GraphPtr = tactic::Graph::Ptr;
  using GraphWeakPtr = tactic::Graph::WeakPtr;
  using GraphBasePtr = tactic::GraphBase::Ptr;
  using VertexId = tactic::VertexId;
  using EdgeId = tactic::EdgeId;
  using PathType = tactic::PathType;

  struct BlockageInterval {
    double start_sec = 0.0;
    double end_sec = 0.0;  // start + duration
  };

  explicit TDSPPlanner(const GraphPtr &graph) : graph_(graph) {}

  void setNominalSpeed(const double speed_mps) { nominal_speed_mps_ = speed_mps; }
  double nominalSpeed() const { return nominal_speed_mps_; }

  /** \brief Static extra per-edge delay (seconds), e.g., huge_delay from grid. */
  void setStaticEdgeDelays(const std::unordered_map<EdgeId, double> &delays) {
    static_edge_delays_ = delays;
  }

  /** \brief Overwrite the blockage intervals map (newest-overrides semantics handled upstream). */
  void setEdgeBlockages(const std::unordered_map<EdgeId, BlockageInterval> &b) {
    edge_blockages_ = b;
  }

  /** \brief Set a set of edges that are never traversable (topological bans). */
  void setBannedEdges(const std::unordered_set<EdgeId> &banned) {
    banned_edges_ = banned;
  }

  /** \brief Path from -> to using departure time = now_sec() */
  PathType path(const VertexId &from, const VertexId &to) override;

  /** \brief Multi-waypoint path. Each segment departs at the previous segment's arrival time. */
  PathType path(const VertexId &from, const VertexId::List &to,
                std::list<uint64_t> &idx) override;

  /** \brief Supply the wall/ROS time (seconds). */
  void setNowSecCallback(std::function<double()> cb) { now_sec_ = std::move(cb); }

  /** \brief Debug helper: return "now" in seconds as seen by the planner. */
  double nowSec() const { return now_sec_ ? now_sec_() : 0.0; }

  /** \brief Debug helper: compute total ETA (seconds) along a given path starting at depart_sec.
   *
   * This uses the same FIFO arc-arrival model as planning:
   * - wait until blockage end if arriving within [start,end)
   * - then add travel time + static edge delay
   *
   * Outputs are for logging/inspection only.
   */
  double debugPathArrivalTimeSec(const PathType &path, const double depart_sec,
                                 double &wait_out_sec,
                                 double &travel_out_sec,
                                 double &static_out_sec) const;

 private:
  std::pair<PathType, double> tdsp(const VertexId &from, const VertexId &to,
                                  const double depart_sec) const;

  double arcArrival(const GraphBasePtr &graph, const EdgeId &e,
                    const double t_in) const;

  GraphPtr getGraph() const;
  GraphBasePtr getPrivilegedGraph() const;

 private:
  GraphWeakPtr graph_;
  std::function<double()> now_sec_;

  double nominal_speed_mps_ = 0.5;
  std::unordered_map<EdgeId, double> static_edge_delays_;
  std::unordered_map<EdgeId, BlockageInterval> edge_blockages_;
  std::unordered_set<EdgeId> banned_edges_;
};

}  // namespace route_planning
}  // namespace vtr


