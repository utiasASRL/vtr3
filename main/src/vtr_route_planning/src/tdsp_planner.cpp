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
 * \file tdsp_planner.cpp
 * \brief FIFO time-dependent shortest path (label-setting) planner implementation.
 */

#include "vtr_route_planning/tdsp_planner.hpp"

namespace vtr {
namespace route_planning {

double TDSPPlanner::debugPathArrivalTimeSec(const PathType &path,
                                            const double depart_sec,
                                            double &wait_out_sec,
                                            double &travel_out_sec,
                                            double &static_out_sec) const {
  wait_out_sec = 0.0;
  travel_out_sec = 0.0;
  static_out_sec = 0.0;

  if (path.size() < 2) return depart_sec;

  const auto graph = getPrivilegedGraph();
  double t = depart_sec;

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const EdgeId e(path[i], path[i + 1]);
    if (banned_edges_.count(e)) return std::numeric_limits<double>::infinity();

    // Travel time
    double len_m = 0.0;
    try {
      auto edge_ptr = graph->at(e);
      if (edge_ptr) len_m = edge_ptr->T().r_ab_inb().norm();
    } catch (...) {
      return std::numeric_limits<double>::infinity();
    }
    const double inv_speed =
        (nominal_speed_mps_ > 1e-6) ? (1.0 / nominal_speed_mps_) : 1.0;
    const double travel = len_m * inv_speed;

    // Static delay
    double static_delay = 0.0;
    if (auto it = static_edge_delays_.find(e); it != static_edge_delays_.end()) {
      static_delay = it->second;
    }

    // FIFO waiting
    double wait = 0.0;
    if (auto it = edge_blockages_.find(e); it != edge_blockages_.end()) {
      const auto &b = it->second;
      if (t >= b.start_sec && t < b.end_sec) wait = std::max(0.0, b.end_sec - t);
    }

    wait_out_sec += wait;
    travel_out_sec += travel;
    static_out_sec += static_delay;

    t = (t + wait) + travel + static_delay;
  }

  return t;
}

auto TDSPPlanner::path(const VertexId &from, const VertexId &to) -> PathType {
  const double depart_sec = now_sec_ ? now_sec_() : 0.0;
  return tdsp(from, to, depart_sec).first;
}

auto TDSPPlanner::path(const VertexId &from, const VertexId::List &to,
                       std::list<uint64_t> &idx) -> PathType {
  if (to.empty()) {
    throw std::invalid_argument("waypoint list is empty");
  }
  idx.clear();

  PathType rval;
  VertexId curr = from;
  double t_depart = now_sec_ ? now_sec_() : 0.0;

  for (const auto &wp : to) {
    auto [segment, t_arrive] = tdsp(curr, wp, t_depart);
    if (!segment.empty()) {
      if (rval.empty()) {
        rval = segment;
      } else {
        rval.insert(rval.end(), std::next(segment.begin()), segment.end());
      }
    }
    idx.push_back(rval.empty() ? 0 : (rval.size() - 1));
    curr = wp;
    t_depart = t_arrive;  // depart next segment at arrival time
  }
  return rval;
}

auto TDSPPlanner::tdsp(const VertexId &from, const VertexId &to,
                       const double depart_sec) const
    -> std::pair<PathType, double> {
  const auto priv_graph = getPrivilegedGraph();

  // Earliest arrival labels
  std::unordered_map<VertexId, double> ea;
  ea.reserve(1024);
  std::unordered_map<VertexId, VertexId> pred;
  pred.reserve(1024);

  ea[from] = depart_sec;

  struct QItem {
    double t;
    VertexId v;
  };
  struct Cmp {
    bool operator()(const QItem &a, const QItem &b) const { return a.t > b.t; }
  };
  std::priority_queue<QItem, std::vector<QItem>, Cmp> pq;
  pq.push({depart_sec, from});

  while (!pq.empty()) {
    const auto [t_u, u] = pq.top();
    pq.pop();

    auto it_u = ea.find(u);
    if (it_u == ea.end() || t_u > it_u->second) continue;

    if (u == to) break;  // label-setting: destination finalized when popped

    // Relax outgoing arcs (u -> v) using FIFO arc-arrival function.
    for (const auto &v : priv_graph->neighbors(u)) {
      const EdgeId e(u, v);
      const double t_v = arcArrival(priv_graph, e, t_u);
      if (!std::isfinite(t_v)) continue;

      auto it_v = ea.find(v);
      if (it_v == ea.end() || t_v < it_v->second) {
        ea[v] = t_v;
        pred[v] = u;
        pq.push({t_v, v});
      }
    }
  }

  auto it_goal = ea.find(to);
  if (it_goal == ea.end()) {
    throw std::runtime_error("TDSPPlanner: no route under current costs/mask");
  }

  // Reconstruct path by predecessors.
  PathType path;
  VertexId cur = to;
  path.push_back(cur);
  while (cur != from) {
    auto it = pred.find(cur);
    if (it == pred.end()) {
      throw std::runtime_error("TDSPPlanner: predecessor chain broken");
    }
    cur = it->second;
    path.push_back(cur);
  }
  std::reverse(path.begin(), path.end());
  return {path, it_goal->second};
}

double TDSPPlanner::arcArrival(const GraphBasePtr &graph, const EdgeId &e,
                               const double t_in) const {
  if (banned_edges_.count(e)) return std::numeric_limits<double>::infinity();

  // Travel time from geometry (meters -> seconds).
  double len_m = 0.0;
  try {
    auto edge_ptr = graph->at(e);
    if (edge_ptr) len_m = edge_ptr->T().r_ab_inb().norm();
  } catch (...) {
    // If edge lookup fails, treat as non-traversable.
    return std::numeric_limits<double>::infinity();
  }
  const double inv_speed =
      (nominal_speed_mps_ > 1e-6) ? (1.0 / nominal_speed_mps_) : 1.0;
  const double travel = len_m * inv_speed;

  // Static edge delay (seconds), e.g. huge_delay from grid intersections.
  double static_delay = 0.0;
  if (auto it = static_edge_delays_.find(e); it != static_edge_delays_.end()) {
    static_delay = it->second;
  }

  // FIFO blockage waiting: if arrival is within [start,end), wait until end.
  double t_depart = t_in;
  if (auto it = edge_blockages_.find(e); it != edge_blockages_.end()) {
    const auto &b = it->second;
    if (t_in >= b.start_sec && t_in < b.end_sec) t_depart = b.end_sec;
  }

  return t_depart + travel + static_delay;
}

auto TDSPPlanner::getGraph() const -> GraphPtr {
  if (auto g = graph_.lock()) return g;
  throw std::runtime_error("TDSPPlanner: graph expired");
}

auto TDSPPlanner::getPrivilegedGraph() const -> GraphBasePtr {
  const auto graph = getGraph();
  using PrivEval = tactic::PrivilegedEvaluator<tactic::GraphBase>;
  auto priv_eval = std::make_shared<PrivEval>(*graph);
  return graph->getSubgraph(priv_eval);
}

}  // namespace route_planning
}  // namespace vtr


