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
 * \file bfs_planner.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_route_planning/route_planner_interface.hpp"
#include <unordered_set>

namespace vtr {
namespace route_planning {

class BFSPlanner : public RoutePlannerInterface {
 public:
  PTR_TYPEDEFS(BFSPlanner);

  using GraphPtr = tactic::Graph::Ptr;
  using GraphWeakPtr = tactic::Graph::WeakPtr;
  using GraphBasePtr = tactic::GraphBase::Ptr;

  BFSPlanner(const GraphPtr &graph) : graph_(graph) {}

  PathType path(const VertexId &from, const VertexId &to) override;
  PathType path(const VertexId &from, const VertexId::List &to,
                std::list<uint64_t> &idx) override;
  /** Hshmat: \brief Experimental: compute path while honoring banned edges. */
  PathType hshmat_plan(const VertexId &from, const VertexId &to);

  /** \brief Enable/disable masked Dijkstra (hshmat_plan) via config. */
  void setUseMasked(const bool use) { use_masked_ = use; }
  bool useMasked() const { return use_masked_; }

  /** \brief If true, keep banned edges permanently across replans. */
  void setPermanentBan(const bool perm) { permanent_ban_ = perm; }
  bool permanentBan() const { return permanent_ban_; }

  /** \brief Set or update the list of temporarily banned edges (topology). */
  void setBannedEdges(const std::vector<tactic::EdgeId> &edges) {
    banned_edges_ = {edges.begin(), edges.end()};
  }

  /** \brief Add edges to the banned set (union). */
  void addBannedEdges(const std::vector<tactic::EdgeId> &edges) {
    banned_edges_.insert(edges.begin(), edges.end());
  }

  /** \brief Clear any temporarily banned edges. */
  void clearBannedEdges() { banned_edges_.clear(); }

 private:
  /** \brief Helper to get a shared pointer to the graph */
  GraphPtr getGraph() const;
  /** \brief Returns a privileged graph (only contains teach routes) */
  GraphBasePtr getPrivilegedGraph() const;
  /** \brief Computes path from -> to given the privileged graph */
  PathType path(const GraphBasePtr &priv_graph, const VertexId &from,
                const VertexId &to);

  GraphWeakPtr graph_;
  /** \brief Temporarily banned edges (not traversable). */
  std::unordered_set<tactic::EdgeId> banned_edges_;
  /** \brief Whether to use masked planning (hshmat_plan) on next plan. */
  bool use_masked_ = false;
  /** \brief If true, do not clear bans after replans. */
  bool permanent_ban_ = false;
};

}  // namespace route_planning
}  // namespace vtr