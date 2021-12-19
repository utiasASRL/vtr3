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

 private:
  /** \brief Helper to get a shared pointer to the graph */
  GraphPtr getGraph() const;
  /** \brief Returns a privileged graph (only contains teach routes) */
  GraphBasePtr getPrivilegedGraph() const;
  /** \brief Computes path from -> to given the privileged graph */
  PathType path(const GraphBasePtr &priv_graph, const VertexId &from,
                const VertexId &to);

  GraphWeakPtr graph_;
};

}  // namespace route_planning
}  // namespace vtr