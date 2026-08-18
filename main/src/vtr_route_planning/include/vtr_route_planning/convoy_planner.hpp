// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file convoy_planner.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_route_planning/route_planner_interface.hpp"

namespace vtr {
namespace route_planning {

class ConvoyPlanner : public RoutePlannerInterface {
 public:
  PTR_TYPEDEFS(ConvoyPlanner);

  using GraphPtr = tactic::Graph::Ptr;
  using GraphWeakPtr = tactic::Graph::WeakPtr;
  using GraphBasePtr = tactic::GraphBase::Ptr;

  struct ConvoyConfig {
    double convoy_spacing;
    double distance_margin;
  };

  ConvoyPlanner(const GraphPtr &graph, const ConvoyConfig convoy_info) : graph_(graph), convoy_info_(convoy_info) {}

  PathType path(const VertexId &from, const VertexId &to) override;
  PathType path(const VertexId &from, const VertexId::List &to,
                std::list<uint64_t> &idx) override;

 private:
  /** \brief Helper to get a shared pointer to the graph */
  GraphPtr getGraph() const;
  /** \brief Returns a privileged graph (only contains teach routes) */
  GraphBasePtr getPrivilegedGraph();
  /** \brief Computes path from -> to given the privileged graph */
  PathType path(const GraphBasePtr &priv_graph, const VertexId &from,
                const VertexId &to);

  GraphWeakPtr graph_;
  GraphBasePtr priv_graph_cache_;
  const ConvoyConfig convoy_info_;
};

}  // namespace route_planning
}  // namespace vtr