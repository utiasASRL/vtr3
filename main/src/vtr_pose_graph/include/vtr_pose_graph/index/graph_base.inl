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
 * \file graph_base.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/graph_base.hpp"

namespace vtr {
namespace pose_graph {

template <class V, class E>
GraphBase<V, E>::GraphBase(const GraphBase& other, const SimpleGraph& graph)
    : graph_(graph) {
  for (const auto& vid : graph_.node_map_)
    vertices_.emplace(vid.first, other.vertices_.at(vid.first));
  for (const auto& eid : graph_.edges_)
    edges_.emplace(eid, other.edges_.at(eid));
}

template <class V, class E>
GraphBase<V, E>::GraphBase(const GraphBase& other, SimpleGraph&& graph)
    : graph_(graph) {
  for (const auto& vid : graph_.node_map_)
    vertices_.emplace(vid.first, other.vertices_.at(vid.first));
  for (const auto& eid : graph_.edges_)
    edges_.emplace(eid, other.edges_.at(eid));
}

template <class V, class E>
VertexId::Set GraphBase<V, E>::neighbors(const VertexId& v) const {
  std::shared_lock lock(mutex_);

  VertexId::Set neighbours;
  for (const auto& vid : graph_.node_map_.at(v).getAdjacent())
    neighbours.insert(vid);

  return neighbours;
}

template <class V, class E>
auto GraphBase<V, E>::pathDecomposition(
    SimpleGraph::ComponentList& paths, SimpleGraph::ComponentList& cycles) const
    -> SimpleGraph::JunctionSet {
  std::shared_lock lock(mutex_);
  return graph_.pathDecomposition(paths, cycles);
}

}  // namespace pose_graph
}  // namespace vtr
