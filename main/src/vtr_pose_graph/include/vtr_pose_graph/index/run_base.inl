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
 * \file run_base.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/run_base.hpp>

namespace vtr {
namespace pose_graph {

template <class V, class E>
typename RunBase<V, E>::Ptr RunBase<V, E>::MakeShared() {
  return Ptr(new RunBase());
}

template <class V, class E>
typename RunBase<V, E>::Ptr RunBase<V, E>::MakeShared(const IdType& runId,
                                                      const IdType& graphId) {
  return Ptr(new RunBase(runId, graphId));
}

template <class V, class E>
RunBase<V, E>::RunBase()
    : id_(-1),
      graphId_(-1),
      vertices_(VertexPtrMap()),
      currentVertex_(-1),
      edges_(EdgePtrMapArray()),
      currentEdge_(CurrentEdgeArray()),
      manual_(false) {
  for (auto it = currentEdge_.begin(); it != currentEdge_.end(); ++it) *it = -1;
}

template <class V, class E>
RunBase<V, E>::RunBase(const IdType& runId, const IdType& graphId)
    : id_(runId),
      graphId_(graphId),
      vertices_(VertexPtrMap()),
      currentVertex_(-1),
      edges_(EdgePtrMapArray()),
      currentEdge_(CurrentEdgeArray()),
      manual_(false) {
  for (auto it = currentEdge_.begin(); it != currentEdge_.end(); ++it) *it = -1;
}

template <class V, class E>
auto RunBase<V, E>::addVertex(const VertexIdType& v) -> const VertexPtr& {
  if (v.isSet()) {
    if (v.majorId() != id_)
      throw std::invalid_argument(
          "[RunBase::addVertex] Cannot add a vertex with a different run id");

    if (vertices_.find(v) != vertices_.end()) return vertices_.at(v);

    currentVertex_ = std::max(currentVertex_, v.minorId());

    return vertices_.emplace(v.id(), VertexPtr(new VertexType(v.id())))
        .first->second;
  } else {
    ++currentVertex_;

    return vertices_
        .emplace(VertexIdType(id_, currentVertex_),
                 VertexPtr(new VertexType(VertexIdType(id_, currentVertex_))))
        .first->second;
  }
}

template <class V, class E>
auto RunBase<V, E>::addEdge(const VertexIdType& from, const VertexIdType& to,
                            const EdgeEnumType& type_, bool manual)
    -> const EdgePtr& {
  if (from.majorId() < to.majorId())
    throw std::invalid_argument(
        "Spatial edges may only be added from higher run numbers to lower "
        "ones");

  ++currentEdge_[size_t(type_)];
  manual_ |= manual;

  return edges_[size_t(type_)]
      .emplace(
          EdgeIdType(from, to, type_),
          EdgeType::MakeShared(EdgeIdType(from, to, type_), from, to, manual))
      .first->second;
}

template <class V, class E>
auto RunBase<V, E>::addEdge(const VertexIdType& from, const VertexIdType& to,
                            const TransformType& T_to_from,
                            const EdgeEnumType& type_, bool manual)
    -> const EdgePtr& {
  if (from.majorId() < to.majorId())
    throw std::invalid_argument(
        "Spatial edges may only be added from higher run numbers to lower "
        "ones");

  ++currentEdge_[size_t(type_)];
  manual_ |= manual;

  return edges_[size_t(type_)]
      .emplace(EdgeIdType(from, to, type_),
               EdgeType::MakeShared(EdgeIdType(from, to, type_), from, to,
                                    T_to_from, manual))
      .first->second;
}

template <class V, class E>
void RunBase<V, E>::addEdge(const EdgePtr& edge) {
  if (edge->from().majorId() < edge->to().majorId())
    throw std::invalid_argument(
        "Spatial edges may only be added from higher run numbers to lower "
        "ones");

  ++currentEdge_[edge->idx()];
  (void)edges_[edge->idx()]
      .insert(std::make_pair(edge->id(), edge))
      .first->second;

  manual_ |= edge->isManual();
}

template <class V, class E>
void RunBase<V, E>::computeManual() {
  for (auto&& it : edges_[Temporal]) {
    if (it.second->isManual()) {
      manual_ = true;
      break;
    }
  }

  for (auto&& it : edges_[Spatial]) {
    if (it.second->isManual()) {
      manual_ = true;
      break;
    }
  }
}

}  // namespace pose_graph
}  // namespace vtr
