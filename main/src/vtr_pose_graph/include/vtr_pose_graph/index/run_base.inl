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
typename RunBase<V, E>::Ptr RunBase<V, E>::MakeShared(const IdType& id) {
  return Ptr(new RunBase(id));
}

template <class V, class E>
RunBase<V, E>::RunBase(const IdType& id) : id_(id) {
  current_edge_.fill(-1);
}

template <class V, class E>
template <class... Args>
auto RunBase<V, E>::addVertex(Args&&... args) -> VertexPtr {
  std::unique_lock lock(mutex_);

  ++current_vertex_;
  const auto v = VertexIdType(id_, current_vertex_);

  if (!v.isSet())
    throw std::invalid_argument("Cannot add a vertex with an invalid id");

  if (vertices_.find(v) != vertices_.end()) return vertices_.at(v);

  current_vertex_ = std::max(current_vertex_, v.minorId());

  return vertices_
      .emplace(v.id(), std::make_shared<VertexType>(
                           v.id(), std::forward<Args>(args)...))
      .first->second;
}

template <class V, class E>
template <class... Args>
auto RunBase<V, E>::addVertex(const VertexIdType& v, Args&&... args)
    -> VertexPtr {
  std::unique_lock lock(mutex_);

  if (!v.isSet())
    throw std::invalid_argument("Cannot add a vertex with an invalid id");

  if (v.majorId() != id_)
    throw std::invalid_argument("Cannot add a vertex with a different run id");

  if (vertices_.find(v) != vertices_.end()) return vertices_.at(v);

  current_vertex_ = std::max(current_vertex_, v.minorId());

  return vertices_
      .emplace(v.id(), std::make_shared<VertexType>(
                           v.id(), std::forward<Args>(args)...))
      .first->second;
}

template <class V, class E>
template <class... Args>
auto RunBase<V, E>::addEdge(const VertexIdType& from, const VertexIdType& to,
                            const EdgeEnumType& type, bool manual,
                            Args&&... args) -> EdgePtr {
  std::unique_lock lock(mutex_);

  if (from.majorId() < to.majorId())
    throw std::invalid_argument(
        "Spatial edges may only be added from higher run numbers to lower "
        "ones");

  ++current_edge_[size_t(type)];
  manual_ |= manual;

  return edges_[size_t(type)]
      .emplace(EdgeIdType(from, to, type),
               EdgeType::MakeShared(from, to, type, manual,
                                    std::forward<Args>(args)...))
      .first->second;
}

template <class V, class E>
template <class... Args>
auto RunBase<V, E>::addEdge(const VertexIdType& from, const VertexIdType& to,
                            const EdgeEnumType& type,
                            const TransformType& T_to_from, bool manual,
                            Args&&... args) -> EdgePtr {
  std::unique_lock lock(mutex_);

  if (from.majorId() < to.majorId())
    throw std::invalid_argument(
        "Spatial edges may only be added from higher run numbers to lower "
        "ones");

  ++current_edge_[size_t(type)];
  manual_ |= manual;

  return edges_[size_t(type)]
      .emplace(EdgeIdType(from, to, type),
               EdgeType::MakeShared(from, to, type, T_to_from, manual,
                                    std::forward<Args>(args)...))
      .first->second;
}

template <class V, class E>
void RunBase<V, E>::addEdge(const EdgePtr& edge) {
  std::unique_lock lock(mutex_);

  if (edge->from().majorId() < edge->to().majorId())
    throw std::invalid_argument(
        "Spatial edges may only be added from higher run numbers to lower "
        "ones");

  ++current_edge_[edge->idx()];
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
