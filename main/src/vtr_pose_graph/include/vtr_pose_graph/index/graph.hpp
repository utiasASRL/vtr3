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
 * \file graph.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/callback_interface.hpp"
#include "vtr_pose_graph/index/graph_base.hpp"

namespace vtr {
namespace pose_graph {

template <class V, class E>
class Graph : public GraphBase<V, E> {
 public:
  PTR_TYPEDEFS(Graph);

  using GraphType = Graph<V, E>;
  using Base = GraphBase<V, E>;

  using Vertex = typename Base::Vertex;
  using VertexPtr = typename Base::VertexPtr;

  using Edge = typename Base::Edge;
  using EdgePtr = typename Base::EdgePtr;

  using VertexMap = typename Base::VertexMap;
  using EdgeMap = typename Base::EdgeMap;

  using Callback = GraphCallbackInterface<V, E>;
  using CallbackPtr = typename Callback::Ptr;

  using ChangeMutex = std::recursive_mutex;
  using ChangeLock = std::unique_lock<ChangeMutex>;
  using ChangeGuard = std::lock_guard<ChangeMutex>;

  /** \brief Pseudo-constructor to make shared pointers */
  static Ptr MakeShared(
      const CallbackPtr& callback = std::make_shared<Callback>());

  Graph(const CallbackPtr& callback = std::make_shared<Callback>());

  /** \brief Add a new run an increment the run id */
  BaseIdType addRun();

  /** \brief Return a blank vertex (current run) with the next available Id */
  template <class... Args>
  VertexPtr addVertex(Args&&... args);

  /** \brief Return a blank edge with the next available Id */
  template <class... Args>
  EdgePtr addEdge(const VertexId& from, const VertexId& to,
                  const EdgeType& type, const bool manual,
                  const EdgeTransform& T_to_from, Args&&... args);

  /** \brief Lock to prevent graph change */
  ChangeLock guard() const { return ChangeLock(change_mutex_); }

 protected:
  /** \brief protects access to all members below include callback methods */
  using Base::mutex_;

  using Base::graph_;

  using Base::vertices_;

  using Base::edges_;

  /** \brief The current maximum run index */
  BaseIdType curr_major_id_ = InvalidBaseId;
  BaseIdType curr_minor_id_ = InvalidBaseId;

  /** \brief The current maximum run index */
  const CallbackPtr callback_;

  /**
   * \brief Lock by methods that change graph structure, can be used externally
   * to prevent graph structure changes during multiple calls to graph methods.
   * \note Call to individual method is already safe with the use of
   * shared_mutex, this is only for calls to multiple methods sequentially
   * without potential graph change by another thread, and this is not a shared
   * mutex.
   */
  mutable ChangeMutex change_mutex_;
};

extern template class Graph<VertexBase, EdgeBase>;
using BasicGraph = Graph<VertexBase, EdgeBase>;

}  // namespace pose_graph
}  // namespace vtr

#include "vtr_pose_graph/index/graph.inl"