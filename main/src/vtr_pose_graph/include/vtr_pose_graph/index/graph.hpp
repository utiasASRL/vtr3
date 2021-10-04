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
 * \brief
 * \details Graph defines all graph modification operations
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/callback_interface.hpp>
#include <vtr_pose_graph/index/graph_base.hpp>

namespace vtr {
namespace pose_graph {

template <class V, class E, class R>
class Graph : public virtual GraphBase<V, E, R> {
 public:
  using Base = GraphBase<V, E, R>;
  using RType = GraphBase<V, E, R>;

  using Base::edges_;
  using Base::graph_;
  using Base::runs_;
  using Base::simple_graph_mutex_;
  using Base::vertices_;

  using VertexType = typename Base::VertexType;
  using VertexPtr = typename Base::VertexPtr;
  using VertexIdType = typename Base::VertexIdType;
  using SimpleVertexId = typename Base::SimpleVertexId;

  using EdgeType = typename Base::EdgeType;
  using EdgePtr = typename Base::EdgePtr;
  using EdgeIdType = typename Base::EdgeIdType;
  using EdgeEnumType = typename Base::EdgeEnumType;
  using SimpleEdgeId = typename Base::SimpleEdgeId;
  using TransformType = typename EdgeType::TransformType;

  using RunType = typename Base::RunType;
  using RunPtr = typename Base::RunPtr;
  using RunIdType = typename Base::RunIdType;

  using RunMap = typename Base::RunMap;
  using VertexMap = typename Base::VertexMap;
  using EdgeMap = typename Base::EdgeMap;

  using CallbackPtr = typename CallbackInterface<V, E, R>::Ptr;

  using MutexType = std::recursive_mutex;
  using UniqueLock = std::unique_lock<MutexType>;
  using LockGuard = std::lock_guard<MutexType>;

  PTR_TYPEDEFS(Graph)

  /** \brief Pseudo-constructor to make shared pointers */
  static Ptr MakeShared();

  Graph();

  Graph(const Graph&) = delete;
  Graph(Graph&& other) = delete;
  Graph& operator=(const Graph&) = delete;
  Graph& operator=(Graph&& other) = delete;

  /** \brief Set the callback handling procedure */
  void setCallbackMode(const CallbackPtr& callback) { callback_ = callback; }
#if false
  /** \brief Get a pointer to the callback manager */
  const CallbackPtr& callbacks() const { return callback_; }
#endif
  /** \brief Add a new run an increment the run id */
  template <class... Args>
  RunIdType addRun(Args&&... args);

  /** \brief Return a blank vertex (current run) with the next available Id */
  template <class... Args>
  VertexPtr addVertex(Args&&... args);

  /** \brief Return a blank vertex with the next available Id */
  template <class... Args>
  VertexPtr addVertex(const RunIdType& runId, Args&&... args);

  /** \brief Return a blank edge with the next available Id */
  template <class... Args>
  EdgePtr addEdge(const VertexIdType& from, const VertexIdType& to,
                  const EdgeEnumType& type, bool manual = false,
                  Args&&... args);

  /** \brief Return a blank edge with the next available Id */
  template <class... Args>
  EdgePtr addEdge(const VertexIdType& from, const VertexIdType& to,
                  const EdgeEnumType& type, const TransformType& T_to_from,
                  bool manual = false, Args&&... args);

  /** \brief Acquires a lock object that blocks modifications. */
  UniqueLock guard() const { return UniqueLock(mtx_); }
  /** \brief Manually locks the graph, preventing modifications. */
  void lock() const { mtx_.lock(); }
  /** \brief Manually unlocks the graph, allowing modifications. */
  void unlock() const { mtx_.unlock(); }
  /** \brief Get a reference to the mutex */
  MutexType& mutex() const { return mtx_; }

 protected:
  /** \brief The current run */
  RunPtr current_run_ = nullptr;

  /** \brief The current maximum run index */
  RunIdType last_run_id_ = uint32_t(-1);

  /** \brief The current maximum run index */
  CallbackPtr callback_ = std::make_shared<IgnoreCallbacks<V, E, R>>();

  /** \brief protects: current_run_, last_run_id_ and callback calls */
  mutable MutexType mtx_;
};

extern template class Graph<VertexBase, EdgeBase,
                            RunBase<VertexBase, EdgeBase>>;
using BasicGraph = Graph<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/index/graph.inl>