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
 * \file graph.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/index/graph.hpp>

namespace vtr {
namespace pose_graph {

template <class V, class E, class R>
typename Graph<V, E, R>::Ptr Graph<V, E, R>::MakeShared(
    const CallbackPtr& callback) {
  return std::make_shared<Graph>(callback);
}

template <class V, class E, class R>
Graph<V, E, R>::Graph(const CallbackPtr& callback)
    : GraphBase<V, E, R>(), callback_(callback) {}

template <class V, class E, class R>
template <class... Args>
typename Graph<V, E, R>::RunIdType Graph<V, E, R>::addRun(Args&&... args) {
  LockGuard lck(mtx_);

  if (current_run_ == nullptr || current_run_->numberOfVertices() > 0) {
    RunIdType new_run_id = ++last_run_id_;
    current_run_ = RunType::MakeShared(new_run_id, std::forward<Args>(args)...);
    runs_->locked().get().insert({new_run_id, current_run_});
    callback_->runAdded(current_run_);
  } else {
    CLOG(WARNING, "pose_graph") << "Adding a new run while the current run was "
                                   "empty; returning the existing run";
  }

  return current_run_->id();
}

template <class V, class E, class R>
template <class... Args>
typename Graph<V, E, R>::VertexPtr Graph<V, E, R>::addVertex(Args&&... args) {
  LockGuard lck(mtx_);

  VertexPtr vertex = nullptr;
  {
    std::unique_lock lock1(simple_graph_mutex_, std::defer_lock);
    std::unique_lock lock2(vertices_->mutex(), std::defer_lock);
    std::lock(lock1, lock2);
    vertex = current_run_->addVertex(std::forward<Args>(args)...);
    vertices_->unlocked().get().insert({vertex->simpleId(), vertex});
    graph_.addVertex(vertex->simpleId());
  }

  callback_->vertexAdded(vertex);

  return vertex;
}

template <class V, class E, class R>
template <class... Args>
typename Graph<V, E, R>::VertexPtr Graph<V, E, R>::addVertex(
    const RunIdType& run_id, Args&&... args) {
  LockGuard lck(mtx_);

  VertexPtr vertex = nullptr;
  {
    std::unique_lock graph_lock(simple_graph_mutex_, std::defer_lock);
    std::unique_lock vertices_lock(vertices_->mutex(), std::defer_lock);
    std::shared_lock runs_lock(runs_->mutex(), std::defer_lock);
    std::lock(graph_lock, vertices_lock, runs_lock);

    auto& runs = runs_->unlocked().get();
    vertex = runs.at(run_id)->addVertex(std::forward<Args>(args)...);
    vertices_->unlocked().get().insert({vertex->simpleId(), vertex});
    graph_.addVertex(vertex->simpleId());
  }

  callback_->vertexAdded(vertex);

  return vertex;
}

template <class V, class E, class R>
template <class... Args>
typename Graph<V, E, R>::EdgePtr Graph<V, E, R>::addEdge(
    const VertexIdType& from, const VertexIdType& to, const EdgeEnumType& type,
    bool manual, Args&&... args) {
  LockGuard lck(mtx_);

  auto run_id = std::max(from.majorId(), to.majorId());
  EdgePtr edge = nullptr;
  {
    std::unique_lock graph_lock(simple_graph_mutex_, std::defer_lock);
    std::unique_lock edges_lock(edges_->mutex(), std::defer_lock);
    std::shared_lock runs_lock(runs_->mutex(), std::defer_lock);
    std::lock(graph_lock, edges_lock, runs_lock);

    /// \note this function assumes the vertices already present
    auto& runs = runs_->unlocked().get();
    edge = runs.at(run_id)->addEdge(from, to, type, manual,
                                    std::forward<Args>(args)...);
    runs.at(from.majorId())->at(from)->addEdge(edge->id());
    runs.at(to.majorId())->at(to)->addEdge(edge->id());

    edges_->unlocked().get().insert({edge->simpleId(), edge});
    graph_.addEdge(edge->simpleId());
  }

  callback_->edgeAdded(edge);

  return edge;
}

template <class V, class E, class R>
template <class... Args>
typename Graph<V, E, R>::EdgePtr Graph<V, E, R>::addEdge(
    const VertexIdType& from, const VertexIdType& to, const EdgeEnumType& type,
    const TransformType& T_to_from, bool manual, Args&&... args) {
  LockGuard lck(mtx_);

  auto run_id = std::max(from.majorId(), to.majorId());
  EdgePtr edge = nullptr;
  {
    std::unique_lock graph_lock(simple_graph_mutex_, std::defer_lock);
    std::unique_lock edges_lock(edges_->mutex(), std::defer_lock);
    std::shared_lock runs_lock(runs_->mutex(), std::defer_lock);
    std::lock(graph_lock, edges_lock, runs_lock);

    /// \note this function assumes the vertices already present
    const auto& runs = runs_->unlocked().get();
    edge = runs.at(run_id)->addEdge(from, to, type, T_to_from, manual,
                                    std::forward<Args>(args)...);
    runs.at(from.majorId())->at(from)->addEdge(edge->id());
    runs.at(to.majorId())->at(to)->addEdge(edge->id());

    edges_->unlocked().get().insert({edge->simpleId(), edge});
    graph_.addEdge(edge->simpleId());
  }

  callback_->edgeAdded(edge);

  return edge;
}

}  // namespace pose_graph
}  // namespace vtr
