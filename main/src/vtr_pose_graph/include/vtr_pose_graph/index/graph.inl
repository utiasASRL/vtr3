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
typename Graph<V, E, R>::Ptr Graph<V, E, R>::MakeShared() {
  return Ptr(new Graph());
}

template <class V, class E, class R>
typename Graph<V, E, R>::Ptr Graph<V, E, R>::MakeShared(const IdType& id) {
  return Ptr(new Graph(id));
}

template <class V, class E, class R>
Graph<V, E, R>::Graph()
    : GraphBase<V, E, R>(),
      currentRun_(nullptr),
      lastRunIdx_(uint32_t(-1)),
      callback_(new IgnoreCallbacks<V, E, R>()) {}

template <class V, class E, class R>
Graph<V, E, R>::Graph(const IdType& id)
    : GraphBase<V, E, R>(id),
      currentRun_(nullptr),
      lastRunIdx_(uint32_t(-1)),
      callback_(new IgnoreCallbacks<V, E, R>()) {}

template <class V, class E, class R>
typename Graph<V, E, R>::RunIdType Graph<V, E, R>::addRun() {
  LockGuard lck(mtx_);

  if (currentRun_ == nullptr || currentRun_->vertices().size() > 0) {
    RunIdType newRunId = ++lastRunIdx_;
    currentRun_ = RunType::MakeShared(newRunId, id_);
    runs_->insert({newRunId, currentRun_});
    callback_->runAdded(currentRun_);
  } else {
    LOG(WARNING) << "[Graph] Added a new run while the current run was empty; "
                    "returning the existing run";
  }

  return currentRun_->id();
}

template <class V, class E, class R>
typename Graph<V, E, R>::VertexPtr Graph<V, E, R>::addVertex() {
  LockGuard lck(mtx_);

  auto vertex = currentRun_->addVertex();
  vertices_->insert({vertex->simpleId(), vertex});
  graph_.addVertex(vertex->simpleId());

  callback_->vertexAdded(vertex);

  return vertex;
}

template <class V, class E, class R>
typename Graph<V, E, R>::VertexPtr Graph<V, E, R>::addVertex(
    const RunIdType& runId) {
  LockGuard lck(mtx_);

  auto vertex = run(runId)->addVertex();
  vertices_->insert({vertex->simpleId(), vertex});
  graph_.addVertex(vertex->simpleId());

  callback_->vertexAdded(vertex);

  return vertex;
}

template <class V, class E, class R>
typename Graph<V, E, R>::EdgePtr Graph<V, E, R>::addEdge(
    const VertexIdType& from, const VertexIdType& to, const EdgeTypeEnum& type,
    bool manual) {
  LockGuard lck(mtx_);

  auto runId = std::max(from.majorId(), to.majorId());

  EdgePtr tmp(run(runId)->addEdge(from, to, type, manual));
  run(from.majorId())->at(from)->addEdge(tmp->id());
  run(to.majorId())->at(to)->addEdge(tmp->id());

  graph_.addEdge(tmp->simpleId());
  edges_->insert({tmp->simpleId(), tmp});

  callback_->edgeAdded(tmp);

  return tmp;
}

template <class V, class E, class R>
typename Graph<V, E, R>::EdgePtr Graph<V, E, R>::addEdge(
    const VertexIdType& from, const VertexIdType& to,
    const TransformType& T_to_from, const EdgeTypeEnum& type, bool manual) {
  LockGuard lck(mtx_);

  auto runId = std::max(from.majorId(), to.majorId());

  EdgePtr tmp(run(runId)->addEdge(from, to, T_to_from, type, manual));
  run(from.majorId())->at(from)->addEdge(tmp->id());
  run(to.majorId())->at(to)->addEdge(tmp->id());

  graph_.addEdge(tmp->simpleId());
  edges_->insert({tmp->simpleId(), tmp});

  //  tmp->setManual(manual);
  //  tmp->setTransform(T_to_from);

  callback_->edgeAdded(tmp);

  return tmp;
}

}  // namespace pose_graph
}  // namespace vtr
