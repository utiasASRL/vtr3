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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/index/graph.hpp"

namespace vtr {
namespace pose_graph {

template <class V, class E>
typename Graph<V, E>::Ptr Graph<V, E>::MakeShared(const CallbackPtr& callback) {
  return std::make_shared<Graph>(callback);
}

template <class V, class E>
Graph<V, E>::Graph(const CallbackPtr& callback) : callback_(callback) {}

template <class V, class E>
BaseIdType Graph<V, E>::addRun() {
  ChangeGuard change_guard(change_mutex_);
  std::unique_lock lock(mutex_);

  if ((curr_major_id_ == InvalidBaseId) || (curr_minor_id_ != InvalidBaseId)) {
    ++curr_major_id_;
    curr_minor_id_ = InvalidBaseId;
  } else {
    CLOG(WARNING, "pose_graph")
        << "No vertex added since last run, not incrementing run id";
  }

  CLOG(DEBUG, "pose_graph") << "Added run " << curr_major_id_;

  return curr_major_id_;
}

template <class V, class E>
template <class... Args>
auto Graph<V, E>::addVertex(Args&&... args) -> VertexPtr {
  ChangeGuard change_guard(change_mutex_);
  std::unique_lock lock(mutex_);

  if (curr_major_id_ == InvalidBaseId) {
    CLOG(ERROR, "pose_graph") << "No run added";
    throw std::runtime_error("No run added");
  }

  VertexId vid(curr_major_id_, ++curr_minor_id_);
  graph_.addVertex(vid);
  auto vertex = Vertex::MakeShared(vid, std::forward<Args>(args)...);
  vertices_.insert({vid, vertex});

  CLOG(DEBUG, "pose_graph") << "Added vertex " << vid;
  callback_->vertexAdded(vertex);

  return vertex;
}

template <class V, class E>
template <class... Args>
auto Graph<V, E>::addEdge(const VertexId& from, const VertexId& to,
                          const EdgeType& type, const bool manual,
                          const EdgeTransform& T_to_from, Args&&... args)
    -> EdgePtr {
  ChangeGuard change_guard(change_mutex_);
  std::unique_lock lock(mutex_);

  if ((vertices_.find(from) == vertices_.end()) ||
      (vertices_.find(to) == vertices_.end())) {
    CLOG(ERROR, "pose_graph") << "Adding edge between non-existent vertices";
    throw std::range_error("Adding edge between non-existent vertices");
  }

  if (from.majorId() < to.majorId()) {
    CLOG(ERROR, "pose_graph")
        << "Cannot add edge from " << from << " to " << to
        << " since the major id of the from vertex is smaller than the to "
           "vertex";
    throw std::invalid_argument(
        "Spatial edges may only be added from higher run numbers to lower "
        "ones");
  }

  if (from.majorId() == to.majorId() && from.minorId() > to.minorId()) {
    CLOG(ERROR, "pose_graph")
        << "Cannot create edge from " << from << " to " << to
        << " since the minor id of the from vertex is greater than the to "
           "vertex when they have the same major id";
    throw std::invalid_argument(
        "Temporal edges may only be added from lower id to higher id");
  }

  EdgeId eid(from, to);
  graph_.addEdge(eid);
  auto edge = Edge::MakeShared(from, to, type, manual, T_to_from,
                               std::forward<Args>(args)...);
  edges_.insert({eid, edge});

  CLOG(DEBUG, "pose_graph") << "Added edge " << eid;
  callback_->edgeAdded(edge);

  return edge;
}

}  // namespace pose_graph
}  // namespace vtr
