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
 * \file privileged_frame.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <map>

#include "vtr_common/utils/macros.hpp"
#include "vtr_pose_graph/index/graph_iterator.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"

namespace vtr {
namespace pose_graph {

template <class Graph>
void updatePrivilegedFrame(
    const typename Graph::Ptr& graph, const typename Graph::VertexIdType& root,
    std::unordered_map<typename Graph::VertexIdType,
                       typename Graph::TransformType>& vid2tf_map) {
  using Transform = typename Graph::TransformType;
  // initial transform defaults to identity
  auto iter = graph->begin(root);
  vid2tf_map.try_emplace(iter->v()->id(), Transform(true));
  ++iter;

  for (; iter != graph->end(); ++iter) {
    const auto curr_vid = iter->v()->id();
    if (vid2tf_map.find(curr_vid) != vid2tf_map.end()) continue;
    //
    const auto e = iter->e();
    // Check if we traversed the edge "backwards", and invert if necessary
    const auto T_curr_prev =
        (e->from() == iter->from()) ? e->T() : e->T().inverse();

    const auto T_curr_priv = T_curr_prev * vid2tf_map.at(iter->from());
    vid2tf_map.emplace(curr_vid, T_curr_priv);
  }
}

template <class Graph>
class PrivilegedFrame {
 public:
  using VertexPtr = typename Graph::VertexPtr;
  using EdgePtr = typename Graph::EdgePtr;
  using VertexId = typename Graph::VertexIdType;
  using Transform = typename Graph::TransformType;
  using Iterator = typename Graph::OrderedIter;

  PrivilegedFrame(const Iterator& begin, const Iterator& end,
                  const Transform& T_root_world = Transform(true),
                  const bool lazy = false);

  /** \brief Get the global transform of a vertex (computed lazily) */
  const Transform& operator[](const VertexId& v);

  /** \brief Get the global transform of a vertex (v must have been computed) */
  const Transform& at(const VertexId& v) const { return tf_map_.at(v); }

 private:
  /** \brief Force the computation of all transforms now */
  void computeAll();

  Iterator iter_;
  Iterator end_;

  std::unordered_map<VertexId, Transform> tf_map_;
};

template <class Graph>
PrivilegedFrame<Graph>::PrivilegedFrame(const Iterator& begin,
                                        const Iterator& end,
                                        const Transform& T_root_world,
                                        const bool lazy)
    : iter_(begin), end_(end) {
  if (iter_ == end_) return;

  tf_map_.emplace(iter_->v()->id(), T_root_world);
  ++iter_;

  if (!lazy) computeAll();
}

template <class Graph>
auto PrivilegedFrame<Graph>::operator[](const VertexId& vid)
    -> const Transform& {
  auto it = tf_map_.find(vid);
  if (it != tf_map_.end()) return it->second;

  Transform T_curr_root;
  while (iter_ != end_) {
    const auto curr_vid = iter_->v()->id();
    if (tf_map_.find(curr_vid) != tf_map_.end()) {
      ++iter_;
      continue;
    }

    const auto e = iter_->e();
    // Check if we traversed the edge "backwards", and invert if necessary
    if (e->from() != iter_->from())
      T_curr_root = e->T().inverse();
    else
      T_curr_root = e->T();

    // T_{vertex}_{root} = T_{vertex}_{from} * T_{from}_{root}
    // We know that the "from" state variable already exists, because we are
    // expanding in search-order
    T_curr_root *= tf_map_.at(iter_->from());
    auto res = tf_map_.emplace(curr_vid, T_curr_root);

    ++iter_;
    if (curr_vid == vid) return res.first->second;
  }

  std::stringstream ss;
  ss << "Requested vertex was not in the graph: " << vid;
  CLOG(ERROR, "pose_graph") << ss.str();
  throw std::runtime_error(ss.str());
}

template <class Graph>
void PrivilegedFrame<Graph>::computeAll() {
  Transform T_curr_root;
  while (iter_ != end_) {
    const auto curr_vid = iter_->v()->id();

    const auto e = iter_->e();
    // Check if we traversed the edge "backwards", and invert if necessary
    if (e->from() != iter_->from())
      T_curr_root = e->T().inverse();
    else
      T_curr_root = e->T();

    // T_{vertex}_{root} = T_{vertex}_{from} * T_{from}_{root}
    // We know that the "from" state variable already exists, because we are
    // expanding in search-order
    T_curr_root *= tf_map_.at(iter_->from());
    tf_map_.emplace(curr_vid, T_curr_root);

    ++iter_;
  }
}

extern template class PrivilegedFrame<BasicGraph>;
extern template class PrivilegedFrame<RCGraph>;

}  // namespace pose_graph
}  // namespace vtr