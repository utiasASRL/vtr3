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
 * \file privileged_frame.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/relaxation/privileged_frame.hpp>

namespace vtr {
namespace pose_graph {

template <class GRAPH>
PrivilegedFrame<GRAPH>::PrivilegedFrame(IterType begin, IterType end, bool lazy,
                                        bool cache)
    : iter_(begin), end_(end), useCached_(cache) {
  if (iter_ == end_) {
    LOG(WARNING) << "[PrivilegedFrame] Cannot initialize with empty iterator";
    return;
  }

  if (iter_->v()->hasTransform() && useCached_) {
    tfMap_.emplace(iter_->v()->id(), iter_->v()->T());
  } else {
    tfMap_.emplace(iter_->v()->id(), TransformType(true));
    useCached_ = false;
  }
  ++iter_;

  if (!lazy) computeAll();
}

template <class GRAPH>
PrivilegedFrame<GRAPH>::PrivilegedFrame(IterType begin, IterType end,
                                        TransformType T_root_world, bool lazy,
                                        bool cache)
    : iter_(begin), end_(end), useCached_(cache) {
  if (iter_ == end_) {
    //    throw std::invalid_argument("[PrivilegedFrame] Cannot initialize with
    //    empty iterator");
    LOG(WARNING) << "[PrivilegedFrame] Cannot initialize with empty iterator";
    return;
  }

  tfMap_.emplace(iter_->v()->id(), T_root_world);
  ++iter_;

  if (!lazy) computeAll();
}

template <class GRAPH>
auto PrivilegedFrame<GRAPH>::operator[](const VertexIdType& v)
    -> const TransformType& {
  auto it = tfMap_.find(v);
  if (it != tfMap_.end()) return it->second;

  TransformType T_ab;
  while (iter_ != end_) {
    VertexIdType vid = iter_->v()->id();
    if (tfMap_.find(v) != tfMap_.end()) {
      ++iter_;
      continue;
    }

    if (useCached_ && iter_->v()->hasTransform()) {
      T_ab = iter_->v()->T();
    } else {
      // If we ever got to any transform that doesn't have a cached value, stop
      // using cached values as there will end up being an inconsistency in the
      // result transforms
      useCached_ = false;

      const EdgePtr& e = iter_->e();

      // Check if we traversed the edge "backwards", and invert if necessary
      if (e->from() != iter_->from())
        T_ab = e->T().inverse();
      else
        T_ab = e->T();

      T_ab *= tfMap_.at(iter_->from());
    }

    // T_{vertex}_{root} = T_{vertex}_{from} * T_{from}_{root}
    // We know that the "from" state variable already exists, because we are
    // expanding in search-order
    auto res = tfMap_.emplace(vid, T_ab);
    ++iter_;

    if (vid == v) return res.first->second;
  }

  std::stringstream ss;
  ss << "[PrivilegedFrame] Requested vertex was not in the graph: " << v;
  throw std::runtime_error(ss.str());
}

template <class GRAPH>
void PrivilegedFrame<GRAPH>::computeAll() {
  TransformType T_ab;
  while (iter_ != end_) {
    VertexIdType vid = iter_->v()->id();

    if (useCached_ && iter_->v()->hasTransform()) {
      T_ab = iter_->v()->T();
    } else {
      try {
        // If we ever got to any transform that doesn't have a cached value,
        // stop using cached values as there will end up being an inconsistency
        // in the result transforms
        useCached_ = false;

        const EdgePtr& e = iter_->e();

        // Check if we traversed the edge "backwards", and invert if necessary
        if (e->from() != iter_->from()) {
          T_ab = e->T().inverse();
        } else {
          T_ab = e->T();
        }

        // T_{vertex}_{root} = T_{vertex}_{from} * T_{from}_{root}
        // We know that the "from" state variable already exists, because we are
        // expanding in search-order
        T_ab *= tfMap_.at(iter_->from());
      } catch (...) {
        LOG(ERROR) << "Could not get vertex " << iter_->from();
      }
    }

    (void)tfMap_.emplace(vid, T_ab);

    ++iter_;
  }
}

}  // namespace pose_graph
}  // namespace vtr