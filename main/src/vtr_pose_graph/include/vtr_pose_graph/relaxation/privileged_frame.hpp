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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <map>

#include <vtr_common/utils/macros.hpp>
#include <vtr_pose_graph/index/graph_iterator.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace vtr {
namespace pose_graph {

template <class GRAPH>
class PrivilegedFrame {
 public:
  using VertexPtr = typename GRAPH::VertexPtr;
  using EdgePtr = typename GRAPH::EdgePtr;
  using VertexIdType = typename GRAPH::VertexIdType;
  using TransformType = typename GRAPH::TransformType;
  using IterType = typename GRAPH::OrderedIter;

  PrivilegedFrame(IterType begin, IterType end, bool lazy, bool cache = true);
  PrivilegedFrame(IterType begin, IterType end,
                  TransformType T_root_world = TransformType(true),
                  bool lazy = true, bool cache = true);
  PrivilegedFrame(const PrivilegedFrame&) = default;
  PrivilegedFrame(PrivilegedFrame&&) = default;

  PrivilegedFrame& operator=(const PrivilegedFrame&) = default;
  PrivilegedFrame& operator=(PrivilegedFrame&&) = default;

  /** \brief Get the global transform of a vertex (computed lazily) */
  const TransformType& operator[](const VertexIdType& v);

  /** \brief Get the global transform of a vertex (v must have been computed) */
  const TransformType& at(const VertexIdType& v) const { return tfMap_.at(v); }

  /** \brief Force the computation of all transforms now */
  void computeAll();

 protected:
  IterType iter_;
  IterType end_;

  std::map<VertexIdType, TransformType> tfMap_;

  bool useCached_;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/privileged_frame.inl>

namespace vtr {
namespace pose_graph {
#if 0
#ifndef PRIVILEGED_FRAME_NO_EXTERN
extern template class PrivilegedFrame<BasicGraph>;
extern template class PrivilegedFrame<RCGraph>;
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr
