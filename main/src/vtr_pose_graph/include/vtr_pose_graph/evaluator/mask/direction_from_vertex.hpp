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
 * \file common.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator_base/common.hpp>

#include <vtr_pose_graph/serializable/rc_graph.hpp>
#include <vtr_pose_graph/serializable/rc_graph_base.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {
namespace Mask {

template <class GRAPH>
class DirectionFromVertexDirect : public virtual Typed<GRAPH>::Direct {
 public:
  using AbstractBase = typename Typed<GRAPH>::Direct;
  PTR_TYPEDEFS(DirectionFromVertexDirect);

  static Ptr MakeShared(VertexId id, bool reverse = false) {
    return Ptr(new DirectionFromVertexDirect(id, reverse));
  }
  DirectionFromVertexDirect(VertexId id, bool reverse = false)
      : reverse_(reverse), id_(id) {}

  virtual ~DirectionFromVertexDirect() {}

  DirectionFromVertexDirect(const DirectionFromVertexDirect &) = default;
  DirectionFromVertexDirect &operator=(const DirectionFromVertexDirect &) =
      default;
  DirectionFromVertexDirect(DirectionFromVertexDirect &&) = default;
  DirectionFromVertexDirect &operator=(DirectionFromVertexDirect &&) = default;

 protected:
  ReturnType computeEdge(
      const typename Typed<GRAPH>::Direct::EdgePtr &e) const override {
    // Garbage computation to avoid warnings
    (void)&e;
    return true;
  }

  virtual ReturnType computeVertex(
      const typename Typed<GRAPH>::Direct::VertexPtr &v) const override {
    const auto &id = v->id();
    // We can't tell direction if the majors are different
    if (id_.majorId() != id.majorId()) return true;
    // Itself is in the mask
    if (id_.minorId() == id.minorId()) return true;
    // If the query is earlier, and we're headed in reverse, OK.
    if ((id_.minorId() > id.minorId()) == reverse_) return true;
    // Nope!
    return false;
  }

  const bool reverse_;
  const VertexId id_;
};

}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
