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
 * \file direction_from_vertex.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/evaluator_base/types.hpp"

namespace vtr {
namespace pose_graph {
namespace eval {
namespace mask {
namespace direction_from_vertex {

class Eval : public BaseEval {
 public:
  PTR_TYPEDEFS(Eval);

  Eval(const VertexId &id, const bool reverse = false)
      : id_(id), reverse_(reverse) {}

 protected:
  ReturnType computeEdge(const EdgeId &) override { return true; }

  ReturnType computeVertex(const VertexId &id) override {
    // We can't tell direction if the majors are different
    if (id_.majorId() != id.majorId()) return true;
    // Itself is in the mask
    if (id_.minorId() == id.minorId()) return true;
    // If the query is earlier, and we're headed in reverse, OK.
    if ((id_.minorId() > id.minorId()) == reverse_) return true;
    // Nope!
    return false;
  }

 private:
  const VertexId id_;
  const bool reverse_;
};

}  // namespace direction_from_vertex
}  // namespace mask
}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr