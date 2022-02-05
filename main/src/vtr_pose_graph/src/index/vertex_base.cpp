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
 * \file vertex_base.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_pose_graph/index/vertex_base.hpp"

namespace vtr {
namespace pose_graph {

VertexBase::Ptr VertexBase::MakeShared(const VertexId& id) {
  return std::make_shared<VertexBase>(id);
}

VertexBase::VertexBase(const VertexId& id) : id_(id) {}

VertexId VertexBase::id() const { return id_; }

std::ostream& operator<<(std::ostream& out, const VertexBase& v) {
  return out << v.id();
}

}  // namespace pose_graph
}  // namespace vtr
