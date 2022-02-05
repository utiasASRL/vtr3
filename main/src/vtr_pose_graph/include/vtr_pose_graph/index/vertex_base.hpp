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
 * \file vertex_base.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <shared_mutex>

#include "vtr_pose_graph/id/id.hpp"

namespace vtr {
namespace pose_graph {

class VertexBase {
 public:
  PTR_TYPEDEFS(VertexBase);
  CONTAINER_TYPEDEFS(VertexBase);

  static Ptr MakeShared(const VertexId& id);

  explicit VertexBase(const VertexId& id);

  virtual ~VertexBase() = default;

  /** \brief Get the vertex id */
  VertexId id() const;

  /** \brief String output */
  friend std::ostream& operator<<(std::ostream& out, const VertexBase& v);

 protected:
  /** \brief The vertex Id */
  const VertexId id_;

  /** \brief protects non-const class members including */
  mutable std::shared_mutex mutex_;
};

}  // namespace pose_graph
}  // namespace vtr
