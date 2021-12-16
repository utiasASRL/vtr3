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
 * \file callback_interface.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_common/utils/macros.hpp"

namespace vtr {
namespace pose_graph {

template <class V, class E, class R>
class GraphCallbackInterface {
 public:
  PTR_TYPEDEFS(GraphCallbackInterface);

  using RunPtr = typename R::Ptr;
  using EdgePtr = typename E::Ptr;
  using VertexPtr = typename V::Ptr;

  virtual void runAdded(const RunPtr&) {}
  virtual void vertexAdded(const VertexPtr&) {}
  virtual void edgeAdded(const EdgePtr&) {}
};

}  // namespace pose_graph
}  // namespace vtr
