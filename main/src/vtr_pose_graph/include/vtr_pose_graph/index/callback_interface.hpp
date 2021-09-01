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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <mutex>
#include <vtr_common/utils/macros.hpp>

namespace vtr {

namespace path_planning {
class PlanningInterface;
}

namespace pose_graph {

template <class V, class E, class R>
class CallbackInterface {
 public:
  using RunPtr = typename R::Ptr;
  using EdgePtr = typename E::Ptr;
  using VertexPtr = typename V::Ptr;
  using MutexPtr = std::shared_ptr<std::mutex>;

  PTR_TYPEDEFS(CallbackInterface)

  virtual void runAdded(const RunPtr&) = 0;
  virtual void vertexAdded(const VertexPtr&) = 0;
  virtual void edgeAdded(const EdgePtr&) = 0;

  virtual void updateRelaxation() = 0;

  virtual void setPlanner(
      const std::shared_ptr<path_planning::PlanningInterface>&){};
};

template <class V, class E, class R>
class IgnoreCallbacks : public virtual CallbackInterface<V, E, R> {
 public:
  using Base = CallbackInterface<V, E, R>;
  using RunPtr = typename Base::RunPtr;
  using EdgePtr = typename Base::EdgePtr;
  using VertexPtr = typename Base::VertexPtr;
  using MutexPtr = std::shared_ptr<std::mutex>;

  PTR_TYPEDEFS(IgnoreCallbacks)

  IgnoreCallbacks() = default;
  IgnoreCallbacks(const IgnoreCallbacks&) = default;
  IgnoreCallbacks(IgnoreCallbacks&&) = default;

  IgnoreCallbacks& operator=(const IgnoreCallbacks&) = default;
  IgnoreCallbacks& operator=(IgnoreCallbacks&&) = default;

  void runAdded(const RunPtr&) override {}
  void vertexAdded(const VertexPtr&) override {}
  void edgeAdded(const EdgePtr&) override {}

  void updateRelaxation() override {}
};

}  // namespace pose_graph
}  // namespace vtr
