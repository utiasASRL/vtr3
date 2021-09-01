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
 * \file planning_interface.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>

namespace vtr {
namespace path_planning {

using vtr::pose_graph::EdgeId;
using vtr::pose_graph::VertexId;
using PathType = vtr::pose_graph::VertexId::Vector;

class PlanningInterface {
 public:
  using EvalPtr = vtr::pose_graph::eval::Weight::Ptr;
  PTR_TYPEDEFS(PlanningInterface)

  virtual PathType path(const VertexId& from, const VertexId& to) = 0;
  virtual PathType path(const VertexId& from, const VertexId::List& to,
                        std::list<uint64_t>* idx) = 0;

  virtual void updatePrivileged() = 0;

  virtual double cost(const VertexId&) = 0;
  virtual double cost(const EdgeId&) = 0;

  virtual EvalPtr cost() = 0;
};

}  // namespace path_planning
}  // namespace vtr
