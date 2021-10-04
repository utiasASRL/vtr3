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
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/evaluator_base/evaluator_base.hpp>
#include <vtr_pose_graph/evaluator_base/evaluator_ops.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

/** \brief boolean interfaces: evaluator base and operators */
EVALUATOR_BASE_DECLARE_EXTERN(bool);
EVALUATOR_BOOLEAN_INTERFACE_HPP(bool, bool);
EVALUATOR_COMPARISON_INTERFACE_HPP(bool);

/** \brief double interfaces: evaluator base and operators */
EVALUATOR_BASE_DECLARE_EXTERN(double);
EVALUATOR_SCALAR_INTERFACE_HPP(double, double);

/** \brief generic evaluator used in graph search: Mask and Weight */
NEW_EVALUATOR_TYPE(Mask, bool);
NEW_EVALUATOR_TYPE(Weight, double);

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
