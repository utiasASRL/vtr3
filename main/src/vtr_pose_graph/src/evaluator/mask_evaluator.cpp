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
 * \file mask_evaluator.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#if 0
#define Mask_EVAL_NO_EXTERN
#endif
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>

namespace vtr {
namespace pose_graph {
namespace eval {

EVALUATOR_BOOLEAN_INTERFACE_CPP(bool, bool);
EVALUATOR_COMPARISON_INTERFACE_CPP(bool);

EVAL_BASE_EXPLICIT_INSTANTIATE(bool);

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr