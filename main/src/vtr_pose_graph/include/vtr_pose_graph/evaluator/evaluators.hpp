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
 * \file evaluators.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/evaluator_base/evaluator_base.hpp"
#include "vtr_pose_graph/evaluator_base/evaluator_ops.hpp"

#include "vtr_pose_graph/evaluator/weight/distance.hpp"

#include "vtr_pose_graph/evaluator/mask/direction_from_vertex.hpp"
#include "vtr_pose_graph/evaluator/mask/privileged.hpp"
#include "vtr_pose_graph/evaluator/mask/spatial.hpp"
#include "vtr_pose_graph/evaluator/mask/temporal.hpp"
#include "vtr_pose_graph/evaluator/mask/run_select.hpp"
#include "vtr_pose_graph/evaluator/mask/lambda.hpp"