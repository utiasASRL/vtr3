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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once
#include <vtr_pose_graph/evaluator_base/evaluator_base.hpp>
#include <vtr_pose_graph/evaluator_base/evaluator_ops.hpp>
#include "vtr_pose_graph/serializable/rc_graph.hpp"

#include "vtr_pose_graph/evaluator/weight/angle.hpp"
#include "vtr_pose_graph/evaluator/weight/distance.hpp"
#include "vtr_pose_graph/evaluator/weight/distangle.hpp"

#include "vtr_pose_graph/evaluator/mask/direction_from_vertex.hpp"
#include "vtr_pose_graph/evaluator/mask/privileged.hpp"
#include "vtr_pose_graph/evaluator/mask/spatial.hpp"
#include "vtr_pose_graph/evaluator/mask/temporal.hpp"

#define EVALUATOR_EXPLICIT_DECLARE(EvalType, GraphType) \
  extern template class EvalType##Direct<GraphType>;    \
  extern template class EvalType##Caching<GraphType>;   \
  extern template class EvalType##Windowed<GraphType>;

namespace vtr {
namespace pose_graph {

EVALUATOR_TYPED_DECLARE_EXTERN(double, RCGraph)
EVALUATOR_TYPED_DECLARE_EXTERN(double, RCGraphBase)
EVALUATOR_TYPED_DECLARE_EXTERN(double, BasicGraph)
EVALUATOR_TYPED_DECLARE_EXTERN(double, BasicGraphBase)

EVALUATOR_TYPED_DECLARE_EXTERN(bool, RCGraph)
EVALUATOR_TYPED_DECLARE_EXTERN(bool, RCGraphBase)
EVALUATOR_TYPED_DECLARE_EXTERN(bool, BasicGraph)
EVALUATOR_TYPED_DECLARE_EXTERN(bool, BasicGraphBase)

namespace eval {

namespace Weight {

EVALUATOR_EXPLICIT_DECLARE(Distance, RCGraph)
EVALUATOR_EXPLICIT_DECLARE(Distance, RCGraphBase)
EVALUATOR_EXPLICIT_DECLARE(Distance, BasicGraph)
EVALUATOR_EXPLICIT_DECLARE(Distance, BasicGraphBase)

EVALUATOR_EXPLICIT_DECLARE(Angle, RCGraph)
EVALUATOR_EXPLICIT_DECLARE(Angle, RCGraphBase)
EVALUATOR_EXPLICIT_DECLARE(Angle, BasicGraph)
EVALUATOR_EXPLICIT_DECLARE(Angle, BasicGraphBase)

EVALUATOR_EXPLICIT_DECLARE(DistAngle, RCGraph)
EVALUATOR_EXPLICIT_DECLARE(DistAngle, RCGraphBase)
EVALUATOR_EXPLICIT_DECLARE(DistAngle, BasicGraph)
EVALUATOR_EXPLICIT_DECLARE(DistAngle, BasicGraphBase)
}  // namespace Weight

namespace Mask {

EVALUATOR_EXPLICIT_DECLARE(Privileged, RCGraph)
EVALUATOR_EXPLICIT_DECLARE(Privileged, RCGraphBase)
EVALUATOR_EXPLICIT_DECLARE(Privileged, BasicGraph)
EVALUATOR_EXPLICIT_DECLARE(Privileged, BasicGraphBase)

EVALUATOR_EXPLICIT_DECLARE(Spatial, RCGraph)
EVALUATOR_EXPLICIT_DECLARE(Spatial, RCGraphBase)
EVALUATOR_EXPLICIT_DECLARE(Spatial, BasicGraph)
EVALUATOR_EXPLICIT_DECLARE(Spatial, BasicGraphBase)

EVALUATOR_EXPLICIT_DECLARE(Temporal, RCGraph)
EVALUATOR_EXPLICIT_DECLARE(Temporal, RCGraphBase)
EVALUATOR_EXPLICIT_DECLARE(Temporal, BasicGraph)
EVALUATOR_EXPLICIT_DECLARE(Temporal, BasicGraphBase)

//   DistanceFromVertex doesn't have Caching or Windowed variants
extern template class DirectionFromVertexDirect<RCGraph>;
extern template class DirectionFromVertexDirect<RCGraphBase>;
extern template class DirectionFromVertexDirect<BasicGraph>;
extern template class DirectionFromVertexDirect<BasicGraphBase>;
}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr