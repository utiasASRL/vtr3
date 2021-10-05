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
 * \file evaluators.cpp
 * \brief
 * \details
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/evaluator/evaluators.hpp>

#define EVALUATOR_EXPLICIT_INSTANTIATE(EvalType, GraphType) \
  template class EvalType##Direct<GraphType>;               \
  template class EvalType##Caching<GraphType>;              \
  template class EvalType##Windowed<GraphType>;

namespace vtr {
namespace pose_graph {

EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(double, RCGraph)
EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(double, RCGraphBase)
EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraph)
EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraphBase)

EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(bool, RCGraph)
EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(bool, RCGraphBase)
EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraph)
EVALUATOR_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraphBase)

namespace eval {

namespace Weight {

EVALUATOR_EXPLICIT_INSTANTIATE(Distance, RCGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Distance, RCGraphBase);
EVALUATOR_EXPLICIT_INSTANTIATE(Distance, BasicGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Distance, BasicGraphBase);

EVALUATOR_EXPLICIT_INSTANTIATE(Angle, RCGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Angle, RCGraphBase);
EVALUATOR_EXPLICIT_INSTANTIATE(Angle, BasicGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Angle, BasicGraphBase);

EVALUATOR_EXPLICIT_INSTANTIATE(DistAngle, RCGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(DistAngle, RCGraphBase);
EVALUATOR_EXPLICIT_INSTANTIATE(DistAngle, BasicGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(DistAngle, BasicGraphBase);

}  // namespace Weight

namespace Mask {

EVALUATOR_EXPLICIT_INSTANTIATE(Privileged, RCGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Privileged, RCGraphBase);
EVALUATOR_EXPLICIT_INSTANTIATE(Privileged, BasicGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Privileged, BasicGraphBase);

EVALUATOR_EXPLICIT_INSTANTIATE(Spatial, RCGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Spatial, RCGraphBase);
EVALUATOR_EXPLICIT_INSTANTIATE(Spatial, BasicGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Spatial, BasicGraphBase);

EVALUATOR_EXPLICIT_INSTANTIATE(Temporal, RCGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Temporal, RCGraphBase);
EVALUATOR_EXPLICIT_INSTANTIATE(Temporal, BasicGraph);
EVALUATOR_EXPLICIT_INSTANTIATE(Temporal, BasicGraphBase);

// DistanceFromVertex is a special flower, because it doesn't have Caching or
// Windowed variants
template class DirectionFromVertexDirect<RCGraph>;
template class DirectionFromVertexDirect<RCGraphBase>;
template class DirectionFromVertexDirect<BasicGraph>;
template class DirectionFromVertexDirect<BasicGraphBase>;

}  // namespace Mask

}  // namespace eval
}  // namespace pose_graph
}  // namespace vtr
