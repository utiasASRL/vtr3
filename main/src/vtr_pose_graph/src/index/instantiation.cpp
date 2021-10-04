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
 * \file instantiation.cpp
 * \brief
 * \details
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/index/graph.hpp>

namespace vtr {
namespace pose_graph {

template class RunBase<VertexBase, EdgeBase>;
template class GraphBase<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;
template class Graph<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;

#if false
EVAL_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraphBase)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraphBase)

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraph)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraph)
#endif

}  // namespace pose_graph
}  // namespace vtr
