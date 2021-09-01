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
 * \file basic_graph.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#if 0
#define BASIC_GRAPH_NO_EXTERN

#include <vtr_pose_graph/index/graph.hpp>

namespace vtr {
namespace pose_graph {

template class RunBase<VertexBase, EdgeBase>;
template class GraphBase<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;
template class Graph<VertexBase, EdgeBase, RunBase<VertexBase, EdgeBase>>;

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraphBase)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraphBase)

EVAL_TYPED_EXPLICIT_INSTANTIATE(double, BasicGraph)
EVAL_TYPED_EXPLICIT_INSTANTIATE(bool, BasicGraph)

}  // namespace pose_graph
}  // namespace vtr
#endif