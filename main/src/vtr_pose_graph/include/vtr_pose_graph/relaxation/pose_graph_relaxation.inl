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
 * \file pose_graph_relaxation.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <array>
#include <typeinfo>
#include <vtr_pose_graph/relaxation/pose_graph_relaxation.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
void PoseGraphRelaxation<G>::addCostTerms(const GraphPtr& graph,
                                          const VertexIdType&,
                                          StateMapType& stateMap,
                                          CostTermPtr& costTerms,
                                          const eval::Mask::Ptr& mask) {
  if (!costTerms.get())
    costTerms.reset(new steam::ParallelizedCostTermCollection());

  auto subgraph = graph->getSubgraph(mask);
  for (auto jt = subgraph->beginEdge(), jte = subgraph->endEdge(); jt != jte;
       ++jt) {
    // Don't add cost terms when both things are locked
    if (stateMap[jt->to()]->isLocked() && stateMap[jt->from()]->isLocked())
      continue;

    // Add an edge constraint between the from and to vertices
    steam::TransformErrorEval::Ptr errorFunc(new steam::TransformErrorEval(
        jt->T(), stateMap[jt->to()], stateMap[jt->from()]));

    steam::WeightedLeastSqCostTerm<6, 6>::Ptr cost(
        new steam::WeightedLeastSqCostTerm<6, 6>(
            errorFunc, noiseModels_[jt->idx()](jt->T()), lossFunc_));

    costTerms->add(cost);
  }
}

}  // namespace pose_graph
}  // namespace vtr
