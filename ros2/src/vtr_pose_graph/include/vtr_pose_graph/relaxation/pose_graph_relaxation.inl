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

    //    auto tmp = cost->cost();
    //    LOG_IF(!std::isfinite(tmp) || std::isnan(tmp), INFO) << jt->id() <<":
    //    " << jt->T().cov(); LOG_IF(!std::isfinite(tmp) || std::isnan(tmp),
    //    INFO) << jt->id() <<": " <<jt->T().matrix() *
    //    stateMap[jt->from()]->getValue().matrix() *
    //    stateMap[jt->to()]->getValue().inverse().matrix();
  }
}

}  // namespace pose_graph
}  // namespace vtr
