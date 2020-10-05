#pragma once

#include <array>

#include <vtr_pose_graph/relaxation/pose_graph_relaxation.hpp>

namespace vtr {
namespace pose_graph {
#if 0
/////////////////////////////////////////////////////////////////////////////
/// @brief Build state variables/cost terms and add them to the problem
/////////////////////////////////////////////////////////////////////////////
template <class G>
PoseGraphRelaxation<G>::PoseGraphRelaxation(const Matrix6d& cov,
                                            const LossFuncPtr& lossFunc)
    : noiseModels_({{ModelGen(cov), ModelGen(cov)}}), lossFunc_(lossFunc) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Build state variables/cost terms and add them to the problem
/////////////////////////////////////////////////////////////////////////////
template <class G>
PoseGraphRelaxation<G>::PoseGraphRelaxation(const Matrix6d& covTemporal,
                                            const Matrix6d& covSpatial,
                                            const LossFuncPtr& lossFunc)
    : noiseModels_({{ModelGen(covTemporal), ModelGen(covSpatial)}}),
      lossFunc_(lossFunc) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Build state variables/cost terms and add them to the problem
/////////////////////////////////////////////////////////////////////////////
template <class G>
PoseGraphRelaxation<G>::PoseGraphRelaxation(const NoiseModelPtr& noiseModel,
                                            const LossFuncPtr& lossFunc)
    : noiseModels_({{ModelGen(noiseModel), ModelGen(noiseModel)}}),
      lossFunc_(lossFunc) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Build state variables/cost terms and add them to the problem
/////////////////////////////////////////////////////////////////////////////
template <class G>
PoseGraphRelaxation<G>::PoseGraphRelaxation(
    const NoiseModelPtr& noiseModelTemporal,
    const NoiseModelPtr& noiseModelSpatial, const LossFuncPtr& lossFunc)
    : noiseModels_(
          {{ModelGen(noiseModelTemporal), ModelGen(noiseModelSpatial)}}),
      lossFunc_(lossFunc) {}

/////////////////////////////////////////////////////////////////////////////
/// @brief Build state variables/cost terms and add them to the problem
/////////////////////////////////////////////////////////////////////////////
template <class G>
void PoseGraphRelaxation<G>::addCostTerms(const GraphPtr& graph,
                                          const VertexIdType&,
                                          StateMapType& stateMap,
                                          CostTermPtr& costTerms,
                                          const Eval::Mask::Ptr& mask) {
  if (!costTerms.get()) {
    costTerms.reset(new steam::ParallelizedCostTermCollection());
  }

  auto subgraph = graph->getSubgraph(mask);
  for (auto jt = subgraph->beginEdge(), jte = subgraph->endEdge(); jt != jte;
       ++jt) {
    // Don't add cost terms when both things are locked
    if (stateMap[jt->to()]->isLocked() && stateMap[jt->from()]->isLocked()) {
      continue;
    }

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
#endif
}  // namespace pose_graph
}  // namespace vtr
