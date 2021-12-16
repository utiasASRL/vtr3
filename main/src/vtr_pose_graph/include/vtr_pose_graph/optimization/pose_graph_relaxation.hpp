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
 * \file relaxation_factor.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_pose_graph/optimization/noise_model_generator.hpp"
#include "vtr_pose_graph/optimization/pose_graph_optimizer.hpp"

namespace vtr {
namespace pose_graph {

template <class Graph>
class PoseGraphRelaxation : public PGOFactorInterface<Graph> {
 public:
  PTR_TYPEDEFS(PoseGraphRelaxation);
  using Base = PGOFactorInterface<Graph>;

  // Explicity import things from the dependent scope
  using GraphPtr = typename Base::GraphPtr;
  using VertexId = typename Base::VertexId;
  using Transform = typename Base::Transform;
  using VertexId2TransformMap = typename Base::VertexId2TransformMap;
  using StateMap = typename Base::StateMap;
  using CostTermPtr = typename Base::CostTermPtr;

  using ModelGen = NoiseModelGenerator<Transform, 6>;

  using LossFuncPtr = steam::LossFunctionBase::Ptr;
  using NoiseModelPtr = steam::BaseNoiseModel<6>::Ptr;
  using NoiseModel = steam::StaticNoiseModel<6>;

  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  /** \brief MakeShared forwarding template to catch all constructors */
  template <typename... Args>
  static Ptr MakeShared(Args&&... args) {
    return Ptr(new PoseGraphRelaxation(std::forward<Args>(args)...));
  }

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const Matrix6d& cov = Matrix6d::Identity(6, 6),
      const LossFuncPtr& loss_func = std::make_shared<steam::L2LossFunc>())
      : noise_models_({{ModelGen(cov), ModelGen(cov)}}),
        loss_func_(loss_func) {}

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const Matrix6d& cov_temporal, const Matrix6d& cov_spatial,
      const LossFuncPtr& loss_func = std::make_shared<steam::L2LossFunc>())
      : noise_models_({{ModelGen(cov_temporal), ModelGen(cov_spatial)}}),
        loss_func_(loss_func) {}

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const NoiseModelPtr& noiseModel,
      const LossFuncPtr& loss_func = std::make_shared<steam::L2LossFunc>())
      : noise_models_({{ModelGen(noiseModel), ModelGen(noiseModel)}}),
        loss_func_(loss_func) {}

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const NoiseModelPtr& noiseModelTemporal,
      const NoiseModelPtr& noiseModelSpatial,
      const LossFuncPtr& loss_func = std::make_shared<steam::L2LossFunc>())
      : noise_models_(
            {{ModelGen(noiseModelTemporal), ModelGen(noiseModelSpatial)}}),
        loss_func_(loss_func) {}

  /** \brief Build state variables/cost terms and add them to the problem */
  void addCostTerms(const GraphPtr& graph, StateMap& state_map,
                    const CostTermPtr& cost_terms) override;

 private:
  std::array<ModelGen, 2> noise_models_;

  LossFuncPtr loss_func_;
};

template <class Graph>
void PoseGraphRelaxation<Graph>::addCostTerms(const GraphPtr& graph,
                                              StateMap& state_map,
                                              const CostTermPtr& cost_terms) {
  for (auto jt = graph->beginEdge(), jte = graph->endEdge(); jt != jte; ++jt) {
    // Don't add cost terms when both things are locked
    if (state_map.at(jt->to())->isLocked() &&
        state_map.at(jt->from())->isLocked())
      continue;

    // Add an edge constraint between the from and to vertices
    auto error_func = std::make_shared<steam::TransformErrorEval>(
        jt->T(), state_map.at(jt->to()), state_map.at(jt->from()));

    auto cost = std::make_shared<steam::WeightedLeastSqCostTerm<6, 6>>(
        error_func, noise_models_[jt->idx()](jt->T()), loss_func_);

    cost_terms->add(cost);
  }
}

extern template class PoseGraphRelaxation<BasicGraph>;
extern template class PoseGraphRelaxation<RCGraph>;

}  // namespace pose_graph
}  // namespace vtr
