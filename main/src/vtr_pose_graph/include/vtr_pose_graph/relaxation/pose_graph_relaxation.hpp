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

#include <vtr_pose_graph/relaxation/graph_optimization_problem.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
class PoseGraphRelaxation : public OptimizationTypeBase<G> {
 public:
  using Base = OptimizationTypeBase<G>;

  // Explicity import things from the dependent scope
  using GraphPtr = typename Base::GraphPtr;
  using VertexIdType = typename Base::VertexIdType;
  using TransformType = typename Base::TransformType;
  using TfMapType = typename Base::TfMapType;
  using StateMapType = typename Base::StateMapType;
  using CostTermPtr = typename Base::CostTermPtr;

  using ModelGen = NoiseModelGenerator<TransformType, 6>;

  using LossFuncPtr = steam::LossFunctionBase::Ptr;
  using NoiseModelPtr = steam::BaseNoiseModel<6>::Ptr;
  using NoiseModel = steam::StaticNoiseModel<6>;

  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  PTR_TYPEDEFS(PoseGraphRelaxation);

  /** \brief MakeShared forwarding template to catch all constructors */
  template <typename... Args>
  static Ptr MakeShared(Args&&... args) {
    return Ptr(new PoseGraphRelaxation(std::forward<Args>(args)...));
  }

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const Matrix6d& cov = Matrix6d::Identity(6, 6),
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()))
      : noiseModels_({{ModelGen(cov), ModelGen(cov)}}), lossFunc_(lossFunc) {}

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const Matrix6d& covTemporal, const Matrix6d& covSpatial,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()))
      : noiseModels_({{ModelGen(covTemporal), ModelGen(covSpatial)}}),
        lossFunc_(lossFunc) {}

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const NoiseModelPtr& noiseModel,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()))
      : noiseModels_({{ModelGen(noiseModel), ModelGen(noiseModel)}}),
        lossFunc_(lossFunc) {}

  /** \brief Build state variables/cost terms and add them to the problem */
  PoseGraphRelaxation(
      const NoiseModelPtr& noiseModelTemporal,
      const NoiseModelPtr& noiseModelSpatial,
      const LossFuncPtr& lossFunc = LossFuncPtr(new steam::L2LossFunc()))
      : noiseModels_(
            {{ModelGen(noiseModelTemporal), ModelGen(noiseModelSpatial)}}),
        lossFunc_(lossFunc) {}

  PoseGraphRelaxation(const PoseGraphRelaxation&) = default;
  PoseGraphRelaxation(PoseGraphRelaxation&&) = default;

  PoseGraphRelaxation& operator=(const PoseGraphRelaxation&) = default;
  PoseGraphRelaxation& operator=(PoseGraphRelaxation&&) = default;

  virtual ~PoseGraphRelaxation() {}

  /** \brief Build state variables/cost terms and add them to the problem */
  virtual void addCostTerms(const GraphPtr& graph, const VertexIdType& root,
                            StateMapType& stateMap, CostTermPtr& costTerms,
                            const eval::Mask::Ptr& mask);

 private:
  std::array<ModelGen, 2> noiseModels_;

  LossFuncPtr lossFunc_;
};

#if 0
#ifndef POSE_GRAPH_RELAXATION_NO_EXTERN
extern template class PoseGraphRelaxation<BasicGraph>;
extern template class PoseGraphRelaxation<RCGraph>;
#endif
#endif
}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/pose_graph_relaxation.inl>
