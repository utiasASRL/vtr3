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
 * \file pgr_vertex_pin_prior.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/relaxation/graph_optimization_problem.hpp>
#include <vtr_pose_graph/relaxation/pgr_vertex_pin_prior_error_eval.hpp>

namespace vtr {
namespace pose_graph {

template <class G>
class PGRVertexPinPrior : public OptimizationTypeBase<G> {
 public:
  using Base = OptimizationTypeBase<G>;

  // Explicity import things from the dependent scope
  using GraphPtr = typename Base::GraphPtr;
  using VertexIdType = typename Base::VertexIdType;
  using TransformType = typename Base::TransformType;
  using TfMapType = typename Base::TfMapType;
  using StateMapType = typename Base::StateMapType;
  using CostTermPtr = typename Base::CostTermPtr;

  using ModelGen = NoiseModelGenerator<Eigen::Vector2d, 2>;

  using LossFuncPtr = steam::LossFunctionBase::Ptr;
  using NoiseModelPtr = steam::BaseNoiseModel<2>::Ptr;
  using NoiseModel = steam::StaticNoiseModel<2>;

  PTR_TYPEDEFS(PGRVertexPinPrior);

  /** \brief MakeShared forwarding template to catch all constructors */
  template <typename... Args>
  static Ptr MakeShared(Args&&... args) {
    return Ptr(new PGRVertexPinPrior(std::forward<Args>(args)...));
  }

  /** \brief Build state variables/cost terms and add them to the problem */
  PGRVertexPinPrior(
      VertexId id, Eigen::Vector2d meas, double weight,
      const LossFuncPtr& loss_func = LossFuncPtr(new steam::L2LossFunc()))
      : id_(id),
        meas_(meas),
        noise_model_(
            ModelGen(Eigen::Matrix2d(Eigen::Matrix2d::Identity() / weight))),
        loss_func_(loss_func) {}

  virtual ~PGRVertexPinPrior() {}

  /** \brief Build state variables/cost terms and add them to the problem */
  void addCostTerms(const GraphPtr& graph, const VertexIdType& root,
                    StateMapType& stateMap, CostTermPtr& costTerms,
                    const eval::Mask::Ptr& mask) override {
    if (!costTerms.get())
      costTerms.reset(new steam::ParallelizedCostTermCollection());

    // Don't add cost terms when both things are locked
    if (stateMap.count(id_) == 0) {
      LOG(WARNING) << "Cannot find the required vertex id: " << id_
                   << " in state map.";
    }

    if (stateMap.at(id_)->isLocked()) return;

    // create an evaluator inverse
    steam::se3::TransformStateEvaluator::Ptr temp =
        steam::se3::TransformStateEvaluator::MakeShared(stateMap.at(id_));
    steam::se3::InverseTransformEvaluator::Ptr T_rv =
        steam::se3::InverseTransformEvaluator::MakeShared(temp);

    // Add an edge constraint between the from and to vertices
    steam::PGRVertexPinPriorErrorEval::Ptr error_func(
        new steam::PGRVertexPinPriorErrorEval(T_rv, meas_));

    steam::WeightedLeastSqCostTerm<2, 6>::Ptr cost(
        new steam::WeightedLeastSqCostTerm<2, 6>(
            error_func, noise_model_(meas_), loss_func_));

    costTerms->add(cost);
  }

 private:
  VertexId id_;

  Eigen::Vector2d meas_;

  ModelGen noise_model_;

  LossFuncPtr loss_func_;
};

}  // namespace pose_graph
}  // namespace vtr

#include <vtr_pose_graph/relaxation/pose_graph_relaxation.inl>
