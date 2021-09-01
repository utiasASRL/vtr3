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
 * \file pgr_vertex_pin_prior_error_eval.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/relaxation/pgr_vertex_pin_prior_error_eval.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
PGRVertexPinPriorErrorEval::PGRVertexPinPriorErrorEval(
    const se3::TransformEvaluator::ConstPtr &T_rv, const Eigen::Vector2d &meas)
    : T_rv_(T_rv) {
  D_.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
  meas_ << meas(0, 0), meas(1, 0), 0, 1;
  origin_ << 0, 0, 0, 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool PGRVertexPinPriorErrorEval::isActive() const { return T_rv_->isActive(); }

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 2-d measurement error (x, y)
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 2, 1> PGRVertexPinPriorErrorEval::evaluate() const {
  return D_ * (meas_ - T_rv_->evaluate() * origin_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 2-d measurement error (x, y) and Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 2, 1> PGRVertexPinPriorErrorEval::evaluate(
    const Eigen::Matrix<double, 2, 2> &lhs,
    std::vector<Jacobian<2, 6>> *jacs) const {
  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument(
        "Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation tree
  EvalTreeHandle<lgmath::se3::Transformation> blkAutoEvalPosOfTransformDiff =
      T_rv_->getBlockAutomaticEvaluation();

  // Get evaluation from tree
  const lgmath::se3::Transformation T_rv =
      blkAutoEvalPosOfTransformDiff.getValue();
  Eigen::Matrix<double, 2, 1> error =
      D_ * (meas_ - T_rv_->evaluate() * origin_);

  // Get Jacobians
  const Eigen::Matrix<double, 3, 1> Tq = (T_rv * meas_).block<3, 1>(0, 0);
  const Eigen::Matrix<double, 2, 6> new_lhs =
      -lhs * D_ * lgmath::se3::point2fs(Tq);
  T_rv_->appendBlockAutomaticJacobians(
      new_lhs, blkAutoEvalPosOfTransformDiff.getRoot(), jacs);

  // Return evaluation
  return error;
}

}  // namespace steam