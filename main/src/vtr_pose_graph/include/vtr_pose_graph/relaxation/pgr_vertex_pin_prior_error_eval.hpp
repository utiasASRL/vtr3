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
 * \file pgr_vertex_pin_prior_error_eval.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <steam.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief The distance between two points living in their respective frame is
///        used as our error function.
//////////////////////////////////////////////////////////////////////////////////////////////
class PGRVertexPinPriorErrorEval : public ErrorEvaluator<2, 6>::type {
 public:
  /// Convenience typedefs
  using Ptr = boost::shared_ptr<PGRVertexPinPriorErrorEval>;
  using ConstPtr = boost::shared_ptr<const PGRVertexPinPriorErrorEval>;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  /// \param T_rv \todo
  /// \param meas \todo
  //////////////////////////////////////////////////////////////////////////////////////////////
  PGRVertexPinPriorErrorEval(const se3::TransformEvaluator::ConstPtr &T_rv,
                             const Eigen::Vector2d &meas);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state
  ///        variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool isActive() const override;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 2-d measurement error (x, y)
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix<double, 2, 1> evaluate() const override;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 2-d measurement error (x, y) and Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double, 2, 1> evaluate(
      const Eigen::Matrix<double, 2, 2> &lhs,
      std::vector<Jacobian<2, 6>> *jacs) const;

 private:
  se3::TransformEvaluator::ConstPtr T_rv_;

  Eigen::Matrix<double, 2, 4> D_ = Eigen::Matrix<double, 2, 4>::Zero();

  Eigen::Vector4d meas_;
  Eigen::Vector4d origin_;
};

}  // namespace steam
