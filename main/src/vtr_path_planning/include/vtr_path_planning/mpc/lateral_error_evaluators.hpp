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
 * \file lateral_error_evaluators.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include "steam/evaluable/state_var.hpp"

namespace steam {

class LateralErrorEvaluatorLeft : public Evaluable<Eigen::Matrix<double, 1, 1>> {
 public:
  using Ptr = std::shared_ptr<LateralErrorEvaluatorLeft>;
  using ConstPtr = std::shared_ptr<const LateralErrorEvaluatorLeft>;

  using InType = Eigen::Vector4d;
  using OutType = Eigen::Matrix<double, 1, 1>;

  static Ptr MakeShared(const Evaluable<InType>::ConstPtr& pt,
                        const InType& meas_pt);
  LateralErrorEvaluatorLeft(const Evaluable<InType>::ConstPtr& pt,
                          const InType& meas_pt);

  bool active() const override;
  void getRelatedVarKeys(KeySet& keys) const override;

  OutType value() const override;
  Node<OutType>::Ptr forward() const override;
  void backward(const Eigen::MatrixXd& lhs, const Node<OutType>::Ptr& node,
                Jacobians& jacs) const override;

 private:
  /** \brief Transform evaluable */
  const Evaluable<InType>::ConstPtr pt_;
  /** \brief Landmark state variable */
  const InType meas_pt_;
  // constants
  Eigen::Matrix<double, 1, 4> D_ = Eigen::Matrix<double, 1, 4>::Zero();
};

LateralErrorEvaluatorLeft::Ptr homo_point_error_left(
    const Evaluable<LateralErrorEvaluatorLeft::InType>::ConstPtr& pt,
    const LateralErrorEvaluatorLeft::InType& meas_pt);





class LateralErrorEvaluatorRight : public Evaluable<Eigen::Matrix<double, 1, 1>> {
 public:
  using Ptr = std::shared_ptr<LateralErrorEvaluatorRight>;
  using ConstPtr = std::shared_ptr<const LateralErrorEvaluatorRight>;

  using InType = Eigen::Vector4d;
  using OutType = Eigen::Matrix<double, 1, 1>;

  static Ptr MakeShared(const Evaluable<InType>::ConstPtr& pt,
                        const InType& meas_pt);
  LateralErrorEvaluatorRight(const Evaluable<InType>::ConstPtr& pt,
                          const InType& meas_pt);

  bool active() const override;
  void getRelatedVarKeys(KeySet& keys) const override;

  OutType value() const override;
  Node<OutType>::Ptr forward() const override;
  void backward(const Eigen::MatrixXd& lhs, const Node<OutType>::Ptr& node,
                Jacobians& jacs) const override;

 private:
  /** \brief Transform evaluable */
  const Evaluable<InType>::ConstPtr pt_;
  /** \brief Landmark state variable */
  const InType meas_pt_;
  // constants
  Eigen::Matrix<double, 1, 4> D_ = Eigen::Matrix<double, 1, 4>::Zero();
};

LateralErrorEvaluatorRight::Ptr homo_point_error_right(
    const Evaluable<LateralErrorEvaluatorRight::InType>::ConstPtr& pt,
    const LateralErrorEvaluatorRight::InType& meas_pt);


}  // namespace steam
