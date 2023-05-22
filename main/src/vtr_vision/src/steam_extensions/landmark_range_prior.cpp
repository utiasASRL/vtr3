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
 * \file range_conditioning_eval.hpp
 * \brief
 * \details
 *
 * \author Alec Krawciw, Sean Anderson, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/steam_extensions/landmark_range_prior.hpp>

namespace steam {
namespace stereo {

LandmarkRangePrior::Ptr LandmarkRangePrior::MakeShared (
    const Evaluable<LmInType>::ConstPtr landmark) {
        return std::make_shared<LandmarkRangePrior>(landmark);
    }

/// \brief Constructor
LandmarkRangePrior::LandmarkRangePrior(
    const Evaluable<LmInType>::ConstPtr landmark)
    : landmark_(landmark) {
  Eigen::Vector4d h = landmark_->value();
  Eigen::Vector3d v = (1.0 / h[3]) * h.head<3>();
  meas_ << v.norm();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool LandmarkRangePrior::active() const { return landmark_->active(); }

LandmarkRangePrior::OutType LandmarkRangePrior::value() const {
  Eigen::Vector4d h = landmark_->value();
  Eigen::Vector3d v = (1.0 / h[3]) * h.head<3>();
  Eigen::Matrix<double, 1, 1> rhs;
  rhs << v.norm();
  return meas_ - rhs;
}

void LandmarkRangePrior::getRelatedVarKeys(KeySet& keys) const {
    landmark_->getRelatedVarKeys(keys);
}

auto LandmarkRangePrior::forward() const -> Node<OutType>::Ptr {
// Distance between the current z value and the 
  const auto child = landmark_->forward();
  Eigen::Vector4d h = child->value();
  Eigen::Vector3d v = (1.0 / h[3]) * h.head<3>();
  Eigen::Matrix<double, 1, 1> rhs;
  rhs << v.norm();
  const auto value = meas_ - rhs;
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

void LandmarkRangePrior::backward(const Eigen::MatrixXd& lhs, const Node<OutType>::Ptr& node,
                Jacobians& jacs) const {

  if (active()) {
    Eigen::Vector4d h = landmark_->value();
    Eigen::Vector3d v = (1.0 / h[3]) * h.head<3>();
    const auto child = std::static_pointer_cast<Node<LmInType>>(node->at(0));
    // Get Jacobians
    Eigen::MatrixXd new_lhs = -1.0 / (h[3] * v.norm()) * lhs * v.transpose();
    landmark_->backward(new_lhs, child, jacs);
  }
}

} // namespace stereo
} // namespace steam
