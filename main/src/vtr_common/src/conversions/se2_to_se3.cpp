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
 * \file se2_to_se3.cpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_common/conversions/se2_to_se3.hpp"

namespace vtr {
namespace common {
namespace conversions {

Eigen::Vector<double, 3> vec3Dto2D(const Eigen::Matrix<double, 6, 1> &se3_vec) {
  Eigen::Vector3d se2_vec;
  se2_vec << se3_vec(0), se3_vec(1), se3_vec(5);
  return se2_vec;
}

Eigen::Vector<double, 6> vec2Dto3D(const Eigen::Matrix<double, 3, 1> &se2_vec) {
  Eigen::Vector<double, 6> se3_vec = Eigen::Vector<double, 6>::Zero();
  se3_vec << se2_vec(0), se2_vec(1), 0, 0, 0, se2_vec(2);
  return se3_vec;
}

Eigen::Matrix<double, 6, 6> cov3Dto2D(const Eigen::Matrix<double, 12, 12> &se3_cov) {
  Eigen::Matrix<double, 6, 6> se2_cov = Eigen::Matrix<double, 6, 6>::Zero();
  // Extract xy block
  se2_cov.block<2, 2>(0, 0) = se3_cov.block<2, 2>(0, 0);
  // Extract yaw, vx, vy block
  se2_cov.block<3,3>(2,2) = se3_cov.block<3,3>(5,5);
  // Extract vyaw element
  se2_cov.block<1,1>(5,5) = se3_cov.block<1,1>(11,11);
  // Extract x, y to yaw, vx, vy cross-correlation blocks
  se2_cov.block<2,3>(0,2) = se3_cov.block<2,3>(0,5);
  se2_cov.block<3,2>(2,0) = se3_cov.block<3,2>(5,0);
  // Extract x, y to vyaw cross-correlation blocks
  se2_cov.block<2,1>(0,5) = se3_cov.block<2,1>(0,11);
  se2_cov.block<1,2>(5,0) = se3_cov.block<1,2>(11,0);
  // Extract yaw, vx, vy to vyaw cross-correlation blocks
  se2_cov.block<3,1>(2,5) = se3_cov.block<3,1>(5,11);
  se2_cov.block<1,3>(5,2) = se3_cov.block<1,3>(11,5);
  // Normalize for safety
  se2_cov = 0.5 * (se2_cov + se2_cov.transpose());
  return se2_cov;
}

Eigen::Matrix<double, 3, 3> cov3Dto2D(const Eigen::Matrix<double, 6, 6> &se3_cov) {
  Eigen::Matrix<double, 3, 3> se2_cov = Eigen::Matrix<double, 3, 3>::Zero();
  // Extract xy block
  se2_cov.block<2, 2>(0, 0) = se3_cov.block<2, 2>(0, 0);
  // Exract yaw element
  se2_cov.block<1,1>(2,2) = se3_cov.block<1,1>(5,5);
  // Extract x, y to yaw cross-correlation blocks
  se2_cov.block<2,1>(0,2) = se3_cov.block<2,1>(0,5);
  se2_cov.block<1,2>(2,0) = se3_cov.block<1,2>(5,0);
  // Normalize for safety
  se2_cov = 0.5 * (se2_cov + se2_cov.transpose());
  return se2_cov;
}

Eigen::Matrix<double, 12, 12> cov2Dto3D(const Eigen::Matrix<double, 6, 6> &se2_cov) {
  // Initialize to small diagonal value to make valid covariance matrix
  Eigen::Matrix<double, 12, 12> se3_cov = 1e-12 * Eigen::Matrix<double, 12, 12>::Identity();
  // Insert xy block
  se3_cov.block<2, 2>(0, 0) = se2_cov.block<2, 2>(0, 0);
  // Insert yaw, vx, vy block
  se3_cov.block<3,3>(5,5) = se2_cov.block<3,3>(2,2);
  // Insert vyaw element
  se3_cov.block<1,1>(11,11) = se2_cov.block<1,1>(5,5);
  // Insert x, y to yaw, vx, vy cross-correlation blocks
  se3_cov.block<2,3>(0,5) = se2_cov.block<2,3>(0,2);
  se3_cov.block<3,2>(5,0) = se2_cov.block<3,2>(2,0);
  // Insert x, y to vyaw cross-correlation blocks
  se3_cov.block<2,1>(0,11) = se2_cov.block<2,1>(0,5);
  se3_cov.block<1,2>(11,0) = se2_cov.block<1,2>(5,0);
  // Insert yaw, vx, vy to vyaw cross-correlation blocks
  se3_cov.block<3,1>(5,11) = se2_cov.block<3,1>(2,5);
  se3_cov.block<1,3>(11,5) = se2_cov.block<1,3>(5,2);
  // Normalize for safety
  se3_cov = 0.5 * (se3_cov + se3_cov.transpose());
  return se3_cov;
}

Eigen::Matrix<double, 6, 6> cov2Dto3D(const Eigen::Matrix<double, 3, 3> &se2_cov) {
  // Initialize to small diagonal value to make valid covariance matrix
  Eigen::Matrix<double, 6, 6> se3_cov = 1e-12 * Eigen::Matrix<double, 6, 6>::Identity();
  // Insert xy block
  se3_cov.block<2, 2>(0, 0) = se2_cov.block<2, 2>(0, 0);
  // Insert yaw element
  se3_cov.block<1,1>(5,5) = se2_cov.block<1,1>(2,2);
  // Insert x, y to yaw cross-correlation blocks
  se3_cov.block<2,1>(0,5) = se2_cov.block<2,1>(0,2);
  se3_cov.block<1,2>(5,0) = se2_cov.block<1,2>(2,0);
  // Normalize for safety
  se3_cov = 0.5 * (se3_cov + se3_cov.transpose());
  return se3_cov;
}
}  // namespace conversions
}  // namespace common
}  // namespace vtr