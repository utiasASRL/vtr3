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
 * \file se2_to_se3.hpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 * \brief Common SE(2) to SE(3) non-transform conversions
 */
#pragma once

#include "lgmath.hpp"

#include "vtr_common/conversions/tf2_ros_eigen.hpp"
#include "vtr_common_msgs/msg/lie_group_transform.hpp"

namespace vtr {
namespace common {
namespace conversions {

// clang-format off
// Vector conversions
Eigen::Vector<double, 3> vec3Dto2D(const Eigen::Matrix<double, 6, 1> &se3_vec);
Eigen::Vector<double, 6> vec2Dto3D(const Eigen::Matrix<double, 3, 1> &se2_vec);
// Covariance matrix conversions
// Definitions overloaded to support pose-only and pose and velocity covariances
Eigen::Matrix<double, 6, 6> cov3Dto2D(const Eigen::Matrix<double, 12, 12> &se3_cov);
Eigen::Matrix<double, 3, 3> cov3Dto2D(const Eigen::Matrix<double, 6, 6> &se3_cov);
Eigen::Matrix<double, 12, 12> cov2Dto3D(const Eigen::Matrix<double, 6, 6> &se2_cov);
Eigen::Matrix<double, 6, 6> cov2Dto3D(const Eigen::Matrix<double, 3, 3> &se2_cov);
// clang-format on

}  // namespace conversions
}  // namespace common
}  // namespace vtr