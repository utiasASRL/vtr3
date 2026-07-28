// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file correction_utils.hpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */

#ifndef VTR_PATH_PLANNING_ERROR_PREDICTORS_CORRECTION_UTILS_HPP
#define VTR_PATH_PLANNING_ERROR_PREDICTORS_CORRECTION_UTILS_HPP

#include <cmath>
#include <lgmath/se3/Transformation.hpp>

namespace vtr {
namespace path_planning {

// Apply lateral-only ef pose correction
inline lgmath::se3::Transformation applyLateralPoseCorrection(
    const lgmath::se3::Transformation& pose, double dx, double dy, double dyaw) {
  Eigen::Matrix4d M = pose.matrix();

  // Pose heading (planar yaw) via atan2(r10, r00).
  const double yaw = std::atan2(M(1, 0), M(0, 0));
  const double c = std::cos(yaw), s = std::sin(yaw);

  // Rotate (dx, dy) into the pose's local frame: (along, lateral).
  const double along  =  c * dx + s * dy;
  const double lateral = -s * dx + c * dy;
  (void)along;  // dropped: longitudinal component is not applied

  // Rotate the lateral-only local vector back into the planning frame.
  const double dx_lat = -s * lateral;
  const double dy_lat =  c * lateral;

  M(0, 3) -= dx_lat;
  M(1, 3) -= dy_lat;

  Eigen::Matrix4d R_dyaw = Eigen::Matrix4d::Identity();
  R_dyaw(0, 0) = std::cos(-dyaw);  R_dyaw(0, 1) = -std::sin(-dyaw);
  R_dyaw(1, 0) = std::sin(-dyaw);  R_dyaw(1, 1) = std::cos(-dyaw);
  M = M * R_dyaw;

  return lgmath::se3::Transformation(M);
}

}  // namespace path_planning
}  // namespace vtr

#endif  // VTR_PATH_PLANNING_ERROR_PREDICTORS_CORRECTION_UTILS_HPP
