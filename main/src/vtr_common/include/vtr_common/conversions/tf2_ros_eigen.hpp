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
 * \file tf2_ros_eigen.hpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "lgmath.hpp"

namespace vtr {
namespace common {
namespace conversions {

// clang-format off
Eigen::Matrix4d fromPoseMessage(const geometry_msgs::msg::Pose &pose);
geometry_msgs::msg::Pose toPoseMessage(const Eigen::Matrix4d &T_base_pose);

Eigen::Matrix3d fromPoseMessage(const geometry_msgs::msg::Pose2D &pose);

geometry_msgs::msg::Vector3 toVector3Message(const Eigen::Vector3d &v);
geometry_msgs::msg::Point toPointMessage(const Eigen::Vector3d &v);
geometry_msgs::msg::Quaternion toQuaternionMessage(const Eigen::Quaterniond &v);
geometry_msgs::msg::Quaternion toQuaternionMessage(const Eigen::Vector4d &v);
geometry_msgs::msg::Transform toTransformMessage(const Eigen::Matrix4d &T_base_child);

Eigen::Matrix4d fromStampedTransformation(tf2::Stamped<tf2::Transform> const &t_base_child);
Eigen::Matrix4d fromStampedTransformMessage(geometry_msgs::msg::TransformStamped const &t_base_child);

geometry_msgs::msg::Vector3 quat2rpy(const tf2::Quaternion &q);
void getTfPoint(const geometry_msgs::msg::Pose_<std::allocator<void> > &pose, tf2::Vector3 &point);
void getTfQuaternion(const geometry_msgs::msg::Pose_<std::allocator<void> > &pose, tf2::Quaternion &q);
// clang-format on

}  // namespace conversions

}  // namespace common
}  // namespace vtr
