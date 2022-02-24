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
 * \file rviz_tactic_callback.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "vtr_tactic/rviz_tactic_callback.hpp"

namespace vtr {
namespace tactic {

RvizTacticCallback::RvizTacticCallback(const rclcpp::Node::SharedPtr& node,
                                       const std::string& prefix) {
  // clang-format off
  const auto world_offset = node->declare_parameter<std::vector<double>>(prefix + ".rviz_loc_path_offset", std::vector<double>{0., 0., 0.});
  // clang-format on

  tf_static_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  odometry_pub_ = node->create_publisher<OdometryMsg>("odometry", 10);
  loc_path_pub_ = node->create_publisher<PathMsg>("loc_path", 10);

  // world offset for localization path visualization
  Eigen::Vector3d vis_loc_path_offset;
  vis_loc_path_offset << world_offset[0], world_offset[1], world_offset[2];
  Eigen::Affine3d T(Eigen::Translation3d{vis_loc_path_offset});
  auto msg = tf2::eigenToTransform(T);
  msg.header.frame_id = "world";
  msg.child_frame_id = "world (offset)";
  tf_static_bc_->sendTransform(msg);
}

void RvizTacticCallback::publishOdometryRviz(const Timestamp& stamp,
                                             const EdgeTransform& T_r_v_odo,
                                             const EdgeTransform& T_w_v_odo) {
  // publish keyframe
  Eigen::Affine3d T(T_w_v_odo.matrix());
  auto msg = tf2::eigenToTransform(T);
  msg.header.frame_id = "world";
  msg.header.stamp = rclcpp::Time(stamp);
  msg.child_frame_id = "odometry keyframe";
  tf_bc_->sendTransform(msg);

  // publish odometry
  OdometryMsg odometry;
  odometry.header.frame_id = "world";
  odometry.header.stamp = rclcpp::Time(stamp);
  odometry.pose.pose =
      tf2::toMsg(Eigen::Affine3d((T_w_v_odo * T_r_v_odo.inverse()).matrix()));
  odometry_pub_->publish(odometry);

  // publish current frame
  Eigen::Affine3d T2(T_r_v_odo.inverse().matrix());
  auto msg2 = tf2::eigenToTransform(T2);
  msg2.header.frame_id = "odometry keyframe";
  msg2.header.stamp = rclcpp::Time(stamp);
  msg2.child_frame_id = "robot";
  tf_bc_->sendTransform(msg2);
}

void RvizTacticCallback::publishPathRviz(const LocalizationChain& chain) {
  // Publish the repeat path with an offset
  PathMsg path;
  path.header.frame_id = "world (offset)";
  auto& poses = path.poses;
  for (unsigned i = 0; i < chain.size(); ++i) {
    auto& pose = poses.emplace_back();
    pose.pose = tf2::toMsg(Eigen::Affine3d(chain.pose(i).matrix()));
  }
  loc_path_pub_->publish(path);
}

void RvizTacticCallback::publishLocalizationRviz(
    const Timestamp& stamp, const EdgeTransform& T_w_v_loc) {
  /// Publish the current frame localized against in world frame
  Eigen::Affine3d T(T_w_v_loc.matrix());
  auto msg = tf2::eigenToTransform(T);
  msg.header.stamp = rclcpp::Time(stamp);
  msg.header.frame_id = "world";
  msg.child_frame_id = "localization keyframe";
  tf_bc_->sendTransform(msg);

  // apply an offset to separate odometry and localization
  msg.header.frame_id = "world (offset)";
  msg.child_frame_id = "localization keyframe (offset)";
  tf_bc_->sendTransform(msg);
}

}  // namespace tactic
}  // namespace vtr