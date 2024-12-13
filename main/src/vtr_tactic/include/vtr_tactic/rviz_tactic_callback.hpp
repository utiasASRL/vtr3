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
 * \file rviz_tactic_callback.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "vtr_tactic/tactic_callback_interface.hpp"

namespace vtr {
namespace tactic {

class RvizTacticCallback : virtual public TacticCallbackInterface {
 public:
  using PathMsg = nav_msgs::msg::Path;
  using OdometryMsg = nav_msgs::msg::Odometry;

  RvizTacticCallback(const rclcpp::Node::SharedPtr& node,
                     const std::string& prefix = "tactic");

  void publishOdometryRviz(const Timestamp& stamp,
                           const EdgeTransform& T_r_v_odo,
                           const EdgeTransform& T_w_v_odo,
                           const Eigen::Vector<double, 6>& w_r_in_r) override;
  void publishPathRviz(const LocalizationChain& chain) override;
  void publishLocalizationRviz(const Timestamp& stamp,
                               const EdgeTransform& T_w_v_loc) override;

 private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_bc_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr odometry_pub_;
  rclcpp::Publisher<PathMsg>::SharedPtr loc_path_pub_;
};

}  // namespace tactic
}  // namespace vtr
