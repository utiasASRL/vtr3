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
 * \file visualization_utils.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include <tuple>
#include <Eigen/Core>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"

#include "vtr_tactic/tactic.hpp"
#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/cbit/generate_pq.hpp"

namespace vtr {
namespace path_planning {

class VisualizationUtils {
public:
    VisualizationUtils();
    VisualizationUtils(rclcpp::Node::SharedPtr node);
    void visualize(
        const tactic::Timestamp& stamp,
        const tactic::EdgeTransform& T_w_p,
        const tactic::EdgeTransform& T_p_r,
        const tactic::EdgeTransform& T_p_r_extp_mpc,
        const std::vector<lgmath::se3::Transformation>& mpc_prediction,
        const std::vector<lgmath::se3::Transformation>& robot_prediction,
        const std::vector<lgmath::se3::Transformation>& tracking_pose_vec,
        const std::vector<lgmath::se3::Transformation>& homotopy_pose_vec,
        const std::shared_ptr<std::vector<Pose>> cbit_path_ptr,
        const std::shared_ptr<CBITCorridor> corridor_ptr);

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corridor_pub_l_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corridor_pub_r_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ref_pose_pub_tracking_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ref_pose_pub_homotopy_;
};

} // namespace path_planning
} // namespace vtr
