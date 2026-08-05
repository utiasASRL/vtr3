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

#include <array>
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
#include <vtr_path_planning_msgs/msg/path_info_for_external_navigation.hpp>
#include <vtr_path_planning_msgs/msg/predicted_tracking_errors.hpp>
#include <vtr_path_planning_msgs/msg/nn_inputs.hpp>
#include <vtr_path_planning_msgs/msg/nn_true_quantities.hpp>
#include <vtr_path_planning_msgs/msg/extrapolated_robot_pose.hpp>
#include "std_msgs/msg/header.hpp"

#include "vtr_tactic/tactic.hpp"
#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/cbit/generate_pq.hpp"
#include "vtr_path_planning/error_predictors/base_error_predictor.hpp"

namespace vtr {
namespace path_planning {

class VisualizationUtils {
public:
    PTR_TYPEDEFS(VisualizationUtils);

    VisualizationUtils(rclcpp::Node::SharedPtr node);
    void visualize(
        const tactic::Timestamp& stamp,
        const tactic::EdgeTransform& T_w_p,
        const tactic::EdgeTransform& T_p_r,
        const tactic::EdgeTransform& T_p_r_extp_mpc,
        const tactic::EdgeTransform& T_w_r,
        const std::vector<lgmath::se3::Transformation>& mpc_prediction,
        const std::vector<Eigen::Vector2d>& mpc_velocities,
        const std::vector<lgmath::se3::Transformation>& robot_prediction,
        const std::vector<lgmath::se3::Transformation>& reference_pose_vec,
        const std::shared_ptr<std::vector<Pose>> cbit_path_ptr,
        const std::shared_ptr<CBITCorridor> corridor_ptr,
        const lgmath::se3::Transformation& T_w_p_interpolated_closest_to_robot,
        const double& state_p,
        const std::shared_ptr<CBITPath> global_path_ptr,
        unsigned closest_node_idx);

    void publishMPCRollout(const std::vector<lgmath::se3::Transformation>& mpc_prediction, const tactic::Timestamp& stamp, double dt=0.25);
    void publishMPCRollout(const std::vector<std::pair<tactic::Timestamp, lgmath::se3::Transformation>>& mpc_prediction);
    void publishLeaderRollout(const std::vector<lgmath::se3::Transformation>& mpc_prediction, const tactic::Timestamp& stamp, double dt=0.25);
    void publishReferencePoses(const std::vector<lgmath::se3::Transformation>& reference_pose_vec, const tactic::Timestamp& stamp);
    void publishLocalReferencePoses(const std::vector<lgmath::se3::Transformation>& reference_pose_vec, const tactic::Timestamp& stamp);
    void publishCorrectedReferencePoses(const std::vector<lgmath::se3::Transformation>& reference_pose_vec, const tactic::Timestamp& stamp);
    void publishPredictedRobotPath(
        const std::vector<lgmath::se3::Transformation>& predicted_poses,
        const tactic::Timestamp& stamp);
    void publishPredictedErrors(
        const std::vector<std::array<double, 3>>& corrections,
        const tactic::Timestamp& stamp);
    void publishNNInputsOutputs(
        const PredictorInputSnapshot& input_snapshot,
        const tactic::Timestamp& stamp);
    void publishExtrapolatedRobotPose(
        const lgmath::se3::Transformation& T_p_r_extp,
        const tactic::Timestamp& lidar_stamp,
        const tactic::Timestamp& extrap_wall_time);

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr leader_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corridor_pub_l_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corridor_pub_r_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ref_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr local_ref_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr corrected_ref_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_robot_path_pub_;
    rclcpp::Publisher<vtr_path_planning_msgs::msg::PathInfoForExternalNavigation>::SharedPtr path_info_for_external_navigation_pub_;
    rclcpp::Publisher<vtr_path_planning_msgs::msg::PredictedTrackingErrors>::SharedPtr predicted_errors_pub_;
    rclcpp::Publisher<vtr_path_planning_msgs::msg::NNInputs>::SharedPtr nn_inputs_outputs_pub_;
    rclcpp::Publisher<vtr_path_planning_msgs::msg::NNTrueQuantities>::SharedPtr nn_true_quantities_pub_;
    rclcpp::Publisher<vtr_path_planning_msgs::msg::ExtrapolatedRobotPose>::SharedPtr extrapolated_robot_pose_pub_;
};

} // namespace path_planning
} // namespace vtr
