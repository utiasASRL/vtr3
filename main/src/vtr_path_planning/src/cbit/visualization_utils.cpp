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
 * \file visualization_utils.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/cbit/visualization_utils.hpp"

namespace vtr {
namespace path_planning {


VisualizationUtils::VisualizationUtils(rclcpp::Node::SharedPtr node) {
    tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    mpc_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("mpc_prediction", rclcpp::QoS(1).best_effort().durability_volatile());
    leader_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("leader_mpc_prediction", 10);
    robot_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("robot_path", 10);
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planning_path", 10);
    corridor_pub_l_ = node->create_publisher<nav_msgs::msg::Path>("corridor_path_left", 10);
    corridor_pub_r_ = node->create_publisher<nav_msgs::msg::Path>("corridor_path_right", 10);
    ref_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>("mpc_ref_pose_array", 10);
    path_info_for_external_navigation_pub_ = node->create_publisher<vtr_path_planning_msgs::msg::PathInfoForExternalNavigation>("path_info_for_external_navigation", 10);
}

void VisualizationUtils::visualize(
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
    unsigned closest_node_idx)
    {

    {
        vtr_path_planning_msgs::msg::PathInfoForExternalNavigation path_info_for_external_navigation_msg;
        // path_info_msg.timestamp = stamp;
        path_info_for_external_navigation_msg.header.stamp = rclcpp::Time(stamp);
        path_info_for_external_navigation_msg.p_value = state_p;  // Ensure closest_node_id is defined elsewhere

        // Set robot_pose
        const auto T_w_r_matrix = T_w_r.matrix();
        double robot_x = T_w_r_matrix(0, 3);
        double robot_y = T_w_r_matrix(1, 3);
        double robot_th = atan2(T_w_r_matrix(1, 0), T_w_r_matrix(0, 0));

        path_info_for_external_navigation_msg.robot_pose[0] = robot_x;
        path_info_for_external_navigation_msg.robot_pose[1] = robot_y;
        path_info_for_external_navigation_msg.robot_pose[2] = robot_th;

        // Reference path in robot frame
        std::vector<Pose> path = global_path_ptr->disc_path;  // Assuming disc_path is a member of cbit_path_ptr

        Eigen::MatrixXd path_mat(path.size(), 2);
        Eigen::MatrixXd th_mat(path.size(), 1);
        for (size_t i = 0; i < path.size(); i++) {
            path_mat(i, 0) = path[i].x;
            path_mat(i, 1) = path[i].y;
            th_mat(i, 0) = path[i].yaw;
        }

        // Construct rotation matrix C
        Eigen::Matrix2d C;
        C << cos(robot_th), -sin(robot_th), sin(robot_th), cos(robot_th);
        Eigen::MatrixXd path_mat_rot = (path_mat.rowwise() - Eigen::Vector2d(robot_x, robot_y).transpose()) * C.transpose();
        Eigen::MatrixXd th_mat_rot = th_mat.array() - robot_th;  // Element-wise subtraction

        // Assign path to message
        path_info_for_external_navigation_msg.path.resize(path.size() * 4);  // Resize to accommodate all elements

        for (size_t i = 0; i < path.size(); i++) {
            path_info_for_external_navigation_msg.path[i * 4] = path_mat_rot(i, 0);       // x
            path_info_for_external_navigation_msg.path[i * 4 + 1] = path_mat_rot(i, 1);   // y
            path_info_for_external_navigation_msg.path[i * 4 + 2] = cos(th_mat_rot(i, 0));  // dx
            path_info_for_external_navigation_msg.path[i * 4 + 3] = sin(th_mat_rot(i, 0));  // dy
        }

        // Set mpc_solution
        path_info_for_external_navigation_msg.mpc_solution.resize(mpc_velocities.size() * 2);  // Resize to accommodate all elements
        for (int i = 0; i < mpc_velocities.size(); i++) {
            path_info_for_external_navigation_msg.mpc_solution[i * 2] = mpc_velocities[i](0);  // v
            path_info_for_external_navigation_msg.mpc_solution[i * 2 + 1] = mpc_velocities[i](1);  // w
        }

        path_info_for_external_navigation_msg.closest_node_idx = closest_node_idx;

        // closest interpolated node's pose
        path_info_for_external_navigation_msg.closest_interpolated_node_pose[0] = T_w_p_interpolated_closest_to_robot.matrix()(0, 3);
        path_info_for_external_navigation_msg.closest_interpolated_node_pose[1] = T_w_p_interpolated_closest_to_robot.matrix()(1, 3);
        path_info_for_external_navigation_msg.closest_interpolated_node_pose[2] = atan2(T_w_p_interpolated_closest_to_robot.matrix()(1, 0), T_w_p_interpolated_closest_to_robot.matrix()(0, 0));

        // Compute distance and th offset to closest node
        lgmath::se3::Transformation T_p_interpolated_closest_node_r = T_w_p_interpolated_closest_to_robot.inverse() * T_w_r;
        const auto T_p_interpolated_closest_node_r_matrix = T_p_interpolated_closest_node_r.matrix();
        double distance_to_closest_interpolated_node = sqrt(pow(T_p_interpolated_closest_node_r_matrix(0, 3), 2) + pow(T_p_interpolated_closest_node_r_matrix(1, 3), 2));
        path_info_for_external_navigation_msg.distance_to_closest_interpolated_node = distance_to_closest_interpolated_node;

        double theta_offset_to_closest_interpolated_node = atan2(T_p_interpolated_closest_node_r_matrix(1, 0), T_p_interpolated_closest_node_r_matrix(0, 0));
        // note that this is already wrapped to [-pi, pi] by atan2 calc
        path_info_for_external_navigation_msg.theta_offset_to_closest_interpolated_node = theta_offset_to_closest_interpolated_node;

        // Compute distance to goal node
        const auto& goal_node_pose = global_path_ptr->disc_path.back();  
        double goal_node_x = goal_node_pose.x;
        double goal_node_y = goal_node_pose.y;
        double distance_to_goal_node = sqrt(pow(goal_node_x - robot_x, 2) + pow(goal_node_y - robot_y, 2));
        path_info_for_external_navigation_msg.distance_to_goal_node = distance_to_goal_node;

        // Compute theta offset to goal node  
        double theta_offset_to_goal_node = goal_node_pose.yaw - robot_th;
        path_info_for_external_navigation_msg.theta_offset_to_goal_node = theta_offset_to_goal_node;

        path_info_for_external_navigation_pub_->publish(path_info_for_external_navigation_msg); 
        CLOG(INFO, "cbit.visualization") << "Published path info for external navigation";
    }

    /// Publish the current frame for planning
    {
        Eigen::Affine3d T(T_w_p.matrix());
        auto msg = tf2::eigenToTransform(T);
        msg.header.stamp = rclcpp::Time(stamp);
        msg.header.frame_id = "world";
        msg.child_frame_id = "planning frame";
        tf_bc_->sendTransform(msg);
    }

    /// Publish the current robot in the planning frame
    {
        Eigen::Affine3d T(T_p_r.matrix());
        auto msg = tf2::eigenToTransform(T);
        msg.header.frame_id = "planning frame";
        msg.header.stamp = rclcpp::Time(stamp);
        msg.child_frame_id = "robot planning";
        tf_bc_->sendTransform(msg);
    }

    // Publish MPC extrapolated current robot pose
    {
        Eigen::Affine3d T(T_p_r_extp_mpc.matrix());
        auto msg = tf2::eigenToTransform(T);
        msg.header.frame_id = "planning frame";
        msg.header.stamp = rclcpp::Time(stamp);
        msg.child_frame_id = "robot planning (extrapolated) mpc";
        tf_bc_->sendTransform(msg);
    }

    /// Publishing the MPC horizon prediction
    publishMPCRollout(mpc_prediction, stamp);

    /// Publishing the history of the robots actual pose
    {
        nav_msgs::msg::Path robot_path;
        robot_path.header.frame_id = "world";
        robot_path.header.stamp = rclcpp::Time(stamp);
        auto& poses = robot_path.poses;

        // intermediate states
        for (unsigned i = 0; i < robot_prediction.size(); ++i) {
        auto& pose = poses.emplace_back();
        pose.pose = tf2::toMsg(Eigen::Affine3d(robot_prediction[i].matrix()));
        }
        robot_path_pub_->publish(robot_path);
    }


    // Attempting to publish the actual path which we are receiving from the shared pointer in the cbitplanner
    // The path is stored as a vector of se3 Pose objects from cbit/utils, need to iterate through and construct proper ros2 nav_msgs PoseStamped

    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "world";
        path.header.stamp = rclcpp::Time(stamp);
        auto& poses = path.poses;

        // iterate through the path
        //CLOG(INFO, "path_planning.cbit") << "Trying to publish the path, the size is: " << (*cbit_path_ptr).size();
        geometry_msgs::msg::Pose test_pose;
        for (unsigned i = 0; i < (*cbit_path_ptr).size(); ++i)
        {
        auto& pose = poses.emplace_back();
        // pose.pose = tf2::toMsg(Eigen::Affine3d(T_p_i_vec[i].matrix())); // Example for how to grab the transform from a transform with covariance data type
        test_pose.position.x = (*cbit_path_ptr)[i].x;
        test_pose.position.y = (*cbit_path_ptr)[i].y;
        test_pose.position.z = (*cbit_path_ptr)[i].z;
        test_pose.orientation.x = 0.0;
        test_pose.orientation.y = 0.0;
        test_pose.orientation.z = 0.0;
        test_pose.orientation.w = 1.0;
        pose.pose = test_pose;
        }

        path_pub_->publish(path);
    }



    // Attempting to Publish the left and right dynamic corridor for the current path homotopy class
    {
        nav_msgs::msg::Path corridor_left;
        corridor_left.header.frame_id = "world";
        corridor_left.header.stamp = rclcpp::Time(stamp);
        auto& poses_l = corridor_left.poses;

        nav_msgs::msg::Path corridor_right;
        corridor_right.header.frame_id = "world";
        corridor_right.header.stamp = rclcpp::Time(stamp);
        auto& poses_r = corridor_right.poses;

        // iterate through the corridor paths
        geometry_msgs::msg::Pose test_pose_l;
        geometry_msgs::msg::Pose test_pose_r;
        for (unsigned i = 0; i < corridor_ptr->x_left.size(); i++) 
        {
        //lhs
        auto& pose_l = poses_l.emplace_back();
        test_pose_l.position.x = corridor_ptr->x_left[i];
        test_pose_l.position.y = corridor_ptr->y_left[i];
        test_pose_l.position.z = 0.0; // setting this 0.0 for now for flat world assumption, but long term we might want to add a z component
        test_pose_l.orientation.x = 0.0;
        test_pose_l.orientation.y = 0.0;
        test_pose_l.orientation.z = 0.0;
        test_pose_l.orientation.w = 1.0;
        pose_l.pose = test_pose_l;

        // rhs
        auto& pose_r = poses_r.emplace_back();
        test_pose_r.position.x = corridor_ptr->x_right[i];
        test_pose_r.position.y = corridor_ptr->y_right[i];
        test_pose_r.position.z = 0.0; // setting this 0.0 for now for flat world assumption, but long term we might want to add a z component
        test_pose_r.orientation.x = 0.0;
        test_pose_r.orientation.y = 0.0;
        test_pose_r.orientation.z = 0.0;
        test_pose_r.orientation.w = 1.0;
        pose_r.pose = test_pose_r;
        }

        corridor_pub_l_->publish(corridor_left);
        corridor_pub_r_->publish(corridor_right);
    }

    publishReferencePoses(reference_pose_vec);

    
    return;
}

    // Attempting to Publish the reference poses used in the mpc optimization as a pose array
    void VisualizationUtils::publishReferencePoses(const std::vector<lgmath::se3::Transformation>& ref_poses) {

        // create a PoseArray message
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.frame_id = "world";

        // fill the PoseArray with some sample poses
        for (size_t i = 0; i < ref_poses.size(); i++) {
            auto T1 = ref_poses[i].matrix();
            pose_array_msg.poses.push_back(tf2::toMsg(Eigen::Affine3d(T1)));
        }
        ref_pose_pub_->publish(pose_array_msg);
    }
    

    void VisualizationUtils::publishMPCRollout(const std::vector<lgmath::se3::Transformation>& mpc_prediction, const tactic::Timestamp& stamp, double dt) {
        nav_msgs::msg::Path mpc_path;
        mpc_path.header.frame_id = "world";
        mpc_path.header.stamp = rclcpp::Time(stamp);
        auto& poses = mpc_path.poses;

        // intermediate states
        for (unsigned i = 0; i < mpc_prediction.size(); ++i) {
            auto& pose = poses.emplace_back();
            pose.pose = tf2::toMsg(Eigen::Affine3d(mpc_prediction[i].matrix()));
            pose.header.stamp = rclcpp::Time(stamp + i*dt*1e9);
        }
        mpc_path_pub_->publish(mpc_path);
    }

    void VisualizationUtils::publishMPCRollout(const std::vector<std::pair<tactic::Timestamp, lgmath::se3::Transformation>>& mpc_prediction) {
        nav_msgs::msg::Path mpc_path;
        mpc_path.header.frame_id = "world";
        mpc_path.header.stamp = rclcpp::Time(mpc_prediction[1].first);
        auto& poses = mpc_path.poses;

        // intermediate states
        for (unsigned i = 0; i < mpc_prediction.size(); ++i) {
            auto& pose = poses.emplace_back();
            pose.pose = tf2::toMsg(Eigen::Affine3d(mpc_prediction[i].second.matrix()));
            pose.header.stamp = rclcpp::Time(mpc_prediction[i].first);
            pose.header.frame_id = "world";
        }
        mpc_path_pub_->publish(mpc_path);
    }

    void VisualizationUtils::publishLeaderRollout(const std::vector<lgmath::se3::Transformation>& mpc_prediction, const tactic::Timestamp& stamp, double dt) {
        nav_msgs::msg::Path mpc_path;
        mpc_path.header.frame_id = "world";
        mpc_path.header.stamp = rclcpp::Time(stamp);
        auto& poses = mpc_path.poses;

        // intermediate states
        for (unsigned i = 0; i < mpc_prediction.size(); ++i) {
            auto& pose = poses.emplace_back();
            pose.pose = tf2::toMsg(Eigen::Affine3d(mpc_prediction[i].matrix()));
            pose.header.stamp = rclcpp::Time(stamp + i*dt*1e9);
            pose.header.frame_id = "world";
        }
        leader_path_pub_->publish(mpc_path);
    }


} // namespace path_planning
} // namespace vtr