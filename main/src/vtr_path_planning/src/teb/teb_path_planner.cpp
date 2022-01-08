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
 * \file teb_path_planner.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/teb/teb_path_planner.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace teb_local_planner;

namespace vtr {
namespace path_planning {

namespace {

inline std::tuple<double, double, double> T2xyth(
    const tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), std::atan2(Tm(1, 0), Tm(0, 0)));
}

inline tactic::EdgeTransform xyth2T(
    const std::tuple<double, double, double>& xyth) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = std::get<0>(xyth);
  T(1, 3) = std::get<1>(xyth);
  const auto theta = std::get<2>(xyth);
  T.block<2, 2>(0, 0) << std::cos(theta), -std::sin(theta), std::sin(theta),
      std::cos(theta);
  return tactic::EdgeTransform(T);
}

}  // namespace

auto TEBPathPlanner::Config::fromROS(const rclcpp::Node::SharedPtr& node,
                                     const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);
  config->robot_model = node->declare_parameter<std::string>(prefix + ".teb.robot_model", config->robot_model);
  config->robot_radius = node->declare_parameter<double>(prefix + ".teb.robot_radius", config->robot_radius);
  // clang-format on
  config->declareParameters(node, prefix + ".teb");
  config->loadRosParamFromNodeHandle(node, prefix + ".teb");
  return config;
}

TEBPathPlanner::TEBPathPlanner(const rclcpp::Node::SharedPtr& node,
                               const Config::Ptr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback), config_(config) {
  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node->get_node_base_interface(), node->get_node_topics_interface(),
      node->get_node_graph_interface(), node->get_node_services_interface());
  parameter_event_sub_ = parameters_client_->on_parameter_event(
      std::bind(&teb_local_planner::TebConfig::on_parameter_event_callback,
                std::ref(*config_), std::placeholders::_1));
  // for vtr specific visualization
  tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  // create robot footprint/contour model for optimization
  auto robot_model = getRobotFootprintFromParamServer();

  // visualization
  visualization_ = std::make_shared<TebVisualization>(node, *config_);
  visualization_->on_configure();

  // create the planner instance
  if (config_->hcp.enable_homotopy_class_planning) {
    planner_ = std::make_shared<HomotopyClassPlanner>(
        node, *config_, &obstacles_, robot_model, visualization_, &via_points_);
    CLOG(INFO, "path_planning")
        << "Parallel planning in distinctive topologies enabled.";
    // RCLCPP_INFO(logger_,
    //             "Parallel planning in distinctive topologies enabled.");
  } else {
    planner_ = std::make_shared<TebOptimalPlanner>(
        node, *config_, &obstacles_, robot_model, visualization_, &via_points_);
    CLOG(INFO, "path_planning")
        << "Parallel planning in distinctive topologies disabled.";
    // RCLCPP_INFO(logger_,
    //             "Parallel planning in distinctive topologies disabled.");
  }
}

TEBPathPlanner::~TEBPathPlanner() {
  stop();
  CLOG(ERROR, "path_planning") << "resetting the planner.";
  planner_.reset();
  CLOG(ERROR, "path_planning") << "resetting the planner - done.";
}

void TEBPathPlanner::initializeRoute(RobotState& robot_state) {
  /// \todo reset any internal state
}

auto TEBPathPlanner::computeCommand(RobotState& robot_state) -> Command {
  // initialize command to zero velocity
  Command command;

  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(INFO, "path_planning") << "Robot is not localized, stop the robot";
    return Command();
  }

  // T_r_p = T_robot_planning, T_world_planning, T_plannning_goal
  // planning frame is the current localization frame
  auto [T_p_r, T_w_p, T_p_g, curr_sid, target_sid] = [&]() {
    auto lock = chain.guard();
    const auto T_p_r = chain.T_leaf_trunk().inverse();
    const auto T_w_p = chain.T_start_trunk();
    const auto curr_sid = chain.trunkSequenceId();
    const auto target_sid = std::min((size_t)(curr_sid + 2), chain.size() - 1);
    const auto T_w_g = chain.pose(target_sid);
    const auto T_p_g = T_w_p.inverse() * T_w_g;
    return std::make_tuple(T_p_r, T_w_p, T_p_g, curr_sid, target_sid);
  }();

  // get robot pose project onto 2D plane w.r.t. the planning frame
  const auto [rx, ry, rt] = T2xyth(T_p_r);
  const auto robot_pose = PoseSE2(rx, ry, rt);
  const auto T_p_r_2d = xyth2T(T2xyth(T_p_r));

  /// \todo get robot velocity
  const auto robot_velocity = Command();

  // get goal pose project onto 2D plane w.r.t. the planning frame
  const auto [gx, gy, gt] = T2xyth(T_p_g);
  const auto goal_pose = PoseSE2(gx, gy, gt);
  const auto T_p_g_2d = xyth2T(T2xyth(T_p_g));

  // vtr visualization
  visualize(0, T_w_p, T_p_r_2d, T_p_g_2d);

  /// \todo clear planner when we switch to a different planning frame
  CLOG(INFO, "path_planning")
      << "Current time: " << now() << ", planning a trajectory from: [" << rx
      << ", " << ry << ", " << rt << "] to: [" << gx << ", " << gy << ", " << gt
      << "]";

  /// Do not allow config changes during the following optimization step
  std::lock_guard<std::mutex> config_lock(config_->configMutex());

  //
  if (!planner_->plan(robot_pose, goal_pose)) {
    CLOG(WARNING, "path_planning") << "Planning failed!";
    planner_->clearPlanner();  // force reinitialization for next time
    /// \todo add infeasible plan counter
  }

  // check for convergence
  if (planner_->hasDiverged()) {
    CLOG(WARNING, "path_planning") << "Planner has diverged!";
    // Reset everything to start again with the initialization of new
    // trajectories.
    planner_->clearPlanner();
  }

  /// \todo check if the generated trajectory is feasible

  // get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(
          command.linear.x, command.linear.y, command.angular.z,
          config_->trajectory.control_look_ahead_poses)) {
    CLOG(WARNING, "path_planning") << "Computing velocity command failed!";
    planner_->clearPlanner();
  }

  // saturate velocity command
  saturateVelocity(command.linear.x, command.linear.y, command.angular.z,
                   config_->robot.max_vel_x, config_->robot.max_vel_y,
                   config_->robot.max_vel_theta);

  CLOG(INFO, "path_planning")
      << "Velocity command: [" << command.linear.x << ", " << command.linear.y
      << ", " << command.angular.z << "]";

  // visualize the current trajectory
  planner_->visualize();

  return command;
}

RobotFootprintModelPtr TEBPathPlanner::getRobotFootprintFromParamServer() {
  /// \todo support other models
  CLOG(INFO, "path_planning") << "Using point robot model.";
  return std::make_shared<PointRobotFootprint>();
}

void TEBPathPlanner::saturateVelocity(double& vx, double& vy, double& omega,
                                      double max_vel_x, double max_vel_y,
                                      double max_vel_theta) const {
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x || vx < -max_vel_x) ratio_x = std::abs(max_vel_x / vx);
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y) ratio_y = std::abs(max_vel_y / vy);
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);

  vx *= ratio_x;
  vy *= ratio_y;
  omega *= ratio_omega;
}

void TEBPathPlanner::visualize(const tactic::Timestamp& stamp,
                               const tactic::EdgeTransform& T_w_p,
                               const tactic::EdgeTransform& T_p_r,
                               const tactic::EdgeTransform& T_p_g) const {
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
    Eigen::Affine3d T2(T_p_r.matrix());
    auto msg2 = tf2::eigenToTransform(T2);
    msg2.header.frame_id = "planning frame";
    msg2.header.stamp = rclcpp::Time(stamp);
    msg2.child_frame_id = "robot planning";
    tf_bc_->sendTransform(msg2);
  }

  /// Publish the current target in the planning frame
  {
    Eigen::Affine3d T3(T_p_g.matrix());
    auto msg3 = tf2::eigenToTransform(T3);
    msg3.header.frame_id = "planning frame";
    msg3.header.stamp = rclcpp::Time(stamp);
    msg3.child_frame_id = "goal planning";
    tf_bc_->sendTransform(msg3);
  }
}

}  // namespace path_planning
}  // namespace vtr