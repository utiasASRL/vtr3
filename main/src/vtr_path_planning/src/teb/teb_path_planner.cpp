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
  // basics
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);
  config->visualize = node->declare_parameter<bool>(prefix + ".teb.visualize", config->visualize);
  // extrapolation
  config->extrapolate = node->declare_parameter<bool>(prefix + ".teb.extrapolate", config->extrapolate);
  config->extrapolation_timeout = node->declare_parameter<double>(prefix + ".teb.extrapolation_timeout", config->extrapolation_timeout);
  // lookahead
  config->lookahead_distance = node->declare_parameter<double>(prefix + ".teb.lookahead_distance", config->lookahead_distance);
  // robot configuration
  config->robot_model = node->declare_parameter<std::string>(prefix + ".teb.robot_model", config->robot_model);
  config->robot_radius = node->declare_parameter<double>(prefix + ".teb.robot_radius", config->robot_radius);
  // clang-format on
  config->declareParameters(node, prefix + ".teb");
  config->loadRosParamFromNodeHandle(node, prefix + ".teb");
  // set up callback for changes to parameters
  config->ros_parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(
          node->get_node_base_interface(), node->get_node_topics_interface(),
          node->get_node_graph_interface(),
          node->get_node_services_interface());
  config->ros_parameter_event_sub =
      config->ros_parameters_client->on_parameter_event(
          std::bind(&teb_local_planner::TebConfig::on_parameter_event_callback,
                    std::ref(*config), std::placeholders::_1));
  return config;
}

TEBPathPlanner::TEBPathPlanner(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback), config_(config) {
  const auto node = robot_state->node.ptr();
  // for vtr specific visualization
  tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("planning_path", 10);

  // create robot footprint/contour model for optimization
  auto robot_model = [&]() -> RobotFootprintModelPtr {
    if (config_->robot_model == "point")
      return std::make_shared<PointRobotFootprint>();
    else if (config_->robot_model == "circular")
      return std::make_shared<CircularRobotFootprint>(config_->robot_radius);
    else
      throw std::runtime_error("Unknown robot model " + config_->robot_model);
    return std::make_shared<PointRobotFootprint>();
  }();

  // visualization
  visualization_ = std::make_shared<TebVisualization>(node, *config_);
  visualization_->on_configure();

  // create the planner instance
  if (config_->hcp.enable_homotopy_class_planning) {
    planner_ = std::make_shared<HomotopyClassPlanner>(
        node, *config_, &obstacles_, robot_model, visualization_, &via_points_,
        &costmaps_);
    CLOG(INFO, "path_planning.teb")
        << "Parallel planning in distinctive topologies enabled.";
    // RCLCPP_INFO(logger_,
    //             "Parallel planning in distinctive topologies enabled.");
  } else {
    planner_ = std::make_shared<TebOptimalPlanner>(node, *config_, &obstacles_,
                                                   robot_model, visualization_,
                                                   &via_points_, &costmaps_);
    CLOG(INFO, "path_planning.teb")
        << "Parallel planning in distinctive topologies disabled.";
    // RCLCPP_INFO(logger_,
    //             "Parallel planning in distinctive topologies disabled.");
  }
}

TEBPathPlanner::~TEBPathPlanner() { stop(); }

void TEBPathPlanner::initializeRoute(RobotState& robot_state) {
  /// \todo reset any internal state
}

auto TEBPathPlanner::computeCommand(RobotState& robot_state) -> Command {
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "path_planning.teb")
        << "Robot is not localized, command to stop the robot";
    return Command();
  }

  // retrieve info from the localization chain
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_p_g, T_p_i_vec, curr_sid] =
      chain_info;

  // get robot pose project onto 2D plane w.r.t. the planning frame
  const auto T_p_r_2d = xyth2T(T2xyth(T_p_r));

  // keep current time
  const auto curr_time = now();  // always in nanoseconds
  const auto dt = static_cast<double>(curr_time - stamp) * 1e-9;
  if (config_->extrapolate && std::abs(dt) > config_->extrapolation_timeout) {
    CLOG(WARNING, "path_planning.teb")
        << "Time difference b/t estimation and planning: " << dt
        << ", extrapolation timeout reached, stopping the robot";
    return Command();
  }

  // extrapolate robot pose if required
  const auto T_p_r_extp = [&]() {
    if (!config_->extrapolate) return T_p_r;
    // extrapolate based on current time
    Eigen::Matrix<double, 6, 1> xi_p_r_in_r(dt * w_p_r_in_r);
    return T_p_r * tactic::EdgeTransform(xi_p_r_in_r).inverse();
  }();
  // get robot pose project onto 2D plane w.r.t. the planning frame
  const auto T_p_r_extp_2d = xyth2T(T2xyth(T_p_r_extp));
  const auto [rx, ry, rt] = T2xyth(T_p_r_extp);
  const auto robot_pose = PoseSE2(rx, ry, rt);

  // get robot velocity
  Command robot_velocity;
  robot_velocity.linear.x = config_->extrapolate ? -w_p_r_in_r(0) : 0.0;
  robot_velocity.linear.y = config_->extrapolate ? -w_p_r_in_r(1) : 0.0;
  robot_velocity.linear.z = config_->extrapolate ? -w_p_r_in_r(2) : 0.0;
  robot_velocity.angular.x = config_->extrapolate ? -w_p_r_in_r(3) : 0.0;
  robot_velocity.angular.y = config_->extrapolate ? -w_p_r_in_r(4) : 0.0;
  robot_velocity.angular.z = config_->extrapolate ? -w_p_r_in_r(5) : 0.0;

  // get intermediate poses project onto 2D plane w.r.t. the planning frame
  decltype(T_p_i_vec) T_p_i_2d_vec;
  via_points_.clear();
  for (const auto& T_p_i : T_p_i_vec) {
    const auto [ix, iy, it] = T2xyth(T_p_i);
    (void)it;
    via_points_.emplace_back(Eigen::Vector2d(ix, iy));
    T_p_i_2d_vec.emplace_back(xyth2T(T2xyth(T_p_i)));
  }

  // get goal pose project onto 2D plane w.r.t. the planning frame
  const auto [gx, gy, gt] = T2xyth(T_p_g);
  const auto goal_pose = PoseSE2(gx, gy, gt);
  const auto T_p_g_2d = xyth2T(T2xyth(T_p_g));

  // vtr visualization
  if (config_->visualize)
    visualize(0, T_w_p, T_p_r_2d, T_p_r_extp_2d, T_p_g_2d, T_p_i_2d_vec);

  /// \todo clear planner when we switch to a different planning frame

  CLOG(DEBUG, "path_planning.teb")
      << "Current time: " << curr_time << ", leaf time: " << stamp
      << "Time difference in seconds: " << dt << std::endl
      << "Robot velocity: " << -w_p_r_in_r.transpose() << std::endl
      << "Planning a trajectory from: [" << rx << ", " << ry << ", " << rt
      << "] to: [" << gx << ", " << gy << ", " << gt << "]";

  /// Do not allow config changes during the following optimization step
  std::lock_guard<std::mutex> config_lock(config_->configMutex());

  //
  if (!planner_->plan(robot_pose, goal_pose, &robot_velocity,
                      config_->goal_tolerance.free_goal_vel)) {
    CLOG(WARNING, "path_planning.teb") << "Planning failed!";
    planner_->clearPlanner();  // force reinitialization for next time
    /// \todo add infeasible plan counter
  }

  // check for convergence
  if (planner_->hasDiverged()) {
    CLOG(WARNING, "path_planning.teb") << "Planner has diverged!";
    // Reset everything to start again with the initialization of new
    // trajectories.
    planner_->clearPlanner();
  }

  /// \todo check if the generated trajectory is feasible

  // get the velocity command for this sampling interval
  Command command;
  if (!planner_->getVelocityCommand(
          command.linear.x, command.linear.y, command.angular.z,
          config_->trajectory.control_look_ahead_poses)) {
    CLOG(WARNING, "path_planning.teb") << "Computing velocity command failed!";
    planner_->clearPlanner();
  }

  // saturate velocity command
  saturateVelocity(command.linear.x, command.linear.y, command.angular.z,
                   config_->robot.max_vel_x, config_->robot.max_vel_y,
                   config_->robot.max_vel_theta);

  CLOG(DEBUG, "path_planning.teb")
      << "Final control command: [" << command.linear.x << ", "
      << command.linear.y << ", " << command.linear.z << ", "
      << command.angular.x << ", " << command.angular.y << ", "
      << command.angular.z << "]";

  // visualize the current trajectory
  if (config_->visualize) planner_->visualize();

  return command;
}

auto TEBPathPlanner::getChainInfo(RobotState& robot_state) -> ChainInfo {
  auto& chain = *robot_state.chain;
  auto lock = chain.guard();
  const auto stamp = chain.leaf_stamp();
  const auto w_p_r_in_r = chain.leaf_velocity();
  const auto T_p_r = chain.T_leaf_trunk().inverse();
  const auto T_w_p = chain.T_start_trunk();
  const auto curr_sid = chain.trunkSequenceId();
  auto target_sid = std::min(curr_sid + 1, (unsigned)(chain.size() - 1));
  // intermediate poses as waypoints
  std::vector<tactic::EdgeTransform> T_p_i_vec;
  const auto curr_dist = chain.dist(curr_sid);
  while (((chain.dist(target_sid) - curr_dist) < config_->lookahead_distance) &&
         target_sid < (chain.size() - 1)) {
    const auto T_w_i = chain.pose(target_sid);
    const auto T_p_i = T_w_p.inverse() * T_w_i;
    T_p_i_vec.emplace_back(T_p_i);
    ++target_sid;
  }
  const auto T_w_g = chain.pose(target_sid);
  const auto T_p_g = T_w_p.inverse() * T_w_g;

  return ChainInfo{stamp, w_p_r_in_r, T_p_r, T_w_p, T_p_g, T_p_i_vec, curr_sid};
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

void TEBPathPlanner::visualize(
    const tactic::Timestamp& stamp, const tactic::EdgeTransform& T_w_p,
    const tactic::EdgeTransform& T_p_r, const tactic::EdgeTransform& T_p_r_extp,
    const tactic::EdgeTransform& T_p_g,
    const std::vector<tactic::EdgeTransform>& T_p_i_vec) const {
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

  /// Publish the extrapolated robot in the planning frame
  {
    Eigen::Affine3d T(T_p_r_extp.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "planning frame";
    msg.header.stamp = rclcpp::Time(stamp);
    msg.child_frame_id = "robot planning (extrapolated)";
    tf_bc_->sendTransform(msg);
  }

  /// Publish the intermediate goals in the planning frame
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "planning frame";
    path.header.stamp = rclcpp::Time(stamp);
    auto& poses = path.poses;
    // robot itself
    {
      auto& pose = poses.emplace_back();
      pose.pose = tf2::toMsg(Eigen::Affine3d(T_p_r_extp.matrix()));
    }
    // intermediate goals
    for (unsigned i = 0; i < T_p_i_vec.size(); ++i) {
      auto& pose = poses.emplace_back();
      pose.pose = tf2::toMsg(Eigen::Affine3d(T_p_i_vec[i].matrix()));
    }
    // also add the goal
    {
      auto& pose = poses.emplace_back();
      pose.pose = tf2::toMsg(Eigen::Affine3d(T_p_g.matrix()));
    }
    path_pub_->publish(path);
  }

  /// Publish the current goal in the planning frame
  {
    Eigen::Affine3d T(T_p_g.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "planning frame";
    msg.header.stamp = rclcpp::Time(stamp);
    msg.child_frame_id = "goal planning";
    tf_bc_->sendTransform(msg);
  }
}

}  // namespace path_planning
}  // namespace vtr