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
 * \file unicycle_mpc_path_tracker_follower.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/unicycle_mpc_path_tracker_follower.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

namespace vtr {
namespace path_planning {

namespace {
// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Vector2d saturateVel(const Eigen::Vector2d& applied_vel, double v_lim, double w_lim) {
  return {std::clamp(applied_vel(0), -v_lim, v_lim), std::clamp(applied_vel(1), -w_lim, w_lim)};
}
}

PathInterpolator::PathInterpolator(const nav_msgs::msg::Path::SharedPtr& path) {
  using namespace vtr::common::conversions;
  
  for(const auto& pose : path->poses) {
    path_info_[rclcpp::Time(pose.header.stamp).nanoseconds()] = tfFromPoseMessage(pose.pose);
  }
}

PathInterpolator::Transformation PathInterpolator::at(tactic::Timestamp time) const {
  auto up_it = path_info_.lower_bound(time);
  if (up_it == path_info_.begin()){
    CLOG(ERROR, "mpc.follower") << "Finding first element of map!";
    const Transformation T_w_p1 = up_it->second;
    const auto t_1 = up_it->first;
    
    up_it--;
    const Transformation T_w_p0 = up_it->second;
    const auto t_0 = up_it->first;
    CLOG(DEBUG, "mpc.follower") << "Time 1 " << t_1 << " Time 0 " << t_0;

    return T_w_p1;
  } else {
    const Transformation T_w_p1 = up_it->second;
    const auto t_1 = up_it->first;

    up_it--;
    const Transformation T_w_p0 = up_it->second;
    const auto t_0 = up_it->first;

    const auto dt = t_1 - t_0;
    try {
      auto xi_interp = (T_w_p0.inverse() * T_w_p1).vec() * ((time - t_0) / dt);
      return T_w_p0 * Transformation(Eigen::Matrix<double, 6, 1>(xi_interp));
    } catch (std::exception &e) {
      return T_w_p0;
    }
    }
}

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto UnicycleMPCPathFollower::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  auto base_config = std::static_pointer_cast<BasePathPlanner::Config>(config);
  *base_config =  *BasePathPlanner::Config::fromROS(node, prefix);

  config->leader_path_topic = node->declare_parameter<std::string>(prefix + ".leader_path_topic", config->leader_path_topic);
  config->following_offset = node->declare_parameter<double>(prefix + ".follow_distance", config->following_offset);
  config->distance_margin = node->declare_parameter<double>(prefix + ".distance_margin", config->distance_margin);

  // MPC Configs:
  // SPEED SCHEDULER PARAMETERS
  config->planar_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.planar_curv_weight", config->planar_curv_weight);
  config->profile_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.profile_curv_weight", config->profile_curv_weight);
  config->eop_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.eop_weight", config->eop_weight);
  config->min_vel = node->declare_parameter<double>(prefix + ".speed_scheduler.min_vel", config->min_vel);

  // CONTROLLER PARAMS
  config->extrapolate_robot_pose = node->declare_parameter<bool>(prefix + ".mpc.extrapolate_robot_pose", config->extrapolate_robot_pose);
  config->mpc_verbosity = node->declare_parameter<bool>(prefix + ".mpc.mpc_verbosity", config->mpc_verbosity);
  config->forward_vel = node->declare_parameter<double>(prefix + ".mpc.forward_vel", config->forward_vel);
  config->max_lin_vel = node->declare_parameter<double>(prefix + ".mpc.max_lin_vel", config->max_lin_vel);
  config->max_ang_vel = node->declare_parameter<double>(prefix + ".mpc.max_ang_vel", config->max_ang_vel);
  config->max_lin_acc = node->declare_parameter<double>(prefix + ".mpc.max_lin_acc", config->max_lin_acc);
  config->max_ang_acc = node->declare_parameter<double>(prefix + ".mpc.max_ang_acc", config->max_ang_acc);
  config->robot_linear_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_linear_velocity_scale", config->robot_linear_velocity_scale);
  config->robot_angular_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_angular_velocity_scale", config->robot_angular_velocity_scale);

  // MISC
  config->command_history_length = node->declare_parameter<int>(prefix + ".mpc.command_history_length", config->command_history_length);

  return config;
}


// Declare class as inherited from the BasePathPlanner
UnicycleMPCPathFollower::UnicycleMPCPathFollower(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, graph, callback), config_(config), solver_{config_->mpc_verbosity}, robot_state_{robot_state}, graph_{graph} {
  applied_vel_ << 0,
                  0;
  leader_vel_ << 0,
                 0;
  vel_history.reserve(config_->command_history_length);
  for (int i = 0; i < config_->command_history_length; i++)
  {
    vel_history.push_back(applied_vel_);
  }
  
  vis_ = std::make_shared<VisualizationUtils>(robot_state->node.ptr());

  using std::placeholders::_1;
  CLOG(INFO, "mpc.follower") << config->leader_path_topic;
  CLOG(INFO, "mpc.follower") << config->distance_margin;

  leaderRolloutSub_ = robot_state->node->create_subscription<PathMsg>(config->leader_path_topic, rclcpp::SystemDefaultsQoS(), std::bind(&UnicycleMPCPathFollower::onLeaderPath, this, _1));
}


UnicycleMPCPathFollower::~UnicycleMPCPathFollower() {}

auto UnicycleMPCPathFollower::computeCommand(RobotState& robot_state) -> Command {
  auto raw_command = computeCommand_(robot_state);
  
  Eigen::Vector2d output_vel = {raw_command.linear.x, raw_command.angular.z};

  // Apply robot motor controller calibration scaling factors if applicable
  output_vel(0) = output_vel(0) * config_->robot_linear_velocity_scale;
  output_vel(1) = output_vel(1) * config_->robot_angular_velocity_scale;

  // If required, saturate the output velocity commands based on the configuration limits
  CLOG(DEBUG, "cbit.control") << "Saturating the velocity command if required";
  Eigen::Vector2d saturated_vel = saturateVel(output_vel, config_->max_lin_vel, config_->max_ang_vel);
  CLOG(INFO, "cbit.control") << "The Saturated linear velocity is:  " << saturated_vel(0) << " The angular vel is: " << saturated_vel(1);
  
  Command command;
  command.linear.x = saturated_vel(0);
  command.angular.z = saturated_vel(1);
  prev_vel_stamp_ = now();
  applied_vel_ = saturated_vel;

  // Store the result in memory so we can use previous state values to re-initialize and extrapolate the robot pose in subsequent iterations
  vel_history.erase(vel_history.begin());
  vel_history.push_back(applied_vel_);

  CLOG(INFO, "cbit.control")
    << "Final control command: [" << command.linear.x << ", "
    << command.linear.y << ", " << command.linear.z << ", "
    << command.angular.x << ", " << command.angular.y << ", "
    << command.angular.z << "]";
  
  return command;
}


// Generate twist commands to track the planned local path (function is called at the control rate)
auto UnicycleMPCPathFollower::computeCommand_(RobotState& robot_state) -> Command {
  auto& chain = robot_state.chain.ptr();
  if (!chain->isLocalized()) {
    CLOG(WARNING, "cbit.control") << "Robot is not localized, commanding the robot to stop";
    return Command();
  }

  if (recentLeaderPath_ == nullptr) {
    CLOG_EVERY_N(10000, WARNING, "cbit.control") << "Follower has received no path from the leader yet. Stopping";
    return Command();
  }

  // retrieve the transform info from the localization chain for the current robot state
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = getChainInfo(*chain);

  if (robot_state.node->get_clock()->now() - rclcpp::Time(recentLeaderPath_->header.stamp) > rclcpp::Duration(1, 0)) {
    CLOG_EVERY_N(5000, WARNING,"cbit.control") << "Follower has received no path from the leader in more than 1 second. Stopping";
    return Command();
  }

  // Store the current robot state in the robot state path so it can be visualized
  auto T_w_r = T_w_p * T_p_r;

  CasadiUnicycleMPCFollower::Config mpcConfig;
  mpcConfig.vel_max = {config_->max_lin_vel, config_->max_ang_vel};
  mpcConfig.distance = config_->following_offset;
  mpcConfig.distance_margin = config_->distance_margin;

  mpcConfig.VF = leader_vel_(0, 0);


  // EXTRAPOLATING ROBOT POSE INTO THE FUTURE TO COMPENSATE FOR SYSTEM DELAYS
  auto T_p_r_extp = T_p_r;
  auto curr_time = stamp;  // always in nanoseconds

  if (config_->extrapolate_robot_pose) {
    curr_time = now();
    auto dt = static_cast<double>(curr_time - stamp) * 1e-9 - 0.05;
    if (fabs(dt) > 0.25) { 
      CLOG(WARNING, "cbit") << "Pose extrapolation was requested but the time delta is " << dt << "s.\n"
            << "Ignoring extrapolation requestion. Check your time sync!";
      dt = 0;
    }

    CLOG(DEBUG, "cbit.debug") << "Robot velocity Used for Extrapolation: " << -w_p_r_in_r.transpose() << " dt: " << dt << std::endl;
    Eigen::Matrix<double, 6, 1> xi_p_r_in_r(-dt * w_p_r_in_r);
    T_p_r_extp = T_p_r * tactic::EdgeTransform(xi_p_r_in_r);

    CLOG(DEBUG, "cbit.debug") << "New extrapolated pose:"  << T_p_r_extp;
  }

  lgmath::se3::Transformation T0 = T_p_r_extp;
  mpcConfig.T0 = tf_to_global(T0);

  CLOG(DEBUG, "cbit.control") << "Last velocity " << w_p_r_in_r << " with stamp " << stamp;

  double state_p = findRobotP(T_w_p * T_p_r_extp, chain);

  std::vector<double> p_rollout;
  for(int j = 1; j < mpcConfig.N+1; j++){
    p_rollout.push_back(state_p + j*mpcConfig.VF*mpcConfig.DT);
  }

  mpcConfig.follower_reference_poses.clear();
  auto referenceInfo = generateHomotopyReference(p_rollout, chain);
  for(const auto& Tf : referenceInfo.poses) {
    mpcConfig.follower_reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
    CLOG(DEBUG, "mpc.follower") << "Target " << tf_to_global(T_w_p.inverse() *  Tf);
  }

  mpcConfig.leader_reference_poses.clear();
  for (uint i = 0; i < mpcConfig.N; i++){
    const auto T_w_lp = leaderPathInterp_->at(curr_time + (1+i) * mpcConfig.DT * 1e9);
    mpcConfig.leader_reference_poses.push_back(tf_to_global(T_w_p.inverse() *  T_w_lp));
    CLOG(DEBUG, "mpc.follower") << "Leader Target " << tf_to_global(T_w_p.inverse() *  T_w_lp);

  }
  // mpcConfig.leader_reference_poses = mpcConfig.follower_reference_poses;

  // mpcConfig.up_barrier_q = referenceInfo.barrier_q_max;
  // mpcConfig.low_barrier_q = referenceInfo.barrier_q_min;
  
  mpcConfig.previous_vel = {-w_p_r_in_r(0, 0), -w_p_r_in_r(5, 0)};
  

  // Create and solve the casadi optimization problem
  std::vector<lgmath::se3::Transformation> mpc_poses;
  // return the computed velocity command for the first time step
  Command command;
  std::vector<Eigen::Vector2d> mpc_velocities;
  try {
    CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
    auto mpc_res = solver_.solve(mpcConfig);
    
    for(int i = 0; i < mpc_res["pose"].columns(); i++) {
      const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
      mpc_poses.push_back(T_w_p * tf_from_global(pose_i[0], pose_i[1], pose_i[2]));
    }

    CLOG(INFO, "cbit.control") << "Successfully solved MPC problem";
    const auto& mpc_vel_vec = mpc_res["vel"](casadi::Slice(), 0).get_elements();

    command.linear.x = mpc_vel_vec[0];
    command.angular.z = mpc_vel_vec[1];

    // Get all the mpc velocities 
    for (int i = 0; i < mpc_res["vel"].columns(); i++) {
      const auto& vel_i = mpc_res["vel"](casadi::Slice(), i).get_elements();
      mpc_velocities.emplace_back(vel_i[0], vel_i[1]);
    }

  } catch(std::exception &e) {
    CLOG(WARNING, "cbit.control") << "casadi failed! " << e.what() << " Commanding to Stop the Vehicle";
    return Command();
  }

  vis_->publishMPCRollout(mpc_poses, curr_time, mpcConfig.DT);

  CLOG(INFO, "cbit.control") << "The linear velocity is:  " << command.linear.x << " The angular vel is: " << command.angular.z;

  return command;
  
}


void UnicycleMPCPathFollower::onLeaderPath(const PathMsg::SharedPtr path) {
  using namespace vtr::common::conversions;
  
  recentLeaderPath_ = path;

  //reconstruct velocity
  if (path->poses.size() > 1) {
    const Transformation T_w_p0 = tfFromPoseMessage(path->poses[0].pose);
    const Transformation T_w_p1 =  tfFromPoseMessage(path->poses[1].pose);
    const auto dt = rclcpp::Time(path->poses[1].header.stamp) - rclcpp::Time(path->poses[0].header.stamp);
    auto vel = (T_w_p0.inverse() * T_w_p1).vec() / dt.seconds();
    leader_vel_ << vel(0, 0), vel(5, 0);
    CLOG(DEBUG, "mpc.follower") << leader_vel_;
  } 

  leaderPathInterp_ = std::make_shared<const PathInterpolator>(path);

}

}  // namespace path_planning
}  // namespace vtr