// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file bicycle_mpc_path_tracker.cpp
 * \author Luka Antonyshyn, Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/base_mpc_path_tracker.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

namespace vtr::path_planning {
namespace {
// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Vector2d saturateVel(const Eigen::Vector2d& applied_vel, double v_lim, double w_lim) {
  return {std::clamp(applied_vel(0), -v_lim, v_lim), std::clamp(applied_vel(1), -w_lim, w_lim)};
}
}

auto BaseMPCPathTracker::Config::loadConfig(BaseMPCPathTracker::Config::Ptr config, 
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix)->void{
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
  config->repeat_flipped = node->declare_parameter<bool>(prefix + ".mpc.repeat_flipped", config->repeat_flipped);
}

// Subclasses must implement their own Config::fromROS.
auto BaseMPCPathTracker::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  auto base_config = std::static_pointer_cast<BasePathPlanner::Config>(config);
  *base_config =  *BasePathPlanner::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);

  return config;
}

BaseMPCPathTracker::BaseMPCPathTracker(const Config::ConstPtr& config,
                                       const RobotState::Ptr& robot_state,
                                       const tactic::GraphBase::Ptr& graph,
                                       const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, graph, callback),
      base_config_(config),
      robot_state_{robot_state} {

  applied_vel_ << 0, 0;
  vel_history.reserve(base_config_->command_history_length);
  for (int i = 0; i < base_config_->command_history_length; i++) {
    vel_history.push_back(applied_vel_);
  }

  vis_ = std::make_shared<VisualizationUtils>(robot_state->node.ptr());
}

BaseMPCPathTracker::~BaseMPCPathTracker() {}

auto BaseMPCPathTracker::computeCommand(RobotState& robot_state) -> Command {
  auto raw_command = computeCommand_(robot_state);

  Eigen::Vector2d output_vel = {raw_command.linear.x, raw_command.angular.z};

  // Apply robot motor controller calibration scaling factors if applicable
  output_vel(0) = output_vel(0) * base_config_->robot_linear_velocity_scale;
  output_vel(1) = output_vel(1) * base_config_->robot_angular_velocity_scale;

  // If required, saturate the output velocity commands based on the
  // configuration limits
  CLOG(DEBUG, "cbit.control") << "Saturating the velocity command if required";
  Eigen::Vector2d saturated_vel =
      saturateVel(output_vel, base_config_->max_lin_vel, base_config_->max_ang_vel);
  CLOG(INFO, "cbit.control")
      << "The Saturated linear velocity is:  " << saturated_vel(0)
      << " The angular vel is: " << saturated_vel(1);

  // For now, we only admit controllers that output a linear and angular
  // velocity command Even the hunters use this format despit Ackermann steering
  Command command;
  command.linear.x = saturated_vel(0);
  command.angular.z = saturated_vel(1);
  prev_vel_stamp_ = robot_state.node->get_clock()->now().nanoseconds();
  applied_vel_ = saturated_vel;

  // Store the result in memory so we can use previous state values to
  // re-initialize and extrapolate the robot pose in subsequent iterations
  vel_history.erase(vel_history.begin());
  vel_history.push_back(applied_vel_);

  CLOG(INFO, "cbit.control")
      << "Final control command: [" << command.linear.x << ", "
      << command.linear.y << ", " << command.linear.z << ", "
      << command.angular.x << ", " << command.angular.y << ", "
      << command.angular.z << "]";

  return command;
}

// Generate twist commands to track the planned local path (function is called
// at the control rate)
auto BaseMPCPathTracker::computeCommand_(RobotState& robot_state) -> Command {
  auto& chain = robot_state.chain.ptr();
  if (!chain->isLocalized()) {
    CLOG(WARNING, "cbit.control")
        << "Robot is not localized, commanding the robot to stop";
    return Command();
  }


  // retrieve the transform info from the localization chain for the current
  // robot state
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);

  // Store the current robot state in the robot state path so it can be
  // visualized
  auto T_w_r = T_w_p * T_p_r;

  // EXTRAPOLATING ROBOT POSE INTO THE FUTURE TO COMPENSATE FOR SYSTEM DELAYS
  auto T_p_r_extp = T_p_r;
  auto curr_time = stamp;
  if (base_config_->extrapolate_robot_pose) {
    curr_time = robot_state.node->get_clock()
                               ->now().nanoseconds();  // always in nanoseconds
                               
    auto dt = static_cast<double>(curr_time - stamp) * 1e-9;
    CLOG(DEBUG, "cbit.debug")
        << "Robot velocity Used for Extrapolation: " << -w_p_r_in_r.transpose()
        << " dt: " << dt << std::endl;
    Eigen::Matrix<double, 6, 1> xi_p_r_in_r(-dt * w_p_r_in_r);
    T_p_r_extp = T_p_r * tactic::EdgeTransform(xi_p_r_in_r);

    CLOG(DEBUG, "cbit.debug") << "New extrapolated pose:" << T_p_r_extp;
  }

  auto segment_info = findRobotSegmentInfo(T_w_p * T_p_r_extp, chain);

  auto dir = segment_info.dir;
  double state_p = segment_info.start_p;
  bool isReverse = (dir == tactic::Direction::Backward);
  bool dir_switch = segment_info.direction_switch;

  auto mpcConfig = getMPCConfig(isReverse, w_p_r_in_r, applied_vel_);

  if (!isMPCStateValid(mpcConfig, curr_time)){
    return Command();
  }

  lgmath::se3::Transformation T0 = T_p_r_extp;
  mpcConfig->T0 = tf_to_global(T0);

  CLOG(DEBUG, "cbit.control")
      << "Last velocity " << w_p_r_in_r << " with stamp " << stamp;
  // Schedule speed based on path curvatures + other factors
  // TODO refactor to accept the chain and use the curvature of the links
  mpcConfig->VF = ScheduleSpeed(
      chain,
      {base_config_->forward_vel, base_config_->min_vel, base_config_->planar_curv_weight,
       base_config_->profile_curv_weight, base_config_->eop_weight, 7}, dir_switch);

  loadMPCPath(mpcConfig, T_w_p, T_p_r_extp, state_p, robot_state, curr_time);

  // Create and solve the casadi optimization problem
  std::vector<std::pair<tactic::Timestamp, lgmath::se3::Transformation>> mpc_poses;
  mpc_poses.push_back(std::make_pair(stamp, T_w_p*T_p_r));
  // return the computed velocity command for the first time step
  Command command;
  std::vector<Eigen::Vector2d> mpc_velocities;
  try {
    auto mpc_res = callSolver(mpcConfig);

    for (int i = 0; i < mpc_res["pose"].columns(); i++) {
      const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
      mpc_poses.push_back(std::make_pair(curr_time + i*mpcConfig->DT*1e9, T_w_p * tf_from_global(pose_i[0], pose_i[1], pose_i[2])));
    }
  
    CLOG(INFO, "cbit.control") << "Successfully solved MPC problem";
    const auto& mpc_vel_vec = mpc_res["vel"](casadi::Slice(), 0).get_elements();
  
    command.linear.x = mpc_vel_vec[0];
    command.angular.z = mpc_vel_vec[1];
  
    // Get all the mpc velocities
    for (int i = 0; i < mpc_res["vel"].columns(); i++) {
      const auto& vel_i = mpc_res["vel"](casadi::Slice(), i).get_elements();
      mpc_velocities.emplace_back(vel_i[0], vel_i[1]);
      CLOG(DEBUG, "cbit.control")
          << "MPC velocity at step " << i << ": " << mpc_velocities.back().transpose();
    }
  
  } catch (std::exception& e) {
    CLOG(WARNING, "cbit.control")
        << "casadi failed! " << e.what() << " Commanding to Stop the Vehicle";
    return Command();
  }

  vis_->publishMPCRollout(mpc_poses);

  CLOG(INFO, "cbit.control") << "The linear velocity is:  " << command.linear.x
                             << " The angular vel is: " << command.angular.z;

  return command;
}

void BaseMPCPathTracker::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& ) {

  auto& chain = robot_state.chain.ptr();
  std::vector<double> p_rollout;
  for (int j = 1; j < mpcConfig->N + 1; j++) {
    p_rollout.push_back(state_p + j * mpcConfig->VF * mpcConfig->DT);
  }

  mpcConfig->reference_poses.clear();
  auto referenceInfo = generateHomotopyReference(p_rollout, chain, T_w_p*T_p_r_extp);
  for (const auto& Tf : referenceInfo.poses) {
    mpcConfig->reference_poses.push_back(tf_to_global(T_w_p.inverse() * Tf));
    CLOG(DEBUG, "cbit.control")
        << "Adding reference pose: " << tf_to_global(T_w_p.inverse() * Tf);
  }

  // Detect end of path and set the corresponding cost weight vector element to zero
  mpcConfig->cost_weights.clear();
  mpcConfig->cost_weights.push_back(1.0);
  auto last_pose = (T_w_p.inverse()*referenceInfo.poses[0]).vec();
  int end_ind = -1;
  auto weighting = 1.0;
  for (int i = 1; i < mpcConfig->N; i++) {
    auto curr_pose = (T_w_p.inverse() * referenceInfo.poses[i]).vec();
    auto dist = (curr_pose - last_pose).norm();
    if (end_ind < 0 && dist < base_config_->end_of_path_distance_threshold) {
      end_ind = i-1;
      weighting = (float)  1.0 / (mpcConfig->N - end_ind);
      CLOG(DEBUG, "cbit.control") << "Detected end of path. Setting cost of EoP poses to: " << weighting;
    }
    else if (end_ind >= 0 && dist > base_config_->end_of_path_distance_threshold) {
      weighting = 1.0;
      for (int j = 0; j < i; j++) {
        mpcConfig->cost_weights[j] = weighting;
      }
      end_ind = -1;
      CLOG(DEBUG, "cbit.control") << "False end of path. Setting cost of EoP poses to: " << weighting;
    }

    mpcConfig->cost_weights.push_back(weighting);
    last_pose = curr_pose;
  }

  vis_->publishReferencePoses(referenceInfo.poses);
  mpcConfig->eop_index = end_ind;

  mpcConfig->up_barrier_q  = referenceInfo.barrier_q_max;
  mpcConfig->low_barrier_q = referenceInfo.barrier_q_min;
}
}  // namespace vtr::path_planning
