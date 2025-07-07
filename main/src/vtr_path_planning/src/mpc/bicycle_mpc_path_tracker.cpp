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

#include "vtr_path_planning/mpc/bicycle_mpc_path_tracker.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

namespace vtr::path_planning {

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto BicycleMPCPathTracker::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  auto base_config = std::static_pointer_cast<BasePathPlanner::Config>(config);
  *base_config =  *BasePathPlanner::Config::fromROS(node, prefix);

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
  
  // MPC COST PARAMETERS
  // We have one set for reverse, and one for forward, driving
  config->f_q_lat = node->declare_parameter<double>(prefix + ".mpc.forward.q_lat", config->f_q_lat);
  config->f_q_lon = node->declare_parameter<double>(prefix + ".mpc.forward.q_lon", config->f_q_lon);
  config->f_q_th = node->declare_parameter<double>(prefix + ".mpc.forward.q_th", config->f_q_th);
  config->f_r1 = node->declare_parameter<double>(prefix + ".mpc.forward.r1", config->f_r1);
  config->f_r2 = node->declare_parameter<double>(prefix + ".mpc.forward.r2", config->f_r2);
  config->f_racc1 = node->declare_parameter<double>(prefix + ".mpc.forward.racc1", config->f_racc1);
  config->f_racc2 = node->declare_parameter<double>(prefix + ".mpc.forward.racc2", config->f_racc2);
  config->f_q_f = node->declare_parameter<double>(prefix + ".mpc.forward.q_f", config->f_q_f);

  // We have one set for reverse, and one for forward, driving
  config->r_q_lat = node->declare_parameter<double>(prefix + ".mpc.reverse.q_lat", config->r_q_lat);
  config->r_q_lon = node->declare_parameter<double>(prefix + ".mpc.reverse.q_lon", config->r_q_lon);
  config->r_q_th = node->declare_parameter<double>(prefix + ".mpc.reverse.q_th", config->r_q_th);
  config->r_r1 = node->declare_parameter<double>(prefix + ".mpc.reverse.r1", config->r_r1);
  config->r_r2 = node->declare_parameter<double>(prefix + ".mpc.reverse.r2", config->r_r2);
  config->r_racc1 = node->declare_parameter<double>(prefix + ".mpc.reverse.racc1", config->r_racc1);
  config->r_racc2 = node->declare_parameter<double>(prefix + ".mpc.reverse.racc2", config->r_racc2);
  config->r_q_f = node->declare_parameter<double>(prefix + ".mpc.reverse.q_f", config->r_q_f);

  // MISC
  config->command_history_length = node->declare_parameter<int>(prefix + ".mpc.command_history_length", config->command_history_length);

  return config;
}

// Declare class as inherited from the BasePathPlanner
BicycleMPCPathTracker::BicycleMPCPathTracker(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph,
                               const Callback::Ptr& callback)
    : BaseMPCPathTracker(config, robot_state, graph, callback), 
    config_(config), 
    solver_{config_->mpc_verbosity} {
      CLOG(DEBUG, "cbit.control") << "Constructed Bicycle tracker";
}

BicycleMPCPathTracker::~BicycleMPCPathTracker() {}

CasadiMPC::Config::Ptr BicycleMPCPathTracker::loadMPCConfig(const bool isReversing,  Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) {
  auto mpc_config = std::make_shared<CasadiBicycleMPC::Config>();

  // Set the MPC parameters based on the configuration
  mpc_config->VF = config_->forward_vel;
  mpc_config->vel_max(0) = config_->max_lin_vel;
  mpc_config->vel_max(1) = config_->max_ang_vel;
  mpc_config->previous_vel = {-w_p_r_in_r(0, 0), applied_vel(1)};

  if (isReversing) {
    // Set the MPC costs
    mpc_config->Q_lat = config_->r_q_lat;
    mpc_config->Q_lon = config_->r_q_lon;
    mpc_config->Q_th = config_->r_q_th;
    mpc_config->R1 = config_->r_r1;
    mpc_config->R2 = config_->r_r2;
    mpc_config->Acc_R1 = config_->r_racc1;
    mpc_config->Acc_R2 = config_->r_racc2;
    mpc_config->Q_f = config_->r_q_f;
    mpc_config->reversing = true;
  }
  else {
    // Set the MPC costs
    mpc_config->Q_lat = config_->f_q_lat;
    mpc_config->Q_lon = config_->f_q_lon;
    mpc_config->Q_th = config_->f_q_th;
    mpc_config->R1 = config_->f_r1;
    mpc_config->R2 = config_->f_r2;
    mpc_config->Acc_R1 = config_->f_racc1;
    mpc_config->Acc_R2 = config_->f_racc2;
    mpc_config->Q_f = config_->f_q_f;
    mpc_config->reversing = false;
  }

  return mpc_config;
}

casadi::DMDict BicycleMPCPathTracker::callSolver(CasadiMPC::Config::Ptr config) {
  return solver_.solve(*config);
}

}  // namespace vtr::path_planning
