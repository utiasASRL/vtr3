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
  
  // MPC COST PARAMETERS
  config->q_x = node->declare_parameter<double>(prefix + ".mpc.q_x", config->q_x);
  config->q_y = node->declare_parameter<double>(prefix + ".mpc.q_y", config->q_y);
  config->q_th = node->declare_parameter<double>(prefix + ".mpc.q_th", config->q_th);
  config->r1 = node->declare_parameter<double>(prefix + ".mpc.r1", config->r1);
  config->r2 = node->declare_parameter<double>(prefix + ".mpc.r2", config->r2);
  config->racc1 = node->declare_parameter<double>(prefix + ".mpc.racc1", config->racc1);
  config->racc2 = node->declare_parameter<double>(prefix + ".mpc.racc2", config->racc2);
  config->q_f = node->declare_parameter<double>(prefix + ".mpc.q_f", config->q_f);
  config->repeat_flipped = node->declare_parameter<bool>(prefix + ".mpc.repeat_flipped", config->repeat_flipped);
  CLOG(INFO, "cbit.control") << "The config is: Q_x " << config->q_x << " q_y: " << config->q_y<< " q_th: " << config->q_th<< " r1: " << config->r1<< " r2: " << config->r2<< "acc_r1: " << config->racc1<< " acc_r2: " << config->racc2;


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

  if (isReversing) {
    solver_.setReversing(true);
  }

  // Set the MPC parameters based on the configuration
  mpc_config->VF = config_->forward_vel;
  mpc_config->vel_max(0) = config_->max_lin_vel;
  mpc_config->vel_max(1) = config_->max_ang_vel;

  // Set the MPC costs
  mpc_config->Q_x = config_->q_x;
  mpc_config->Q_y = config_->q_y;
  mpc_config->Q_th = config_->q_th;
  mpc_config->R1 = config_->r1;
  mpc_config->R2 = config_->r2;
  mpc_config->Acc_R1 = config_->racc1;
  mpc_config->Acc_R2 = config_->racc2;
  mpc_config->Q_f = config_->q_f;
  mpc_config->previous_vel = {-w_p_r_in_r(0, 0), applied_vel(1)};

  return mpc_config;
}

casadi::DMDict BicycleMPCPathTracker::callSolver(CasadiMPC::Config::Ptr config) {
  return solver_.solve(*config);
}

}  // namespace vtr::path_planning
