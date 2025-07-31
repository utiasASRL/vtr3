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

auto BicycleMPCPathTracker::Config::loadConfig(BicycleMPCPathTracker::Config::Ptr config, 
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix)->void{
  config->failure_threshold = node->declare_parameter<int>(prefix + ".mpc.failure_threshold", config->failure_threshold);
  config->recovery_steps = node->declare_parameter<int>(prefix + ".mpc.recovery_steps", config->recovery_steps);
  config->wheelbase = node->declare_parameter<double>(prefix + ".mpc.wheelbase", config->wheelbase);
  
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
}

auto BicycleMPCPathTracker::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  auto base_config = std::static_pointer_cast<BaseMPCPathTracker::Config>(config);
  *base_config =  *BaseMPCPathTracker::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);

  CLOG(DEBUG, "cbit.control") << "Bicycle MPC forward costs: "
      << "q_lat: " << config->f_q_lat
      << ", q_lon: " << config->f_q_lon
      << ", q_th: " << config->f_q_th
      << ", r1: " << config->f_r1
      << ", r2: " << config->f_r2
      << ", racc1: " << config->f_racc1
      << ", racc2: " << config->f_racc2
      << ", q_f: " << config->f_q_f;

  CLOG(DEBUG, "cbit.control") << "Bicycle MPC reverse costs: "
      << "q_lat: " << config->r_q_lat
      << ", q_lon: " << config->r_q_lon
      << ", q_th: " << config->r_q_th
      << ", r1: " << config->r_r1
      << ", r2: " << config->r_r2
      << ", racc1: " << config->r_racc1
      << ", racc2: " << config->r_racc2
      << ", q_f: " << config->r_q_f;

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

void BicycleMPCPathTracker::loadMPCConfig(
    CasadiBicycleMPC::Config::Ptr mpc_config, const bool isReversing, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) {
  // Set the MPC parameters based on the configuration
  mpc_config->VF = config_->forward_vel;
  mpc_config->vel_max(0) = isReversing ? 0.0 : config_->max_lin_vel;
  mpc_config->vel_min(0) = isReversing ? -config_->max_lin_vel : 0.0;
  mpc_config->vel_max(1) = config_->max_ang_vel;
  mpc_config->vel_min(1) = -config_->max_ang_vel;
  mpc_config->previous_vel = {-w_p_r_in_r(0, 0), applied_vel(1)};
  mpc_config->wheelbase = config_->wheelbase;

  // Set the MPC costs base on if we're reversing or not
  mpc_config->Q_lat = isReversing ? config_->r_q_lat : config_->f_q_lat;
  mpc_config->Q_lon = isReversing ? config_->r_q_lon : config_->f_q_lon;
  mpc_config->Q_th = isReversing ? config_->r_q_th : config_->f_q_th;
  mpc_config->R1 = isReversing ? config_->r_r1 : config_->f_r1; 
  mpc_config->R2 = isReversing ? config_->r_r2 : config_->f_r2;
  mpc_config->Acc_R1 = isReversing ? config_->r_racc1 : config_->f_racc1;
  mpc_config->Acc_R2 = isReversing ? config_->r_racc2 : config_->f_racc2; 
  mpc_config->Q_f = isReversing ? config_->r_q_f : config_->f_q_f;

  if (failure_count >= config_->failure_threshold) {
    CLOG(WARNING, "cbit.control") << "Failure count exceeded threshold. Enabling recovery mode.";
    mpc_config->recovery = true;
    // If we're recovering, the only thing that matters is getting back to the path
    mpc_config->Q_lon = 1.0;
    // TODO: make these parameters configurable
    mpc_config->vel_max(0) = 0.5*config_->max_lin_vel;
    mpc_config->R1 = 0.0;
    mpc_config->R2 = 0.0;
    mpc_config->Acc_R1 = 0.0;
    mpc_config->Acc_R2 = 0.0;
    mpc_config->Q_f = 0.0;
  }
}

CasadiMPC::Config::Ptr BicycleMPCPathTracker::getMPCConfig(const bool isReversing,  Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) {
  auto mpc_config = std::make_shared<CasadiBicycleMPC::Config>();
  loadMPCConfig(mpc_config, isReversing, w_p_r_in_r, applied_vel);
  return mpc_config;
}

bool BicycleMPCPathTracker::isMPCStateValid(CasadiMPC::Config::Ptr, const tactic::Timestamp& ){
  return true;
}

std::map<std::string, casadi::DM> BicycleMPCPathTracker::callSolver(CasadiMPC::Config::Ptr config) {
  std::map<std::string, casadi::DM> result;

  try {
    CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
    result = solver_.solve(*config);
    CLOG(INFO, "cbit.control") << "Solver called";
    success_count += 1;
    if(success_count > config_->recovery_steps){
        failure_count = 0;
    }
    
  } catch (std::exception& e) {
      CLOG(WARNING, "cbit.control")
          << "casadi failed! " << e.what() << ". Incrementing failure count. Current count: " << failure_count;
      failure_count += 1;
      // Only reset success count if we are not actively trying to recover
      // If we are failing consistently when recovering, we should just keep trying.
      if (failure_count < config_->failure_threshold){
          success_count = 0;
      }
      throw e;
  }
  return result;
}

}  // namespace vtr::path_planning