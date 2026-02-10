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
 * \file cbit.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/unicycle_mpc_path_tracker.hpp"

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

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto UnicycleMPCPathTracker::Config::loadConfig(UnicycleMPCPathTracker::Config::Ptr config, 
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix)->void{
  // MPC Configs:
  // SPEED SCHEDULER PARAMETERS
  config->q_x   = node->declare_parameter<double>(prefix + ".mpc.q_x", config->q_x);
  config->q_y   = node->declare_parameter<double>(prefix + ".mpc.q_y", config->q_y);
  config->q_th  = node->declare_parameter<double>(prefix + ".mpc.q_th", config->q_th);
  config->r1    = node->declare_parameter<double>(prefix + ".mpc.r1", config->r1);
  config->r2    = node->declare_parameter<double>(prefix + ".mpc.r2", config->r2);
  config->racc1 = node->declare_parameter<double>(prefix + ".mpc.racc1", config->racc1);
  config->racc2 = node->declare_parameter<double>(prefix + ".mpc.racc2", config->racc2);
}

auto UnicycleMPCPathTracker::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  auto base_config = std::static_pointer_cast<BaseMPCPathTracker::Config>(config);
  *base_config =  *BaseMPCPathTracker::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);

  CLOG(DEBUG, "cbit.control") << "Unicycle MPC parameters: "
      << "q_x: " << config->q_x
      << ", q_y: " << config->q_y
      << ", q_th: " << config->q_th
      << ", r1: " << config->r1
      << ", r2: " << config->r2
      << ", racc1: " << config->racc1
      << ", racc2: " << config->racc2
      << ", alpha: " << config->alpha;

  return config;
}

// Declare class as inherited from the BaseMPCPathTracker 
UnicycleMPCPathTracker::UnicycleMPCPathTracker(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph,
                               const Callback::Ptr& callback)
    : BaseMPCPathTracker(config, robot_state, graph, callback), 
    config_(config), 
    solver_{config_->mpc_verbosity}{
      CLOG(DEBUG, "cbit.control") << "Constructed Unicycle tracker";
}

UnicycleMPCPathTracker::~UnicycleMPCPathTracker() {}

void UnicycleMPCPathTracker::loadMPCConfig(
    CasadiUnicycleMPC::Config::Ptr mpc_config, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) {
  // Set the MPC parameters based on the configuration
  mpc_config->VF           = config_->forward_vel;
  mpc_config->vel_max(0)   = config_->max_lin_vel;
  mpc_config->vel_min(0)   = -config_->max_lin_vel;
  mpc_config->vel_max(1)   = config_->max_ang_vel;
  mpc_config->vel_min(1)   = -config_->max_ang_vel;
  mpc_config->previous_vel = {-w_p_r_in_r(0, 0), -w_p_r_in_r(5,0)};

  // Set the MPC costs based on if we're reversing or not
  mpc_config->Q_x    = config_->q_x;
  mpc_config->Q_y    = config_->q_y;
  mpc_config->Q_th   = config_->q_th;
  mpc_config->R1     = config_->r1; 
  mpc_config->R2     = config_->r2;
  mpc_config->Acc_R1 = config_->racc1;
  mpc_config->Acc_R2 = config_->racc2; 
}

CasadiMPC::Config::Ptr UnicycleMPCPathTracker::getMPCConfig(Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel, const bool) {
  auto mpc_config = std::make_shared<CasadiUnicycleMPC::Config>();
  loadMPCConfig(mpc_config, w_p_r_in_r, applied_vel);
  return mpc_config;
}

bool UnicycleMPCPathTracker::isMPCStateValid(CasadiMPC::Config::Ptr, const tactic::Timestamp& ){
  return true;
}

void UnicycleMPCPathTracker::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& t) {
  auto mpc_config = std::static_pointer_cast<CasadiUnicycleMPC::Config>(mpcConfig);
  BaseMPCPathTracker::loadMPCPath(mpcConfig, T_w_p, T_p_r_extp, state_p, robot_state, t);
}

std::map<std::string, casadi::DM> UnicycleMPCPathTracker::callSolver(CasadiMPC::Config::Ptr config) {
  std::map<std::string, casadi::DM> result;
  CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
  result = solver_.solve(*config);
  CLOG(INFO, "cbit.control") << "Solver called";
  return result;
}

}  // namespace path_planning
}  // namespace vtr