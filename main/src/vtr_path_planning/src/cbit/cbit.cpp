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
#include "vtr_path_planning/cbit/cbit.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

namespace vtr {
namespace path_planning {

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto CBIT::Config::fromROS(const rclcpp::Node::SharedPtr& node,
                                     const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  // Originally I thought I could get rid of this, but it is actually very important with how the codebase is setup.
  // The config class above is apart of the Base Planner class, which declares the default control period as 0. So if we dont update it, we have alot of problems
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);

  // robot configuration
  config->robot_model = node->declare_parameter<std::string>(prefix + ".teb.robot_model", config->robot_model);
  config->robot_radius = node->declare_parameter<double>(prefix + ".teb.robot_radius", config->robot_radius);

  // This is how parameters should be updated from the parameter server. prefix is hardcoded to path_planning in the header, tabs are represented by periods "."
  config->obs_padding = node->declare_parameter<double>(prefix + ".cbit.obs_padding", config->obs_padding);

  // Removed alot of the other config stuff here, I think it was specifically for dynamic reconfigure, which atleast for cbit, I dont think should be dynamically reconfigurable for now
  return config;
}

// Declare class as inherited from the BasePathPlanner
CBIT::CBIT(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback), config_(config) {
  const auto node = robot_state->node.ptr();
}

CBIT::~CBIT() { stop(); }


// Here is where we can do all the teach path pre-processing and then begin the anytime planner asychronously
void CBIT::initializeRoute(RobotState& robot_state) {
  /// \todo reset any internal state
  CLOG(INFO, "path_planning.teb") << "Path Planner has been started, here is where we will begin teach path pre-processing and asychronous cbit";
  CLOG(INFO, "path_planning.teb") << "The obs_padding config param from ROS is: " << config_->obs_padding; // This is how you would reference a param, need to use config_ not config!!!

  auto& chain = *robot_state.chain;

  // Here is an example for getting the teach path frames, use chain.pose(<frame_index>) where 0 is the first frame
  // I think there is a chain.size() function you can use to get the total number of frames in the teach path we are trying to repeat.
  //CLOG(INFO, "path_planning.teb") << "Key Frame 1 " << chain.pose(0);
  //CLOG(INFO, "path_planning.teb") << "Key Frame 2 " << chain.pose(1);

  
  /*
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_p_g, T_p_i_vec, curr_sid] =
      chain_info;

  CLOG(INFO, "path_planning.teb") << "stamp " << stamp;
  CLOG(INFO, "path_planning.teb") << "w_p_r_in_r: " << w_p_r_in_r;
  CLOG(INFO, "path_planning.teb") << "T_p_r: " << T_p_r;
  CLOG(INFO, "path_planning.teb") << "T_w_p: " << T_w_p;
  */
}


// Called at a the control rate in base_planner, eventually we need to do the mpc here using the output of cbit as constraints
auto CBIT::computeCommand(RobotState& robot_state) -> Command {
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "path_planning.teb")
        << "Robot is not localized, command to stop the robot test teb";
    return Command();
  }

  else {
    CLOG(INFO, "path_planning.teb") << "Robot is now localized and we can start doing things in cbit";
  }

  // retrieve info from the localization chain
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid] = chain_info;
  CLOG(INFO, "path_planning.teb") << "stamp " << stamp;
  CLOG(INFO, "path_planning.teb") << "w_p_r_in_r: " << w_p_r_in_r;
  CLOG(INFO, "path_planning.teb") << "T_p_r: " << T_p_r;
  CLOG(INFO, "path_planning.teb") << "T_w_p: " << T_w_p;

  return Command(); // This returns the stop command when called with no arguments
}


// Function for grabbing the robots velocity in planning frame, transform of robot into planning frame, and transform of planning frame to world frame
// Modified it to only give this info, I think the other things were only really teb relevant for now
auto CBIT::getChainInfo(RobotState& robot_state) -> ChainInfo {
  auto& chain = *robot_state.chain;
  auto lock = chain.guard();
  const auto stamp = chain.leaf_stamp();
  const auto w_p_r_in_r = chain.leaf_velocity();
  const auto T_p_r = chain.T_leaf_trunk().inverse();
  const auto T_w_p = chain.T_start_trunk();
  const auto curr_sid = chain.trunkSequenceId();
  return ChainInfo{stamp, w_p_r_in_r, T_p_r, T_w_p, curr_sid};
}


}  // namespace path_planning
}  // namespace vtr