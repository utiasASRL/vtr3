// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file stationary_planner.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_planning/stationary_planner.hpp>

namespace vtr {
namespace path_planning {

auto StationaryPlanner::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) -> Ptr {

  auto config = std::make_shared<StationaryPlanner::Config>();
  auto casted_config =
      std::static_pointer_cast<BasePathPlanner::Config>(config);
  *casted_config = *BasePathPlanner::Config::fromROS(node, param_prefix);  // copy over base config

  return config;
}

// Declare class as inherited from the BasePathPlanner
StationaryPlanner::StationaryPlanner(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback), config_(config) {}

StationaryPlanner::~StationaryPlanner() { stop(); }

StationaryPlanner::Command StationaryPlanner::computeCommand(RobotState&) { 
    return Command();
}

} //path_planning
} //vtr
