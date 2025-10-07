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
 * \file dynamic_path_planner.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_planning/dynamic_path_planner.hpp>

namespace vtr {
namespace path_planning {

auto DynamicPathPlanner::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) -> Ptr {

  auto config = std::make_shared<DynamicPathPlanner::Config>();
  auto casted_config =
      std::static_pointer_cast<BasePathPlanner::Config>(config);
  *casted_config = *BasePathPlanner::Config::fromROS(node, param_prefix);  // copy over base config
  config->planner_whitelist = node->declare_parameter<std::vector<std::string>>(param_prefix + ".planner_whitelist", config->planner_whitelist);
  config->default_planner = node->declare_parameter<std::string>(param_prefix + ".default_type", config->default_planner);
  return config;
}

// Declare class as inherited from the BasePathPlanner
DynamicPathPlanner::DynamicPathPlanner(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph, 
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, graph, callback), 
        config_(config), robot_state_(robot_state), graph_(graph), callback_(callback) {
    factory_ = std::make_shared<ROSPathPlannerFactory>(robot_state->node.ptr());
    active_planner_ = factory_->get("path_planning." + config->default_planner, robot_state_, graph_, callback_);
    controllers_used_.emplace(config->default_planner);
}

DynamicPathPlanner::~DynamicPathPlanner() { stop(); }

void DynamicPathPlanner::setRunning(const bool running) {
    active_planner_->setRunning(running);
}

void DynamicPathPlanner::setController(const std::string& new_controller) {
    active_planner_->setRunning(false);
    auto it = std::find(config_->planner_whitelist.begin(), config_->planner_whitelist.end(), new_controller);

    // Check if the element was found
    if (config_->planner_whitelist.size() == 0 || it != config_->planner_whitelist.end()) {
        try {
            active_planner_ = factory_->get("path_planning." + new_controller, robot_state_, graph_, callback_);
        } catch (std::runtime_error& e) {
            CLOG(ERROR, DynamicPathPlanner::static_name) << "Requested path planner " << new_controller << " but param path_planning." << new_controller << ".type was unset!";
        }
    } else {
        CLOG(WARNING, DynamicPathPlanner::static_name) << "Requested path planner " << new_controller << " which was not in the white list";
    }
    active_planner_->setRunning(running_);
}

DynamicPathPlanner::Command DynamicPathPlanner::computeCommand(RobotState& r) { 
    return active_planner_->computeCommand(r);
}

void DynamicPathPlanner::stop() {
    for (const auto& c_type : controllers_used_) {
        factory_->get("path_planning." + c_type, robot_state_, graph_, callback_)->stop();
    }
    BasePathPlanner::stop();
}

} //path_planning
} //vtr
