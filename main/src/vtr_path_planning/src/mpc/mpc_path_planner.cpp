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
 * \file mpc_path_planner.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/mpc/mpc_path_planner.hpp"

namespace vtr {
namespace path_planning {

auto MPCPathPlanner::Config::fromROS(const rclcpp::Node::SharedPtr& node,
                                     const std::string& prefix) -> UniquePtr {
  auto config = std::make_unique<Config>();
  // clang-format off
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);
  // clang-format on

  return config;
}

MPCPathPlanner::MPCPathPlanner(Config::UniquePtr config,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config->control_period, callback),
      config_(std::move(config)) {}

MPCPathPlanner::~MPCPathPlanner() { stop(); }

auto MPCPathPlanner::computeCommand() -> Command {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return Command();
}

}  // namespace path_planning
}  // namespace vtr