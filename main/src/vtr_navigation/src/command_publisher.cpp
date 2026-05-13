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
 * \file command_publisher.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_navigation/command_publisher.hpp"

namespace vtr {
namespace navigation {

CommandPublisher::CommandPublisher(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
  command_pub_ = node->create_publisher<Command>("command", 10);
}

tactic::Timestamp CommandPublisher::getCurrentTime() const {
  return node_->now().nanoseconds();
}

void CommandPublisher::commandReceived(const Command& command) {
  CLOG(DEBUG, "navigation.command")
      << "Received control command: [" << command.linear.x << ", "
      << command.linear.y << ", " << command.linear.z << ", "
      << command.angular.x << ", " << command.angular.y << ", "
      << command.angular.z << "]";
  command_pub_->publish(command);
}

}  // namespace navigation
}  // namespace vtr