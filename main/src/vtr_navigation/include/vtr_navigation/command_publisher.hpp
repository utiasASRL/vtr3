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
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "vtr_path_planning/base_path_planner.hpp"

namespace vtr {
namespace navigation {

class CommandPublisher : public path_planning::BasePathPlanner::Callback {
 public:
  PTR_TYPEDEFS(CommandPublisher);

  using ROSCommand = geometry_msgs::msg::TwistStamped;

  CommandPublisher(const rclcpp::Node::SharedPtr& node);

  void commandReceived(const Command& command) override;

 private:
  rclcpp::Publisher<ROSCommand>::SharedPtr command_pub_;
};

}  // namespace navigation
}  // namespace vtr