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
 * \file base_monitor.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "vtr_logging/logging.hpp"

namespace vtr {
namespace safety_monitor {

enum class Action : int8_t {
  Continue = 0,
  Pause = 1,
};

std::ostream &operator<<(std::ostream &os, const Action &action);

class BaseMonitor {
 public:
  /** \brief  */
  BaseMonitor(const rclcpp::Node::SharedPtr &node,
              const std::string &name = "Anonymous Monitor",
              const double &timeout = std::numeric_limits<double>::max())
      : node_(node),
        name_(name),
        timeout_(timeout),
        last_update_(node->now()) {}

  /** \brief  */
  void updateSafetyMonitorAction(
      Action &desired_action, double &speed_limit,
      std::vector<std::string> &limiting_signal_monitor_names,
      std::vector<Action> &limiting_signal_monitor_actions);

  virtual ~BaseMonitor() = default;

 protected:
  /** \brief ROS-handle for communication */
  const rclcpp::Node::SharedPtr node_;

  /// parameters
  /** \brief  */
  const std::string name_;
  /** \brief  */
  const double timeout_;

  /// variables
  /** \brief  */
  rclcpp::Time last_update_;
  /** \brief  */
  Action desired_action_ = Action::Pause;
  /** \brief  */
  double speed_limit_ = std::numeric_limits<double>::max();
};

}  // namespace safety_monitor
}  // namespace vtr
