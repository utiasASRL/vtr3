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
 * \file safety_monitor.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_safety_monitor/safety_monitor/safety_monitor.hpp>

namespace vtr {
namespace safety_monitor {

SafetyMonitor::SafetyMonitor(const rclcpp::Node::SharedPtr &node)
    : node_(node) {
  // clang-format off
  monitor_update_period_ = node_->declare_parameter<int>("monitor_update_period", 1000);
  monitor_names_ = node_->declare_parameter<std::vector<std::string>>("list_of_monitors", std::vector<std::string>{});
  absolute_max_speed_ = node_->declare_parameter<double>("max_allowed_speed", 5.0);
  // clang-format on
  CLOG(INFO, "safety_montor") << "Enabled monitors: " << monitor_names_;
  for (auto &monitor : monitor_names_)
    monitors_.push_back(createMonitor(monitor));

  // clang-format off
  safety_status_timer_ = node_->create_wall_timer(std::chrono::milliseconds(monitor_update_period_), std::bind(&SafetyMonitor::getSafetyStatus, this));
  /// use a separate thread for command filtering
  auto cmd_sub_opt = rclcpp::SubscriptionOptions();
  cmd_sub_opt.callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  command_sub_ = node_->create_subscription<TwistMsg>("command", rclcpp::SystemDefaultsQoS(), std::bind(&SafetyMonitor::processCommand, this, std::placeholders::_1), cmd_sub_opt);
  command_pub_ = node_->create_publisher<TwistMsg>("safe_command", 1);
  // clang-format on
}

std::unique_ptr<BaseMonitor> SafetyMonitor::createMonitor(
    const std::string &name) {
  if (name == "deadman_monitor")
    return std::make_unique<DeadmanMonitor>(node_);
  else if (name == "heartbeat_monitor")
    return std::make_unique<HeartbeatMonitor>(node_);
  else {
    std::string err{"Cannot find the specified monitor: " + name};
    CLOG(DEBUG, "safety_monitor") << err;
    throw std::invalid_argument(err);
  }
  return nullptr;
}

void SafetyMonitor::getSafetyStatus() {
  // Initialize values
  int desired_action = CONTINUE;
  double speed_limit = absolute_max_speed_;
  std::vector<std::string> limiting_signal_monitor_names;
  std::vector<int> limiting_signal_monitor_actions;

  // Go through all monitors and signals to determine highest priority behavior
  // and speed limit
  for (auto &monitor : monitors_)
    monitor->updateSafetyMonitorAction(desired_action, speed_limit,
                                       limiting_signal_monitor_names,
                                       limiting_signal_monitor_actions);

  // Log
  // clang-format off
  CLOG(INFO, "safety_monitor") << "=== Safety Status Update ===";
  CLOG(INFO, "safety_monitor") << "Desired action: " << ACTION_STRINGS[desired_action];
  CLOG(INFO, "safety_monitor") << "Safety layer speed limit: " << speed_limit;
  CLOG(INFO, "safety_monitor") << "Limiting signal monitor names: " << limiting_signal_monitor_names;
  CLOG(INFO, "safety_monitor") << "Limiting signal monitor actions: " << limiting_signal_monitor_actions;
  CLOG(INFO, "safety_monitor") << "=== ==================== ===";
  // clang-format on

  // update command
  std::lock_guard<std::mutex> lock(status_mutex_);
  speed_limit_ = speed_limit;
  desired_action_ = desired_action;
}

void SafetyMonitor::processCommand(const TwistMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  /// \todo also check speed_limit_
  if (desired_action_ == CONTINUE) {
    command_pub_->publish(*msg);
  }
}

}  // namespace safety_monitor
}  // namespace vtr