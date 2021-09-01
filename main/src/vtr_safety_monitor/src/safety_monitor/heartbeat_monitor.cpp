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
 * \file heartbeat_monitor.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_logging/logging.hpp>
#include <vtr_safety_monitor/safety_monitor/heartbeat_monitor.hpp>

namespace vtr {
namespace safety_monitor {

HeartbeatMonitor::HeartbeatMonitor(const std::shared_ptr<rclcpp::Node>& node)
    : BaseMonitor(node) {
  // Initialize heartbeat monitor.
  monitor_signals_.emplace_back(node_, "Heartbeat Monitor");

  // Initialize message subscriptions
  // clang-format off
  status_subscriber_ = node_->create_subscription<RobotStatusMsg>("robot", 10, std::bind(&HeartbeatMonitor::statusCallback, this, std::placeholders::_1));
  period_ = node_->declare_parameter<int>("heartbeat_period", 2000);
  timer_ = rclcpp::create_timer(node_, node_->get_clock(), std::chrono::milliseconds(period_), std::bind(&HeartbeatMonitor::timedCallback, this));
  // clang-format on
}

void HeartbeatMonitor::statusCallback(const RobotStatusMsg::SharedPtr status) {
  if (status->state == "::Repeat::Follow") {
    monitor_signals_.front().desired_action = CONTINUE;
    following_ = true;
    CLOG(DEBUG, "safety_monitor")
        << "Heartbeat monitor set action to CONTINUE.";
  } else {
    following_ = false;
  }
  timer_->reset();
}

void HeartbeatMonitor::timedCallback() {
  if (following_) {
    // only pause if we expect to be moving and therefore getting status updates
    monitor_signals_.front().desired_action = PAUSE;
    CLOG(INFO, "safety_monitor")
        << "Heartbeat monitor has not received a robot status for at least "
        << period_ << " seconds. Setting action to PAUSE";
  } else {
    monitor_signals_.front().desired_action = CONTINUE;
    CLOG(DEBUG, "safety_monitor")
        << "Timed callback not triggering pause because not in Follow.";
  }
}

}  // namespace safety_monitor
}  // namespace vtr
