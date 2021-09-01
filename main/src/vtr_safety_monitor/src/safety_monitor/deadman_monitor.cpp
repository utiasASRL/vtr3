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
 * \file deadman_monitor.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_safety_monitor/safety_monitor/deadman_monitor.hpp>

namespace vtr {
namespace safety_monitor {

const int DeadmanMonitor::axis_array[] = {7, 7, 7, 7, 7, 7, 7, 7,
                                          6, 6, 6, 6, 6, 6, 6, 6};
const int DeadmanMonitor::value_array[] = {1, 0, 1,  0, -1, 0, -1, 0,
                                           1, 0, -1, 0, 1,  0, -1, 0};

DeadmanMonitor::DeadmanMonitor(const std::shared_ptr<rclcpp::Node>& node)
    : BaseMonitor(node) {
  // Initialize DEADMAN monitor signal
  monitor_signals_.emplace_back(node_, "Deadman Monitor", 1);

  // Initialize Message Subscriptions
  // clang-format off
  gamepad_subscriber_ = node_->create_subscription<JoyMsg>("/xbox_joy", 10, std::bind(&DeadmanMonitor::gamepadCallback, this, std::placeholders::_1));
  deadman_button_index_ = node_->declare_parameter<int>("deadman_button_index", 1);
  // clang-format on

  // Initialize the gamepad callback
  time_of_last_code_sequence_ = node_->now();
}

void DeadmanMonitor::gamepadCallback(const JoyMsg::SharedPtr msg) {
  monitor_signals_.front().last_update = node_->now();

  bool pressed = msg->buttons[deadman_button_index_] != 0;

  if (monitor_signals_.front().desired_action == PAUSE) {
    if (following_mode_ == FOLLOWING_MODE::AUTO || pressed)
      monitor_signals_.front().desired_action = CONTINUE;

    // check the cheat code state
    checkCodeState(msg);

    // if the cheat code has been activated, put the path tracker into auto mode
    if (code_state_ == CHEAT_CODE_ACTIVATED) {
      CLOG(INFO, "safety_monitor")
          << "CHEAT CODE ACTIVATED, SWITCHING TO AUTO MODE";
      following_mode_ = FOLLOWING_MODE::AUTO;
    }
  } else if (monitor_signals_.front().desired_action == CONTINUE) {
    if (following_mode_ == FOLLOWING_MODE::AUTO) {
      // Check if we need to disable FOLLOWING_MODE::AUTO mode
      for (int button : msg->buttons) {
        rclcpp::Duration diff =
            rclcpp::Clock().now() - time_of_last_code_sequence_;

        if (button == 1 && diff.seconds() > 1.0) {
          CLOG(INFO, "safety_monitor")
              << "CHEAT CODE DEACTIVATED, SWITCHING TO MANUAL MODE";
          following_mode_ = FOLLOWING_MODE::MANUAL;
          code_state_ = 0;
        }
      }

      // Now that we've checked the cheat code and maybe changed
      // following_mode_...
      if (following_mode_ == FOLLOWING_MODE::MANUAL && !pressed)
        monitor_signals_.front().desired_action = PAUSE;
    } else if (!pressed) {
      monitor_signals_.front().desired_action = PAUSE;
    }
  } else {
    monitor_signals_.front().desired_action = PAUSE;
  }
}

void DeadmanMonitor::checkCodeState(const JoyMsg::SharedPtr& msg) {
  const auto diff = node_->now() - time_of_last_code_sequence_;
  CLOG(DEBUG, "safety_monitor")
      << "Time since last button: " << diff.seconds() << " \n";

  if (diff.seconds() > 0.75) {
    code_state_ = 0;
    CLOG(DEBUG, "safety_monitor")
        << "Too long since button press, code_state_ reset.";
  }
  if (code_state_ < 16) {
    if (msg->axes[axis_array[code_state_]] == value_array[code_state_]) {
      code_state_++;
      time_of_last_code_sequence_ = node_->now();
    }
  } else if (code_state_ >= 16 && msg->buttons[deadman_button_index_] != 0) {
    code_state_ = CHEAT_CODE_ACTIVATED;
  }
}

}  // namespace safety_monitor
}  // namespace vtr
