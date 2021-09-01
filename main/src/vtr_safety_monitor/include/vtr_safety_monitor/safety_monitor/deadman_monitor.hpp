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
#pragma once

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include <vtr_safety_monitor/safety_monitor/base_monitor.hpp>

using JoyMsg = sensor_msgs::msg::Joy;

namespace vtr {
namespace safety_monitor {

enum class FOLLOWING_MODE { AUTO, MANUAL };

/**
 * \brief Requires deadman button to be pressed on XBox controller during
 * autonomous repeating
 */
class DeadmanMonitor : public BaseMonitor {
 public:
  DeadmanMonitor(const std::shared_ptr<rclcpp::Node>& node);
  void gamepadCallback(const JoyMsg::SharedPtr msg);

  const static int axis_array[16];
  const static int value_array[16];

 private:
  void checkCodeState(const JoyMsg::SharedPtr& msg);

  rclcpp::Subscription<JoyMsg>::SharedPtr gamepad_subscriber_;

  int deadman_button_index_;

  FOLLOWING_MODE following_mode_ = FOLLOWING_MODE::MANUAL;
  // the state of the cheat code.
  int code_state_ = 0;
  // the timestamp when hte last correct value in the code sequence was added.
  rclcpp::Time time_of_last_code_sequence_;

  // The state when the cheat code is active.
  static const int CHEAT_CODE_ACTIVATED = 7777;
};

}  // namespace safety_monitor
}  // namespace vtr
