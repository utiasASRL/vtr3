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
