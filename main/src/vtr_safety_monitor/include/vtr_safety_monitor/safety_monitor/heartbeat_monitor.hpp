#pragma once

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <vtr_messages/msg/robot_status.hpp>

#include <vtr_safety_monitor/safety_monitor/base_monitor.hpp>

using RobotStatusMsg = vtr_messages::msg::RobotStatus;

namespace vtr {
namespace safety_monitor {

class HeartbeatMonitor : public BaseMonitor {
 public:
  /** \brief Constructor */
  HeartbeatMonitor(const std::shared_ptr<rclcpp::Node>& node);

 private:
  /** \brief Resets heartbeat timer */
  void statusCallback(RobotStatusMsg::SharedPtr status);

  /** \brief Timer has gone off */
  void timedCallback();

  /** \brief Subscriber to the robot status message */
  rclcpp::Subscription<RobotStatusMsg>::SharedPtr status_subscriber_;

  /** \brief The heartbeat timer */
  rclcpp::TimerBase::SharedPtr timer_;

  /** \brief Max time we can go without a status update from Navigator */
  int period_;

  /** \brief Whether we expect the robot is moving */
  bool following_ = false;
};

}  // namespace safety_monitor
}  // namespace vtr
