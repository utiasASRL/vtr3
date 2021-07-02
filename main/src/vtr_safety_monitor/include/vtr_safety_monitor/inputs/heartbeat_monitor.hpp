#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vtr_messages/msg/robot_status.hpp>

#include <vtr_safety_monitor/base/safety_monitor_input_base.hpp>

#include <Eigen/Dense>

using RobotStatus = vtr_messages::msg::RobotStatus;

namespace vtr {
namespace safety_monitor {

class HeartbeatMonitorInput : public SafetyMonitorInput {
 public :
  /** \brief Constructor */
  explicit HeartbeatMonitorInput(std::shared_ptr<rclcpp::Node> node);
  ~HeartbeatMonitorInput() = default;

 private :
  /** \brief Resets heartbeat timer */
  void statusCallback(RobotStatus::SharedPtr status);

  /** \brief Timer has gone off */
  void timedCallback();

  /** \brief Subscriber to the robot status message */
  rclcpp::Subscription<RobotStatus>::SharedPtr status_subscriber_;

  /** \brief The heartbeat timer */
  rclcpp::TimerBase::SharedPtr timer_;

  /** \brief Max time we can go without a status update from Navigator */
  double period_;

};

} // safety_monitor
} // vtr
