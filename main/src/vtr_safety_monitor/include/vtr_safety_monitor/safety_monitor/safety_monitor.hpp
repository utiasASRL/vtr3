#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vtr_safety_monitor/safety_monitor/base_monitor.hpp>
#include <vtr_safety_monitor/safety_monitor/deadman_monitor.hpp>
#include <vtr_safety_monitor/safety_monitor/heartbeat_monitor.hpp>

#include <vtr_messages/msg/robot_status.hpp>

namespace vtr {
namespace safety_monitor {

class SafetyMonitor {
 public:
  SafetyMonitor(const rclcpp::Node::SharedPtr &node);

 private:
  void getSafetyStatus();

  /**
   * \brief Converts strings provided in the launch file into safety monitor
   * objects
   */
  std::unique_ptr<BaseMonitor> createMonitor(
      const std::string &monitor_input_str);

  // Parameters
  int monitor_update_period_;
  std::vector<std::string> monitor_names_;
  double absolute_max_speed_;

  // List of Monitors
  std::vector<std::unique_ptr<BaseMonitor>> monitors_;

  /** \brief ROS-handle for communication */
  const rclcpp::Node::SharedPtr node_;
  // Objects for periodic status updates
  rclcpp::TimerBase::SharedPtr safety_status_timer_;
};

}  // namespace safety_monitor
}  // namespace vtr
