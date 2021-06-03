#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vtr_safety_monitor/base/signal_monitor.hpp>

namespace vtr {
namespace safety_monitor {

class SafetyMonitorInput {
 public :
  /** \brief  */
  SafetyMonitorInput(std::shared_ptr<rclcpp::Node> node);

  /** \brief  */
  bool updateSafetyMonitorAction(int &desired_action,
                                 double &speed_limit,
                                 std::vector<std::string> &limiting_signal_monitor_names,
                                 std::vector<int> &limiting_signal_monitor_actions);

 protected :
  /** \brief ROS-handle for communication */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief  */
  std::vector<SignalMonitor> signal_monitors;

}; // class safetyMonitorInput

} // namespace safety_monitor
} // namespace vtr
