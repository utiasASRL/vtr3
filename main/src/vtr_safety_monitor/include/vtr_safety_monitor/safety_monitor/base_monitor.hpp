#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vtr_logging/logging.hpp>

namespace vtr {
namespace safety_monitor {

// Define possible desired actions
// Integers are used to determine priority, higher num = higher priority
const int CONTINUE = 0;
const int PAUSE = 1;
const std::string ACTION_STRINGS[] = {"CONTINUE", "PAUSE"};

struct MonitorSignal {
  MonitorSignal(const rclcpp::Node::SharedPtr &node,
                const std::string &name = "Anonymous Monitor",
                const double &timeout = std::numeric_limits<double>::max())
      : monitor_name(name), update_timeout(timeout), last_update(node->now()) {}

  /// Monitor parameters
  /** \brief  */
  std::string monitor_name;
  /** \brief  */
  double update_timeout;

  // Monitor variables
  /** \brief  */
  rclcpp::Time last_update;
  /** \brief  */
  int desired_action = PAUSE;
  /** \brief  */
  double speed_limit = std::numeric_limits<double>::max();
};

class BaseMonitor {
 public:
  /** \brief  */
  BaseMonitor(const std::shared_ptr<rclcpp::Node> &node) : node_(node) {}

  /** \brief  */
  void updateSafetyMonitorAction(
      int &desired_action, double &speed_limit,
      std::vector<std::string> &limiting_signal_monitor_names,
      std::vector<int> &limiting_signal_monitor_actions);

 protected:
  /** \brief ROS-handle for communication */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief  */
  std::vector<MonitorSignal> monitor_signals_;

};  // class BaseMonitor

}  // namespace safety_monitor
}  // namespace vtr
