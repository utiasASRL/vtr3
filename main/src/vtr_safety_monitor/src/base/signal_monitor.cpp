#include <vtr_safety_monitor/base/signal_monitor.hpp>
#include <utility>

namespace vtr {
namespace safety_monitor {

SignalMonitor::SignalMonitor(const std::shared_ptr<rclcpp::Node> node) : node_(node) {
  desired_action = PAUSE;
  signal_monitor_status_unknown = true;
  max_allowed_speed_ = node_->get_parameter("max_allowed_speed").as_double();    //todo: prefix?
}

void SignalMonitor::initializeType(int /*type_in*/) {
  monitor_type = DISCRETE_MONITOR;
}

void SignalMonitor::initialize(std::string name_in, double max_time_in) {
  monitor_name = std::move(name_in);
  last_update = rclcpp::Clock().now();
  max_time_between_updates = max_time_in;
}

// Interactions
void SignalMonitor::setMonitorDesiredAction(int desired_action_in) {
  desired_action = desired_action_in;
  last_update = rclcpp::Clock().now();
  signal_monitor_status_unknown = false;
}

int SignalMonitor::getDesiredAction() {
  return desired_action;
}

void SignalMonitor::setStatusUnknown() {
  signal_monitor_status_unknown = true;
}

void SignalMonitor::registerMsgButNoStatusChange() {
  last_update = rclcpp::Clock().now();
  signal_monitor_status_unknown = false;
}

rclcpp::Time SignalMonitor::getLastUpdateTime() {
  return last_update;
}

void SignalMonitor::setMsgTimeout(double new_msg_timeout) {
  max_time_between_updates = new_msg_timeout;
}

double SignalMonitor::getMaxAllowedSpeed() {
  return max_allowed_speed_;
}

void SignalMonitor::setMaxAllowedSpeed(double &max_allowed_speed_in) {
  max_allowed_speed_ = std::abs(max_allowed_speed_in);
}

} // safety_monitor
} // vtr
