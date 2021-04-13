#include <vtr_safety_monitor/base/safety_monitor_input_base.hpp>

#include <vtr_logging/logging.hpp>

namespace vtr {
namespace safety_monitor {

SafetyMonitorInput::SafetyMonitorInput(const std::shared_ptr<rclcpp::Node> node) : node_(node) {
  // Do nothing
}

bool SafetyMonitorInput::updateSafetyMonitorAction(int &desired_action, double &speed_limit,
                                                   std::vector<std::string> &limiting_signal_monitor_names,
                                                   std::vector<int> &limiting_signal_monitor_actions) {

  // Check all signals for a given monitor
  for (auto &signal_monitor : signal_monitors) {

    int desired_action_signal;

    // Check how long it has been since the monitor received a signal!
    rclcpp::Duration time_since_update_ros = rclcpp::Clock().now() - signal_monitor.getLastUpdateTime();
    double time_since_update = time_since_update_ros.seconds();

    // Get the desired behavior for the given signal
    if (time_since_update > signal_monitor.max_time_between_updates &&
        signal_monitor.max_time_between_updates > 0) {
      signal_monitor.setStatusUnknown();
      desired_action_signal = UNKNOWN;

    } else if (signal_monitor.monitor_type == DISCRETE_MONITOR) {
      desired_action_signal = signal_monitor.getDesiredAction();

    } else {
      LOG(ERROR) << "Incorrect definition of safety monitor type: only supporting Discrete monitors.";
      desired_action_signal = PAUSE;
    }

    // Store signal info if NOT continue
    if (desired_action_signal != CONTINUE) {
      limiting_signal_monitor_names.push_back(signal_monitor.monitor_name);
      limiting_signal_monitor_actions.push_back(desired_action_signal);
      speed_limit = std::min(speed_limit, signal_monitor.getMaxAllowedSpeed());
    }

    // Find the worst-case desired behavior
    desired_action = std::max(desired_action, desired_action_signal);
  }
  return true;
}

} // safety_monitor
} // vtr
