#include <vtr_safety_monitor/safety_monitor/base_monitor.hpp>

namespace vtr {
namespace safety_monitor {

void BaseMonitor::updateSafetyMonitorAction(
    int &desired_action, double &speed_limit,
    std::vector<std::string> &limiting_signal_monitor_names,
    std::vector<int> &limiting_signal_monitor_actions) {
  // Check all signals for a given monitor
  for (auto &signal : monitor_signals_) {
    // Check how long it has been since the monitor received a signal.
    double elapsed_time = (node_->now() - signal.last_update).seconds();

    // Get the desired behavior for the given signal
    int desired_action_signal =
        elapsed_time > signal.update_timeout ? PAUSE : signal.desired_action;

    // Store signal info if NOT continue
    if (desired_action_signal != CONTINUE) {
      limiting_signal_monitor_names.push_back(signal.monitor_name);
      limiting_signal_monitor_actions.push_back(desired_action_signal);
      speed_limit = std::min(speed_limit, signal.speed_limit);
    }

    // Find the worst-case desired behavior
    desired_action = std::max(desired_action, desired_action_signal);
  }
}

}  // namespace safety_monitor
}  // namespace vtr
