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
 * \file base_monitor.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
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
