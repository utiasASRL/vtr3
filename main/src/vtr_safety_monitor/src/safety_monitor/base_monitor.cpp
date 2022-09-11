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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_safety_monitor/safety_monitor/base_monitor.hpp"

namespace vtr {
namespace safety_monitor {

std::ostream &operator<<(std::ostream &os, const Action &action) {
  switch (action) {
    case Action::Continue:
      os << "Continue";
      return os;
    case Action::Pause:
      os << "Pause";
      return os;
  };

  // GCC seems to complain when this isn't here
  return os;
}

void BaseMonitor::updateSafetyMonitorAction(
    Action &desired_action, double &speed_limit,
    std::vector<std::string> &limiting_signal_monitor_names,
    std::vector<Action> &limiting_signal_monitor_actions) {
  // Check how long it has been since the monitor received a signal.
  const auto elapsed_time = (node_->now() - last_update_).seconds();

  // Get the desired behavior for the given signal
  const auto desired_action_signal =
      elapsed_time > timeout_ ? Action::Pause : desired_action_;

  // Store signal info if NOT continue
  if (desired_action_signal != Action::Continue) {
    limiting_signal_monitor_names.push_back(name_);
    limiting_signal_monitor_actions.push_back(desired_action_signal);
    speed_limit = std::min(speed_limit, speed_limit_);
  }

  // Find the worst-case desired behavior
  desired_action = (int)desired_action > (int)desired_action_signal
                       ? desired_action
                       : desired_action_signal;
}

}  // namespace safety_monitor
}  // namespace vtr
