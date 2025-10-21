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
 * \file mission_server.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/mission_server/mission_server.hpp"

namespace vtr {
namespace mission_planning {

std::ostream& operator<<(std::ostream& os, const GoalState& goal_state) {
  switch (goal_state) {
    case GoalState::Empty:
      os << "Empty";
      return os;
    case GoalState::Starting:
      os << "Starting";
      return os;
    case GoalState::Running:
      os << "Running";
      return os;
    case GoalState::Finishing:
      os << "Finishing";
      return os;
  };
  return os;
}

std::ostream& operator<<(std::ostream& os, const ServerState& server_state) {
  switch (server_state) {
    case ServerState::Empty:
      os << "Empty";
      return os;
    case ServerState::Processing:
      os << "Processing";
      return os;
    case ServerState::PendingPause:
      os << "PendingPause";
      return os;
    case ServerState::Paused:
      os << "Paused";
      return os;
    case ServerState::Crashed:
      os << "Crashed";
      return os;
  };
  return os;
}

std::ostream& operator<<(std::ostream& os, const GoalTarget& goal_target) {
  switch (goal_target) {
    case GoalTarget::Unknown:
      os << "Unknown";
      return os;
    case GoalTarget::Idle:
      os << "Idle";
      return os;
    case GoalTarget::Teach:
      os << "Teach";
      return os;
    case GoalTarget::Repeat:
      os << "Repeat";
      return os;
    case GoalTarget::Localize:
      os << "Localize";
      return os;
    case GoalTarget::SelectController:
      os << "SelectController";
      return os;
    case GoalTarget::Pause:
      os << "Pause";
      return os;
  };
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const CommandTarget& command_target) {
  switch (command_target) {
    case CommandTarget::Unknown:
      os << "Unknown";
      return os;
    case CommandTarget::Localize:
      os << "Localize";
      return os;
    case CommandTarget::StartMerge:
      os << "StartMerge";
      return os;
    case CommandTarget::ConfirmMerge:
      os << "ConfirmMerge";
      return os;
    case CommandTarget::ContinueTeach:
      os << "ContinueTeach";
      return os;
    case CommandTarget::ForceAddVertex:
      os << "ForceAddVertex";
      return os;
    case CommandTarget::ExitPause:
      os << "ExitPause";
      return os;
  };
  return os;
}

}  // namespace mission_planning
}  // namespace vtr
