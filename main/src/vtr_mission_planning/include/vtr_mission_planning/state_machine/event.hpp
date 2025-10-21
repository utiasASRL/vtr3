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
 * \file event.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/types.hpp"

namespace vtr {
namespace mission_planning {

/**
 * \brief Enumerates direct commands to the StateMachine.  These represent very
 * low level commands that don't require any complex logic or safety checking.
 */
enum class Action : int8_t {
  Abort = -2,    // Kill it, state abort and drop into idle
  Reset = -1,    // Kill it and drop into idle
  Continue = 0,  // Keep going with this state (default action)
  NewGoal,       // Set the goal state, replacing the current goal if necessary
  SwapGoal,      // Replace the goal at the top of the stack.  Like NewGoal, but
                 // only replaces
  AppendGoal,    // Append a new temporary goal state (eg. taking a brief detour
                 // to reteach a bad looking area)
  EndGoal,  // User indication that a goal is complete (eg. mapping/repair is
           // done now)
  ForceAddVertex, // Add a vertex at the robot's current pose during a teach
  ExitPause,    // Exit the pause state and resume previous activity
};

std::ostream& operator<<(std::ostream& os, const Action& action);

/**
 * \brief Enumerates descriptive signals, or world events that might require a
 * response.  These represent higher level objectives or conditions that a state
 * may need to check and respond to appropriately.
 */
enum class Signal : int8_t {
  Continue = 0,  // Keep going with this state (default action)
  // [teach::merge]
  AttemptClosure,  // Attempt to link back to the existing map
  ContinueTeach,    // Cannot merge or user has canceled merge, continue teaching
};

std::ostream& operator<<(std::ostream& os, const Signal& signal);

class StateInterface;

struct Event {
  PTR_TYPEDEFS(Event);
  using VertexId = tactic::VertexId;

  // Constructors for high level command events
  static Event::Ptr StartIdle(const VertexId& vertex_id = VertexId::Invalid());
  static Event::Ptr StartTeach();
  static Event::Ptr StartMerge(const std::vector<VertexId>& match_window);
  static Event::Ptr StartRepeat(const std::list<VertexId>& waypoints);
  static Event::Ptr StartLocalize();
  static Event::Ptr StartPause();
  static Event::Ptr SwitchController(const std::string& new_controller);

  static Event::Ptr Reset();

  std::string signalName() const;
  std::string actionName() const;

  Event() = default;
  Event(const Action& action0,
        const std::shared_ptr<StateInterface>& goal0 = nullptr,
        const Signal& signal0 = Signal::Continue)
      : action(action0), goal(goal0), signal(signal0) {}
  Event(const Signal& signal0) : signal(signal0) {}

  friend std::ostream& operator<<(std::ostream& os, const Event& e);

  Action action = Action::Continue;
  std::shared_ptr<StateInterface> goal = nullptr;
  Signal signal = Signal::Continue;
};

}  // namespace mission_planning
}  // namespace vtr
