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
 * \file base_state.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/base_state.hpp"

namespace vtr {
namespace mission_planning {

StateInterface::Ptr BaseState::nextStep(const StateInterface& new_state) const {
  if (!InChain(new_state)) {
    std::string err("Transitioning outside the base state is not allowed.");
    CLOG(ERROR, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }

  if (IsType(new_state)) {
    std::string err("Transitioning to the base state is not allowed.");
    CLOG(ERROR, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }

  return new_state.entryState();
}

void BaseState::processGoals(StateMachine& state_machine, const Event& event) {
  // this state does not process any goals
  if (event.signal != Signal::Continue) {
    std::stringstream ss;
    ss << "Received unhandled signal: " << event.signal;
    CLOG(ERROR, "mission.state_machine") << ss.str();
    throw std::runtime_error(ss.str());
  }

  //
  bool success = true;
  switch (event.action) {
    case Action::Continue:
      break;
    case Action::Abort:
      // Abort: Something bad happened; stop whatever we were doing
      [[fallthrough]];
    case Action::Reset:
      // Goal canceled so we reset the statemachine
      getGoals(state_machine).clear();
      success = false;
      break;
    case Action::EndGoal:
      // EndGoal: This goal ended normally, so remove it from the stack and
      // move on
      getGoals(state_machine).pop_front();
      break;
    case Action::SwapGoal:
      // SwapGoal: This goal ended normally, but needs to be replaced with
      // another goal
      getGoals(state_machine).pop_front();
      getGoals(state_machine).push_front(event.goal);
      break;
    case Action::NewGoal:
      // NewGoal: We are changing tracks completely, so reset the goal stack
      getGoals(state_machine).clear();
      [[fallthrough]];
    case Action::AppendGoal:
      // A new goal must be added to the stack and transitioned to
      getGoals(state_machine).push_front(event.goal);
      break;
  }
  // If we ever finish our list of goals, drop into Idle automatically
  if (getGoals(state_machine).empty()) {
    CLOG(INFO, "mission.state_machine") << "Goal finished, fallback to Idle";
    getGoals(state_machine).push_front(std::make_shared<Idle>());
    if (success) triggerSuccess(state_machine);
  }

  // Check if there is another goal we must transition through first to get
  // to the target goal. We don't need to check if the target was changed,
  // as goal->nextStep(goal) always returns nullptr.
  const auto next_state = nextStep(*getGoals(state_machine).front());
  if (next_state != nullptr) getGoals(state_machine).push_front(next_state);

  // Raise appropriate callbacks for state changes/successful goal completion
  // if (getGoals(state_machine).front().get() != this)
  //   getCallback(state_machine)->stateChanged(getGoals(state_machine).front());
}

}  // namespace mission_planning
}  // namespace vtr
