// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file controller_switch.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/states/controller_switch.hpp"
#include "vtr_path_planning/dynamic_path_planner.hpp"

namespace vtr {
namespace mission_planning {

StateInterface::Ptr ControllerSwitch::entryState() const { return nullptr; }

StateInterface::Ptr ControllerSwitch::nextStep(const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void ControllerSwitch::processGoals(StateMachine &state_machine, const Event &) {
  return Parent::processGoals(state_machine, Event(Action::EndGoal));
}

void ControllerSwitch::onExit(StateMachine &state_machine, StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state) && !IsType(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void ControllerSwitch::onEntry(StateMachine &state_machine, StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state) && !IsType(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves

  using DynamicPathPlanner = path_planning::DynamicPathPlanner;

  const auto pp = getPathPlanner(state_machine);
  DynamicPathPlanner::Ptr dpp = std::static_pointer_cast<DynamicPathPlanner>(pp);
  if (dpp != nullptr) {
    dpp->setController(new_controller_);
    CLOG(DEBUG, "mission.state_machine") << "Requesting controller " << new_controller_;
  } else {
    CLOG(ERROR, "mission.state_machine") << "Requesting controller change but T&R is not using a DynamicPathPlanner!";
  }
}

}  // namespace mission_planning
}  // namespace vtr
