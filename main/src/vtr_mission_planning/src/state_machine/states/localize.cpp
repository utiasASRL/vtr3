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
 * \file localize.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/states/localize.hpp"

namespace vtr {
namespace mission_planning {

using namespace localize;

StateInterface::Ptr Localize::entryState() const {
  const auto tmp = std::make_shared<TopologicalLocalize>();
  return tmp;
}

StateInterface::Ptr Localize::nextStep(const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  if (IsType(new_state)) {
    // We are not allowed to transition directly to a meta-state
    std::stringstream ss;
    ss << "Transitioning to the meta-state " << name() << " is not allowed";
    CLOG(ERROR, "mission.state-machine") << ss.str();
    throw std::runtime_error(ss.str());
  }

  // We can always transition to TopologicalLocalize, as it is the entry state.
  if (TopologicalLocalize::InChain(new_state) )
    return nullptr;
  else if (MetricLocalize::InChain(new_state)) {
    return nullptr;
  }

  // If we didn't hit one of the above cases, then something is wrong
  std::stringstream ss;
  ss << "Invalid goal transition from " << name() << " to " << new_state.name();
  CLOG(ERROR, "mission.state-machine") << ss.str();
  throw std::runtime_error(ss.str());
}

void Localize::processGoals(StateMachine &state_machine, const Event &event) {
  switch (event.signal) {
    case Signal::Continue:
      break;
    default:
      return Parent::processGoals(state_machine, event);
  }

  switch (event.action) {
    case Action::Continue:
      [[fallthrough]];
    default:
      return Parent::processGoals(state_machine, event);
  }
}

void Localize::onExit(StateMachine &state_machine, StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state) && !IsType(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  const auto tactic = getTactic(state_machine);
  tactic->finishRun();

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void Localize::onEntry(StateMachine &state_machine, StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state) && !IsType(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  getTactic(state_machine)->addRun(false);
}

}  // namespace mission_planning
}  // namespace vtr
