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
 * \file metric_localize.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/states/localize/metric_localize.hpp"

namespace vtr {
namespace mission_planning {
namespace localize {

StateInterface::Ptr MetricLocalize::nextStep(
    const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void MetricLocalize::processGoals(StateMachine &state_machine,
                                  const Event &event) {
  switch (event.signal) {
    case Signal::Continue:
      break;
    default:
      return Parent::processGoals(state_machine, event);
  }

  switch (event.action) {
    case Action::Continue:
    {
      if (getTactic(state_machine)->isLocalized())
        return Parent::processGoals(state_machine, Event(Action::EndGoal));
      [[fallthrough]];
    }
    default:
      return Parent::processGoals(state_machine, event);
  }
}

void MetricLocalize::onExit(StateMachine &state_machine,
                            StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state) && !IsType(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root

  auto tactic = getTactic(state_machine);
  tactic->connectToTrunk(false);
  tactic->setPath(PathType(), 0, tactic::EdgeTransform(true), false);
  

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void MetricLocalize::onEntry(StateMachine &state_machine,
                             StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state) && !IsType(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  const auto tactic = getTactic(state_machine);
  const auto persistent_loc = tactic->getPersistentLoc();
  tactic->setPath({persistent_loc.v}, 0, persistent_loc.T, false);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
}

}  // namespace localize 
}  // namespace mission_planning
}  // namespace vtr
