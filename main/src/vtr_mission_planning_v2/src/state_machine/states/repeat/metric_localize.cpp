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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning_v2/state_machine/states/repeat/metric_localize.hpp"

namespace vtr {
namespace mission_planning {
namespace repeat {

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
    case Action::Continue: {
      const auto tactic_acquired = getTactic(state_machine);
      /// \todo this is not thread safe, should be changed to a tactic call
      // For now we can exit as long as the localization status is Confident
      if (tactic_acquired->persistentLoc().successes > 5) {
        CLOG(INFO, "mission.state_machine")
            << "Metric localization is successful, existing the state.";
        tactic_acquired->connectToTrunk(false, false);
        return Parent::processGoals(state_machine, Event(Action::EndGoal));
      }
    }
      [[fallthrough]];
    default:
      return Parent::processGoals(state_machine, event);
  }
}

void MetricLocalize::onExit(StateMachine &state_machine,
                            StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void MetricLocalize::onEntry(StateMachine &state_machine,
                             StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
}

}  // namespace repeat
}  // namespace mission_planning
}  // namespace vtr