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
 * \file merge.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning/state_machine/states/teach/merge.hpp"

namespace vtr {
namespace mission_planning {
namespace teach {

StateInterface::Ptr Merge::nextStep(const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void Merge::processGoals(StateMachine &state_machine, const Event &event) {
  switch (event.signal) {
    case Signal::Continue:
      break;
    case Signal::AttemptClosure:
      if (getTactic(state_machine)->isLocalized()) {
        closure_required_ = true;
        return Parent::processGoals(state_machine, Event(Action::EndGoal));
      } else {
        std::string err{"Cannot attempt closure without having localized."};
        CLOG(ERROR, "mission.state_machine") << err;
        throw std::runtime_error(err);
      }
    case Signal::ContinueTeach:
      return Parent::processGoals(
          state_machine,
          Event(Action::SwapGoal, std::make_shared<teach::Branch>()));
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

void Merge::onExit(StateMachine &state_machine, StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state) && !IsType(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  const auto tactic = getTactic(state_machine);
  if (closure_required_) tactic->connectToTrunk(true);
  tactic->setPath(PathType(), 0, tactic::EdgeTransform(true), true);
  tactic->setTrunk();  // reset target loc only

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void Merge::onEntry(StateMachine &state_machine, StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state) && !IsType(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
  const auto tactic = getTactic(state_machine);
  const auto trunk_sid = static_cast<unsigned>(match_window_.size() / 2);
  tactic->setPath(match_window_, trunk_sid, tactic::EdgeTransform(true), true);
}

}  // namespace teach
}  // namespace mission_planning
}  // namespace vtr
