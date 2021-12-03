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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning_v2/state_machine/states/teach/merge.hpp"

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
    case Signal::AttemptClosure: {
      bool can_close = getTactic(state_machine)->canCloseLoop();
      if (can_close) {
        canceled_ = false;
        Event tmp(Action::EndGoal);
        tmp.signal = event.signal;
        /// \todo probably fall through here as well so that we continue teach?
        return Parent::processGoals(state_machine, tmp);
      } else {
        canceled_ = true;
        [[fallthrough]];
      }
    }
    case Signal::ContinueTeach: {
      Event tmp(Action::SwapGoal, std::make_shared<teach::Branch>());
      tmp.signal = event.signal;
      return Parent::processGoals(state_machine, tmp);
    }
    default:
      return Parent::processGoals(state_machine, event);
  }

  switch (event.action) {
    case Action::AppendGoal:
    case Action::NewGoal:
    case Action::Abort:
      canceled_ = true;
      [[fallthrough]];
    default:
      return Parent::processGoals(state_machine, event);
  }
}

void Merge::onExit(StateMachine &state_machine, StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  // If we localized, add a loop closure to whatever match we found. Otherwise,
  // do nothing.
  if (!canceled_) {
    getTactic(state_machine)->connectToTrunk(true, true);
  } else {
    CLOG(INFO, "mission.state_machine")
        << "Not merging due to localization conditions/goal termination";
  }

  // Clear the path for merging
  getTactic(state_machine)->setPath(PathType());

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void Merge::onEntry(StateMachine &state_machine, StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
  getTactic(state_machine)->setPath(match_window_);

  // Reset this in case we re-enter the same instance of this goal
  canceled_ = true;
}

}  // namespace teach
}  // namespace mission_planning
}  // namespace vtr
