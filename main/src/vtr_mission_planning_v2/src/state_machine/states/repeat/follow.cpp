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
 * \file follow.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning_v2/state_machine/states/repeat/follow.hpp"

namespace vtr {
namespace mission_planning {
namespace repeat {

StateInterface::Ptr Follow::nextStep(const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void Follow::processGoals(StateMachine &state_machine, const Event &event) {
  switch (event.signal) {
    case Signal::Continue:
      break;
    case Signal::LocalizeFail: {
      // We do not know where we are, so detour back to MetricLocalize
      Event tmp(Action::AppendGoal, std::make_shared<MetricLocalize>(),
                event.signal);
      return Parent::processGoals(state_machine, tmp);
    }
    case Signal::GoalReached: {
      // The path is finished, and we can safely exit
      Event tmp(Action::EndGoal, nullptr, event.signal);
      return Parent::processGoals(state_machine, tmp);
    }
    default:
      return Parent::processGoals(state_machine, event);
  }

  switch (event.action) {
    case Action::Continue: {
      /// \todo change to use the goal reached signal
      const auto tactic_acquired = getTactic(state_machine);

      if (!waypoints_.empty()) {
        CLOG(DEBUG, "mission.state_machine")
            << "Front waypoint is: " << waypoints_.front()
            << ", id: " << waypoint_seq_.front() << ", distance:"
            << tactic_acquired->distanceToSeqId(waypoint_seq_.front());
      }

      // If we have passed a waypoint, remove it from the list
      while (!waypoint_seq_.empty()) {
        const auto dist =
            tactic_acquired->distanceToSeqId(waypoint_seq_.front());
        if (dist > 0) break;
        CLOG(INFO, "mission.state_machine")
            << "Popping waypoint " << waypoints_.front() << " with distance "
            << dist;
        waypoints_.pop_front();
        waypoint_seq_.pop_front();
      }

      // We are done when there are no waypoints left
      if (waypoints_.empty()) {
        if (tactic_acquired->pathFollowingDone()) {
          CLOG(INFO, "mission.state_machine")
              << "Path following completed; ending the current goal.";
          return Parent::processGoals(state_machine, Event(Action::EndGoal));
        } else {
          CLOG_EVERY_N(16, INFO, "mission.state_machine")
              << "All waypoints complete; waiting on path tracker to finish";
        }
      } else {
        const auto travelled = -1 * tactic_acquired->distanceToSeqId(0);
        const auto remained =
            tactic_acquired->distanceToSeqId(waypoint_seq_.back());
        const auto percent = travelled / (travelled + remained);
        getCallback(state_machine)->stateUpdate(percent * 100);
        CLOG_EVERY_N(16, INFO, "mission.state_machine")
            << "Percent complete is: " << percent;
      }
    }
      [[fallthrough]];
    default:
      return Parent::processGoals(state_machine, event);
  }
}

void Follow::onExit(StateMachine &state_machine, StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void Follow::onEntry(StateMachine &state_machine, StateInterface &old_state) {
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
