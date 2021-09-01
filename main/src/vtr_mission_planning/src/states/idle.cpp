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
 * \file idle.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_mission_planning/states/idle.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

auto Idle::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) return Parent::nextStep(newState);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

auto Idle::entryState(const Base *) const -> BasePtr {
  Ptr rptr(new Idle());
  rptr->container_ = this->container_;
  return rptr;
}

void Idle::processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                        const Event &event) {
  switch (event.signal_) {
    case Signal::Continue:
      break;
    default:
      // All signals should be directly handled by the children they affect. If
      // we have a signal here, pass it along to the base class to do the actual
      // goal swapping/error throwing
      return Parent::processGoals(tactic, goal_lock, event);
  }

  switch (event.type_) {
    case Action::Continue:
      // This state doesn't actually have any automatic transitions, so there is
      // nothing here

      // NOTE: the lack of a break statement here is intentional, to allow
      // unhandled cases to percolate up the chain
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}

void Idle::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) return;

  // Idle does not do anything special when it exits

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

void Idle::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // Clear the path when we enter Idle
  tactic->setPath(PathType());
}
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
