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
 * \file repeat.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_mission_planning/states/repeat.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

auto Repeat::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) return Parent::nextStep(newState);

  if (IsType(newState)) {
    // We are not allowed to transition directly to a meta-state
    std::stringstream ss;
    ss << "[Repeat] Transitioning to the meta-state " << this->name()
       << " is not allowed";
    throw std::runtime_error(ss.str());
  }

  using namespace repeat;

  // We can always transition to TopologicalLocalize, as it is the entry state.
  // Since we only get here when we are already in Repeat, we can also move
  // directly to Plan.
  if (TopologicalLocalize::InChain(newState) || Plan::InChain(newState)) {
    return nullptr;
  }
  // We can go directly to MetricLocalize from anything but TopologicalLocalize
  else if (MetricLocalize::InChain(newState)) {
    if (Plan::InChain(this) || Follow::InChain(this))
      return nullptr;
    else if (TopologicalLocalize::InChain(this))
      return BasePtr(new Plan(*this));
  }
  // Going to following requires traversing the chain TopologicalLocalize -->
  // Plan --> MetricLocalize --> Follow
  else if (Follow::InChain(newState)) {
    if (MetricLocalize::InChain(this))
      return nullptr;
    else if (TopologicalLocalize::InChain(this))
      return BasePtr(new Plan(*this));
    else if (Plan::InChain(this))
      return BasePtr(new MetricLocalize(*this));
  }

  // If we didn't hit one of the above cases, then something is wrong
  std::stringstream ss;
  ss << "[Repeat] Invalid goal transition from " << this->name() << " to "
     << newState->name();
  throw std::runtime_error(ss.str());
}

auto Repeat::entryState(const Base *) const -> BasePtr {
  Ptr rptr(new repeat::TopologicalLocalize(*this));
  rptr->container_ = this->container_;
  return rptr;
}

void Repeat::processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                          const Event &event) {
  switch (event.signal_) {
    case Signal::Continue:
      break;
    default:
      // All signals should be directly handled by the children they affect.  If
      // we have a signal here, pass it along to the base class to do the actual
      // goal swapping/error throwing
      return Parent::processGoals(tactic, goal_lock, event);
  }

  switch (event.type_) {
    case Action::Continue:
      /// \todo Check path completion?
      if (false)
        return Parent::processGoals(tactic, goal_lock, Event(Action::EndGoal));

      // NOTE: the lack of a break statement here is intentional, to allow
      // unhandled cases to percolate up the chain
      [[fallthrough]];
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}

void Repeat::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  {
    auto lock = tactic->lockPipeline();
    // Clear the path and stop the path tracker
    tactic->setPath(PathType());

    // save the graph
    tactic->saveGraph();
  }
  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

void Repeat::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) {
    // Propagate repeat-specific data between states
    this->waypoints_ = dynamic_cast<Repeat *>(oldState)->waypoints_;
    this->waypointSeq_ = dynamic_cast<Repeat *>(oldState)->waypointSeq_;
    this->targetVertex_ = dynamic_cast<Repeat *>(oldState)->targetVertex_;
    return;
  }

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves

  // Add a new run, but only if we were previously in a teach state, and not if
  // we are only localizing
  if (targetVertex_.isSet()) {
    std::string err{"Target vertex in repeat should never be set. See header."};
    LOG(ERROR) << err;
    throw std::runtime_error{err};
  }
  addRunInternal_(false);
}

}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
