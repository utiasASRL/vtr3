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
 * \file teach.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning_v2/state_machine/states/teach.hpp"

namespace vtr {
namespace mission_planning {

using namespace teach;

StateInterface::Ptr Teach::entryState() const {
  return std::make_shared<TopologicalLocalize>();
}

StateInterface::Ptr Teach::nextStep(const StateInterface &new_state) const {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(new_state)) return Parent::nextStep(new_state);

  if (IsType(new_state)) {
    // We are not allowed to transition directly to a meta-state
    std::stringstream ss;
    ss << "Transitioning to the meta-state " << name() << " is not allowed";
    CLOG(ERROR, "mission.state_machine") << ss.str();
    throw std::runtime_error(ss.str());
  }

  // We can always transition to TopologicalLocalize, as it is the entry state.
  // Since we only get here when we are already in Teach, we can also move
  // directly to Branch.
  if (TopologicalLocalize::InChain(new_state) || Branch::InChain(new_state))
    return nullptr;

  // We can go directly to Merge from Branch, but TopologicalLocalize must pass
  // through Branch to initialize things
  else if (Merge::InChain(new_state)) {
    if (Branch::InChain(this))
      return nullptr;
    else if (TopologicalLocalize::InChain(this))
      return std::make_shared<Branch>();
  }

  // If we didn't hit one of the above cases, then something is wrong
  std::stringstream ss;
  ss << "Invalid transition from " << name() << " to " << new_state.name();
  CLOG(ERROR, "mission.state_machine") << ss.str();
  throw std::runtime_error(ss.str());
}

void Teach::processGoals(StateMachine &state_machine, const Event &event) {
  switch (event.signal) {
    case Signal::Continue:
      break;
    default:
      return Parent::processGoals(state_machine, event);
  }

  switch (event.action) {
    case Action::Continue:
      // This state doesn't actually have any automatic transitions, so there is
      // nothing here
      [[fallthrough]];
    default:
      return Parent::processGoals(state_machine, event);
  }
}

void Teach::onExit(StateMachine &state_machine, StateInterface &new_state) {
  // If the new target is a derived class, we are not exiting
  if (InChain(new_state)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root

  // Ensure that everything is in one frame and there is no more active chain
  getTactic(state_machine)->relaxGraph();
  getTactic(state_machine)->saveGraph();

  // Update the cached privileged graph for planning
  getPlanner(state_machine)->updatePrivileged();

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(state_machine, new_state);
}

void Teach::onEntry(StateMachine &state_machine, StateInterface &old_state) {
  // If the previous state was a derived class, we did not leave
  if (InChain(old_state)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(state_machine, old_state);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves

  // Add a new run before we teach any paths, regardless of previous state
  getTactic(state_machine)->addRun(true);
}

void Teach::setTarget(const PathType &match_window,
                      const VertexId &target_vertex) {
  match_window_ = match_window;
  target_vertex_ = target_vertex;
}

}  // namespace mission_planning
}  // namespace vtr
