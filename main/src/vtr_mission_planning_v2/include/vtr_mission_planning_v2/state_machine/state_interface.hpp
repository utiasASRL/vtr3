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
 * \file base_state.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_common/utils/macros.hpp"
#include "vtr_mission_planning_v2/state_machine/event.hpp"
#include "vtr_mission_planning_v2/state_machine/state_machine.hpp"

namespace vtr {
namespace mission_planning {

/**
 * \note We have two type of states: meta and concrete states. Meta states
 * cannot be instantiated, while concrete states have associated pipeline.
 */
class StateInterface {
 public:
  PTR_TYPEDEFS(StateInterface);

  using PipelineMode = tactic::PipelineMode;
  using PathType = tactic::PathType;
  using VertexId = tactic::VertexId;

  virtual ~StateInterface() = default;

  /** \brief Return a string representation of the state */
  virtual std::string name() const = 0;
  /** \brief Returns the pipeline that this *concrete* state requires. */
  virtual PipelineMode pipeline() const = 0;
  /** \brief Checks the navigation state and perform state transitions */
  virtual void processGoals(StateMachine&, const Event&) = 0;
  /** \brief Returns the next intermediate state */
  virtual Ptr nextStep(const StateInterface&) const = 0;
  /**
   * \brief Returns the entry state of this state
   * \note this function is only called for concrete states, and an entry state
   * should return nullptr.
   */
  virtual Ptr entryState() const = 0;
  /** \brief Called as a cleanup method when the state exits. */
  virtual void onExit(StateMachine&, StateInterface&) = 0;
  /** \brief Called as a setup method when the state is entered. */
  virtual void onEntry(StateMachine&, StateInterface&) = 0;

 protected:
  void triggerSuccess(StateMachine& sm) { sm.triggerSuccess(); }
  StateMachine::GoalStack& getGoals(StateMachine& sm) { return sm.goals(); }
  /** \brief Returns the tactic of the state machine */
  StateMachine::Tactic::Ptr getTactic(StateMachine& sm) { return sm.tactic(); }
  /** \brief Returns the route planner of the state machine */
  StateMachine::RoutePlanner::Ptr getPlanner(StateMachine& sm) {
    return sm.planner();
  }
  /** \brief Returns the callback of the state machine */
  StateMachineCallback::Ptr getCallback(StateMachine& sm) {
    return sm.callback();
  }
};

/** \brief Convenience name output to stream */
std::ostream& operator<<(std::ostream& os, const StateInterface& s);

}  // namespace mission_planning
}  // namespace vtr