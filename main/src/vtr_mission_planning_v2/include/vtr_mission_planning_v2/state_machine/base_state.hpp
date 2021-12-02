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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_common/utils/macros.hpp"
#include "vtr_mission_planning_v2/state_machine/state_interface.hpp"
#include "vtr_tactic/state_machine_interface.hpp"

namespace vtr {
namespace mission_planning {

/** \brief High-level state of VTR */
class BaseState : public StateInterface {
 public:
  PTR_TYPEDEFS(BaseState);
  INHERITANCE_TESTS(BaseState, StateInterface);

  std::string name() const override { return ""; }
  StateInterface::Ptr nextStep(const StateInterface&) const override;
  void processGoals(StateMachine&, const Event&) override;
  void onExit(StateMachine&, StateInterface&) override {}
  void onEntry(StateMachine&, StateInterface&) override {}
};

/** \brief Convenience name output to stream */
std::ostream& operator<<(std::ostream& os, const StateInterface& s);

}  // namespace mission_planning
}  // namespace vtr

#include "vtr_mission_planning_v2/state_machine/states/idle.hpp"
#include "vtr_mission_planning_v2/state_machine/states/teach.hpp"
#include "vtr_mission_planning_v2/state_machine/states/repeat.hpp"