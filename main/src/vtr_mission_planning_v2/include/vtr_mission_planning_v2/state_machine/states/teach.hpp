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
 * \file teach.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning_v2/state_machine/base_state.hpp"

namespace vtr {
namespace mission_planning {

class Teach : public BaseState {
 public:
  PTR_TYPEDEFS(Teach);
  INHERITANCE_TESTS(Teach, StateInterface);
  using Parent = BaseState;

  std::string name() const override { return Parent::name() + "::Teach"; }
  StateInterface::Ptr entryState() const override;
  StateInterface::Ptr nextStep(const StateInterface &) const override;
  void processGoals(StateMachine &, const Event &) override;
  void onExit(StateMachine &, StateInterface &) override;
  void onEntry(StateMachine &, StateInterface &) override;
};

}  // namespace mission_planning
}  // namespace vtr

#include "vtr_mission_planning_v2/state_machine/states/teach/branch.hpp"
#include "vtr_mission_planning_v2/state_machine/states/teach/merge.hpp"
#include "vtr_mission_planning_v2/state_machine/states/teach/topological_localize.hpp"