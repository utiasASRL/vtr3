// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file controller_switch.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning/state_machine/base_state.hpp"

namespace vtr {
namespace mission_planning {

class ControllerSwitch : public BaseState {
 public:
  PTR_TYPEDEFS(ControllerSwitch);
  INHERITANCE_TESTS(ControllerSwitch, StateInterface);
  using Parent = BaseState;

  std::string name() const override { return Parent::name() + "::ControllerSwitch"; }
  PipelineMode pipeline() const override { return PipelineMode::Idle; }
  StateInterface::Ptr entryState() const override;
  StateInterface::Ptr nextStep(const StateInterface &) const override;
  void processGoals(StateMachine &, const Event &) override;
  void onExit(StateMachine &, StateInterface &) override;
  void onEntry(StateMachine &, StateInterface &) override;

  /** \brief Set the persistent loc id */
  void setController(const std::string &type) {new_controller_ = type;}

 private:
  std::string new_controller_;
};

}  // namespace mission_planning
}  // namespace vtr
