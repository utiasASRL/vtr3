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
 * \file spin.hpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning/state_machine/states/pause.hpp"

namespace vtr {
namespace mission_planning {
namespace pause {

class Spin : public Pause {
 public:
  PTR_TYPEDEFS(Spin);
  INHERITANCE_TESTS(Spin, StateInterface);
  using Parent = Pause;
  // clang-format off
  std::string name() const override { return Parent::name() + "::Pause"; }
  PipelineMode pipeline() const override { return PipelineMode::PauseSpin; }
  StateInterface::Ptr nextStep(const StateInterface &) const override;
  void processGoals(StateMachine &, const Event &) override;
  void onExit(StateMachine &, StateInterface &) override;
  void onEntry(StateMachine &, StateInterface &) override;
  // clang-format on
};

}  // namespace pause
}  // namespace mission_planning
}  // namespace vtr