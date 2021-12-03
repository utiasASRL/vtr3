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
 * \file merge.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning_v2/state_machine/states/teach.hpp"

namespace vtr {
namespace mission_planning {
namespace teach {

class Merge : public Teach {
 public:
  PTR_TYPEDEFS(Merge);
  INHERITANCE_TESTS(Merge, StateInterface);
  using Parent = Teach;

  std::string name() const override { return Parent::name() + "::Merge"; };
  PipelineMode pipeline() const override { return PipelineMode::Merging; }
  StateInterface::Ptr nextStep(const StateInterface &) const override;
  void processGoals(StateMachine &, const Event &) override;
  void onExit(StateMachine &, StateInterface &) override;
  void onEntry(StateMachine &, StateInterface &) override;

  /** \brief Set the target to match against */
  void setTarget(const PathType &match_window) { match_window_ = match_window; }

 private:
  /** \brief Window of vertices to search against for a match/localization */
  PathType match_window_;

  bool canceled_ = false;
};

}  // namespace teach
}  // namespace mission_planning
}  // namespace vtr
