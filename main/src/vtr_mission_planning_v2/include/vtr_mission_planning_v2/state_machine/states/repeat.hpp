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
 * \file repeat.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning_v2/state_machine/base_state.hpp"

namespace vtr {
namespace mission_planning {

class Repeat : public BaseState {
 public:
  PTR_TYPEDEFS(Repeat);
  INHERITANCE_TESTS(Repeat, StateInterface);
  using Parent = BaseState;

  std::string name() const override { return Parent::name() + "::Repeat"; }
  StateInterface::Ptr entryState() const override;
  StateInterface::Ptr nextStep(const StateInterface &) const override;
  void processGoals(StateMachine &, const Event &) override;
  void onExit(StateMachine &, StateInterface &) override;
  void onEntry(StateMachine &, StateInterface &) override;

  /** \brief Set the list of waypoints to follow */
  void setWaypoints(
      const std::list<VertexId> &waypoints,
      const std::list<uint64_t> &waypoint_seq = std::list<uint64_t>());

 protected:
  /** \brief vertex ids that the robot is trying to get to */
  std::list<VertexId> waypoints_;
  /** \brief vertex of sequence ids of the waypoints along the current path */
  std::list<uint64_t> waypoint_seq_;
};

}  // namespace mission_planning
}  // namespace vtr

#include "vtr_mission_planning_v2/state_machine/states/repeat/follow.hpp"
#include "vtr_mission_planning_v2/state_machine/states/repeat/metric_localize.hpp"
#include "vtr_mission_planning_v2/state_machine/states/repeat/plan.hpp"
#include "vtr_mission_planning_v2/state_machine/states/repeat/topological_localize.hpp"