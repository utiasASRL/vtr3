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
 * \file idle.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_mission_planning/state_machine.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

class Idle : public BaseState {
 public:
  PTR_TYPEDEFS(Idle)
  INHERITANCE_TESTS(Idle, Base)
  using Parent = BaseState;

  Idle() {}
  Idle(const Parent &parent) : Parent(parent) {}
  Idle(const Idle &) = default;
  Idle(Idle &&) = default;

  virtual ~Idle() {}

  Idle &operator=(const Idle &) = default;
  Idle &operator=(Idle &&) = default;

  /** \brief Returns a string representation of the state */
  std::string name() const override { return Parent::name() + "::Idle"; }
  /** \brief Returns the type of pipeline that this staterequires. */
  PipelineMode pipeline() const override { return PipelineMode::Idle; }
  /** \brief Returns the next intermediate state */
  BasePtr nextStep(const Base *newState) const override;
  /** \brief State through which we must always enter this meta-state */
  BasePtr entryState(const Base *) const override;
  /** \brief Checks the navigation state and perform state transitions */
  void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                    const Event &event = Event()) override;
  /** \brief Called as a cleanup method when the state exits. */
  void onExit(Tactic *tactic, Base *newState) override;
  /** \brief Called as a setup method when the state is entered. */
  void onEntry(Tactic *tactic, Base *oldState) override;
};

}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
