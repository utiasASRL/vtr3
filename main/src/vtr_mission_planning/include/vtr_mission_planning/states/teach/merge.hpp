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
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_mission_planning/states/teach.hpp>

namespace vtr {
namespace mission_planning {
namespace state {
namespace teach {

class Merge : public Teach {
 public:
  PTR_TYPEDEFS(Merge)
  INHERITANCE_TESTS(Merge, Base)
  using Parent = Teach;

  Merge(const Parent &parent = Parent()) : Parent(parent), cancelled_(false) {}
  Merge(const Base &base) : Parent(base), cancelled_(false) {}
  Merge(const Merge &) = default;
  Merge(Merge &&) = default;

  virtual ~Merge() {}

  Merge &operator=(const Merge &) = default;
  Merge &operator=(Merge &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override { return Parent::name() + "::Merge"; };
  /** \brief Returns the type of pipeline that this state requires. */
  PipelineMode pipeline() const override { return PipelineMode::Merging; }
  /** \brief Returns the next intermediate state */
  BasePtr nextStep(const Base *newState) const override;
  /** \brief The entryState function is not implemented for leaf states */

  /** \brief Checks the navigation state and perform state transitions */
  void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                    const Event &event = Event()) override;

  /** \brief Called as a cleanup method when the state exits. */
  void onExit(Tactic *tactic, Base *newState) override;
  /** \brief Called as a setup method when the state is entered. */
  void onEntry(Tactic *tactic, Base *oldState) override;

 protected:
  bool cancelled_;

  bool canCloseLoop_(Tactic *tactic);
};

}  // namespace teach
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
