#pragma once

#include <vtr_mission_planning/states/repeat.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace repeat {

class Plan : public Repeat {
 public:
  PTR_TYPEDEFS(Plan)
  INHERITANCE_TESTS(Plan, Base)
  using Parent = Repeat;

  Plan(const Parent &parent = Parent()) : Parent(parent) {}
  Plan(const Base &base) : Parent(base) {}
  Plan(const Plan &) = default;
  Plan(Plan &&) = default;

  virtual ~Plan() {}

  Plan &operator=(const Plan &) = default;
  Plan &operator=(Plan &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override { return Parent::name() + "::Plan"; }
  /** \brief Returns the type of pipeline that this state requires. */
  PipelineMode pipeline() const override { return PipelineMode::Idle; }
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
};

}  // namespace repeat
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
