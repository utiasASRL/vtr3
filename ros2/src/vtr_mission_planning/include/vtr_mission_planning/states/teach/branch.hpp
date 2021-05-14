#pragma once

#include <vtr_mission_planning/states/teach.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace teach {

class Branch : public Teach {
 public:
  PTR_TYPEDEFS(Branch)
  INHERITANCE_TESTS(Branch, Base)
  using Parent = Teach;

  Branch(const Parent &parent = Parent()) : Parent(parent) {}
  Branch(const Base &base) : Parent(base) {}
  Branch(const Branch &) = default;
  Branch(Branch &&) = default;

  virtual ~Branch() {}

  Branch &operator=(const Branch &) = default;
  Branch &operator=(Branch &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override { return Parent::name() + "::Branch"; }
  /** \brief Returns the type of pipeline that this state requires. */
  PipelineMode pipeline() const override { return PipelineMode::Branching; }
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

}  // namespace teach
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
