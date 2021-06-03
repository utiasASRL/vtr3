#pragma once

#include <vtr_mission_planning/states/repeat.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace repeat {

class Follow : public Repeat {
 public:
  PTR_TYPEDEFS(Follow)
  INHERITANCE_TESTS(Follow, Base)
  using Parent = Repeat;

  Follow(const Parent &parent = Parent()) : Parent(parent) {}
  Follow(const Base &base) : Parent(base) {}
  Follow(const Follow &) = default;
  Follow(Follow &&) = default;

  virtual ~Follow() {}

  Follow &operator=(const Follow &) = default;
  Follow &operator=(Follow &&) = default;

  /** \brief Return a string representation of the state */
  std::string name() const override { return Parent::name() + "::Follow"; }
  /** \brief Returns the type of pipeline that this state requires. */
  PipelineMode pipeline() const override { return PipelineMode::Following; }
  /** \brief Returns the next intermediate state */
  virtual BasePtr nextStep(const Base *newState) const;
  /** \brief The entryState function is not implemented for leaf states */

  /** \brief Checks the navigation state and perform state transitions */
  virtual void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                            const Event &event = Event());
  /** \brief Called as a cleanup method when the state exits. */
  virtual void onExit(Tactic *tactic, Base *newState);
  /** \brief Called as a setup method when the state is entered. */
  virtual void onEntry(Tactic *tactic, Base *oldState);
};

}  // namespace repeat
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
