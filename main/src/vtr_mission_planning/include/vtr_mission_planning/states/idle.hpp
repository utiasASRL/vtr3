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
