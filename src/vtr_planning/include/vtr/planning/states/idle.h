#pragma once

#include <vtr/planning/state_machine.h>

namespace vtr {
namespace planning {
namespace state {
#if 0
class BaseState;
class Event;
#endif
class Idle : public BaseState {
 public:
  PTR_TYPEDEFS(Idle)
  DEFAULT_COPY_MOVE(Idle)
  INHERITANCE_TESTS(Idle, Base)
  using Parent = BaseState;
#if 0
  using Base = Parent::Base;
  using BasePtr = Base::Ptr;
  using Tactic = Parent::Tactic;
#endif

  Idle() {}
  Idle(const Parent &parent) : Parent(parent) {}
  virtual ~Idle() {}

  /** \brief Gets an enum representing the type of pipeline that this state
   * requires.
   */
  virtual PipelineType pipeline() const { return PipelineType::Idle; }
  /** \brief Return a string representation of the state
   */
  virtual std::string name() const { return Parent::name() + "::Idle"; }
  /** \brief Get the next intermediate state, for when no direct transition is
   * possible.
   */
  virtual BasePtr nextStep(const Base *newState) const;
  /** \brief State through which we must always enter this meta-state
   */
  virtual BasePtr entryState(const Base *) const;
  /** \brief Check the navigation state and perform necessary state transitions
   */
  virtual void processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                            const Event &event = Event());
  /** \brief Called as a cleanup method when the state exits.  The base state
   * never exits.
   */
  virtual void onExit(Tactic *tactic, Base *newState);
  /** \brief Called as a setup method when the state is entered.  The base state
   * is never entered explicitly.
   */
  virtual void onEntry(Tactic *tactic, Base *oldState);
};

}  // namespace state
}  // namespace planning
}  // namespace vtr
