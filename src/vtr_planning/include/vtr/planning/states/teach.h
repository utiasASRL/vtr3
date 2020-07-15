#pragma once

#include <vtr/planning/state_machine.h>

namespace vtr {
namespace planning {
namespace state {
#if 0
class BaseState;
class Event;
#endif
class Teach : public BaseState {
 public:
  PTR_TYPEDEFS(Teach)
  DEFAULT_COPY_MOVE(Teach)
  INHERITANCE_TESTS(Teach, Base)
  using Parent = BaseState;
#if 0 
  using Base = Parent::Base;
  using BasePtr = Base::Ptr;
  using Tactic = Parent::Tactic;
#endif

  Teach(const Base &base = Base()) : Parent(base) {}
  virtual ~Teach() {}

  /** \brief Return a string representation of the state
   */
  virtual std::string name() const { return Parent::name() + "::Teach"; }
  /** \brief Get the next intermediate state, for when no direct transition is
   * possible
   */
  virtual BasePtr nextStep(const Base *newState) const;
  /** \brief State through which we must always enter this meta-state
   */
  virtual BasePtr entryState(const Base *) const;

  /** \brief Set the target to match against
   */
  void setTarget(const std::vector<VertexId> &matchWindow,
                 const VertexId &targetVertex) {
    matchWindow_ = matchWindow;
    targetVertex_ = targetVertex;
  }

  /** \brief Pure Virtual, check the navigation state and perform necessary
   * state transitions
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

 protected:
  /** \brief Window of vertices to search against for a match/localization
   */
  std::vector<VertexId> matchWindow_;

  /** \brief Target vertex to rejoin to/start mapping from
   */
  VertexId targetVertex_;
};
}  // namespace state
}  // namespace planning
}  // namespace vtr

// We chain includes here for convenience
#include <vtr/planning/states/teach/branch.h>
#include <vtr/planning/states/teach/merge.h>
#include <vtr/planning/states/teach/topological_localize.h>
