#include <vtr/planning/states/idle.h>

namespace vtr {
namespace planning {
namespace state {
// Idle::Idle() : Base(PipelinePtr(new PipelineType())) { }

/** \brief Get the next intermediate state, for when no direct transition is
 * possible
 */
auto Idle::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) {
    return Parent::nextStep(newState);
  }

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

/** \brief State through which we must always enter this meta-state
 */
auto Idle::entryState(const Base *) const -> BasePtr {
  Ptr rptr(new Idle());
  rptr->container_ = this->container_;
  return rptr;
}

/** \brief Check the navigation state and perform necessary state transitions
 */
void Idle::processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                        const Event &event) {
  switch (event.signal_) {
    case Signal::Continue:
      break;
    default:
      // All signals should be directly handled by the children they affect.  If
      // we have a signal here, pass it along to the base class to do the actual
      // goal swapping/error throwing
      return Parent::processGoals(tactic, goal_lock, event);
  }

  switch (event.type_) {
    case Action::Continue:
      // This state doesn't actually have any automatic transitions, so there is
      // nothing here

      // NOTE: the lack of a break statement here is intentional, to allow
      // unhandled cases to percolate up the chain
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}
/** \brief Called as a cleanup method when the state exits.  The base state
 * never exits.
 */
void Idle::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) {
    return;
  }

  // Idle does not do anything special when it exits

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

/** \brief Called as a setup method when the state is entered.  The base state
 * is never entered explicitly.
 */
void Idle::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) {
    return;
  }

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // Clear the path when we enter Idle
  tactic->setPath(PathType());
}
}  // namespace state
}  // namespace planning
}  // namespace vtr
