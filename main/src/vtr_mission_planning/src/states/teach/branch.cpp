#include <vtr_mission_planning/states/teach/branch.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace teach {

auto Branch::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) return Parent::nextStep(newState);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void Branch::processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                          const Event &event) {
  switch (event.signal_) {
    case Signal::Continue:
      break;
#if 0
    case Signal::AttemptClosure: {
      // We need to perform a closure, so swap this state with the Merge state
      Event tmp(Action::SwapGoal, BasePtr(new teach::Merge(*this)));
      tmp.signal_ = event.signal_;
      return Parent::processGoals(tactic, goal_lock, tmp);
    }
#endif
    default:
      // Any unhandled signals percolate upwards
      return Parent::processGoals(tactic, goal_lock, event);
  }

  switch (event.type_) {
    case Action::Continue:
      // If a match window has been set, we are trying to merge but had to pass
      // through here first Result: End this goal, as we expect the next goal on
      // the stack to be Merge
      if (matchWindow_.size() > 0) {
        Event tmp(Action::EndGoal);
        LOG(INFO) << "[::Teach::Branch] Exiting due to requested merge";
        return Parent::processGoals(tactic, goal_lock, tmp);
      }
      if (false /* \todo (Old) Check for user end conditions? Maybe needs another interupt event... */) {
        return Parent::processGoals(tactic, goal_lock, Event(Action::EndGoal));
      }
      // NOTE: the lack of a break statement here is intentional, to allow
      // unhandled cases to percolate up the chain
      [[fallthrough]];
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}

void Branch::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  // TODO: Exit from repeating metastate

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

void Branch::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
  // TODO: The stuff
}

}  // namespace teach
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
