#include <vtr_mission_planning/states/teach.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

auto Teach::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) return Parent::nextStep(newState);

  if (IsType(newState)) {
    // We are not allowed to transition directly to a meta-state
    std::stringstream ss;
    ss << "[Teach] Transitioning to the meta-state " << this->name()
       << " is not allowed";
    throw std::runtime_error(ss.str());
  }

  using namespace teach;

  // We can always transition to TopologicalLocalize, as it is the entry state.
  // Since we only get here when we are already in Teach, we can also move
  // directly to Branch.
  if (TopologicalLocalize::InChain(newState) || Branch::InChain(newState)) {
    return nullptr;
  }
  // We can go directly to Merge from Branch, but TopologicalLocalize must pass
  // through Branch to initialize things
  else if (Merge::InChain(newState)) {
    if (Branch::InChain(this))
      return nullptr;
    else if (TopologicalLocalize::InChain(this))
      return BasePtr(new Branch(*this));
  }

  // If we didn't hit one of the above cases, then something is wrong
  std::stringstream ss;
  ss << "[Teach] Invalid goal transition from " << this->name() << " to "
     << newState->name();
  throw std::runtime_error(ss.str());
}

auto Teach::entryState(const Base *) const -> BasePtr {
  Ptr rptr(new teach::TopologicalLocalize(*this));
  rptr->container_ = this->container_;
  return rptr;
}

void Teach::processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
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
      [[fallthrough]];
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}

void Teach::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  {
    auto lock = tactic->lockPipeline();
    // Ensure that everything is in one frame and there is no more active chain
    tactic->relaxGraph();
    tactic->saveGraph();

    // Update the cached privileged graph for planning
    container_->planner()->updatePrivileged();
  }
  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

void Teach::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves

  // Add a new run before we teach any paths, regardless of previous state
  addRunInternal_(true);
}

}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
