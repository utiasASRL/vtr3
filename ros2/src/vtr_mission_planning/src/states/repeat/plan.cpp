#include <vtr_mission_planning/states/repeat/plan.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace repeat {

auto Plan::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) return Parent::nextStep(newState);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void Plan::processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                        const Event &event) {
  switch (event.signal_) {
    case Signal::Continue:
      break;
#if 0
    case Signal::RobotLost: {
      // We need to reinitialize our location, so push a TopoLoc goal onto the
      // stack as a temporary detour
      Event tmp(Action::AppendGoal,
                BasePtr(new repeat::TopologicalLocalize(*this)));
      tmp.signal_ = event.signal_;
      return Parent::processGoals(tactic, goal_lock, tmp);
    }
    case Signal::CantPlan: {
      // We cannot come up with a plan to the goal, so drop everything and stop
      Event tmp(Action::NewGoal, BasePtr(new Idle()));
      tmp.signal_ = event.signal_;
      return Parent::processGoals(tactic, goal_lock, tmp);
    }
    case Signal::PlanSuccess: {
      // We have found a plan, so pop this goal from the stack
      Event tmp(Action::EndGoal);
      tmp.signal_ = event.signal_;
      return Parent::processGoals(tactic, goal_lock, tmp);
    }
    case Signal::DoRepair: {
      // Path repair has been requested; add a Merge goal as a temporary detour
      Event tmp(Action::AppendGoal, BasePtr(new teach::Branch()));
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
      if (!tactic->persistentLoc().v.isSet()) {
        // If we are lost, redo topological localization
        return Parent::processGoals(
            tactic, goal_lock,
            Event(Action::AppendGoal,
                  BasePtr(new repeat::TopologicalLocalize(*this))));
      } else if (tactic->status().localization_ ==
                 LocalizationStatus::DeadReckoning) {
        [[fallthrough]];  /// \todo (yuchen) block needed? for now it gets rid
                          /// of the fall through warning.
        // If we are dead-reckoning, try to relocalize
        // TODO: Need to incorporate state history here to avoid infinite loops
        //        return Parent::processGoals(tactic, goal_lock,
        //        Event(Action::EndGoal));
      } else {
        // If we are localized, then continue on to repeat
        return Parent::processGoals(tactic, goal_lock, Event(Action::EndGoal));
      }
      // NOTE: the lack of a break statement here is intentional, to allow
      // unhandled cases to percolate up the chain
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}

void Plan::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root
  if (repeat::MetricLocalize::InChain(newState)) {
    std::stringstream ss;
    for (auto &&it : this->waypoints_) {
      ss << it << ", ";
    }

    LOG(INFO) << "Current vertex: " << tactic->persistentLoc().v
              << "  Waypoints: " << ss.str();

    PathType path = this->container_->planner()->path(
        tactic->persistentLoc().v, this->waypoints_, &this->waypointSeq_);
    tactic->setPath(path, true);
    this->container_->callbacks()->stateUpdate(0);
  }
  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

void Plan::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
  // TODO: The stuff
}

}  // namespace repeat
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr