#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_mission_planning/states/repeat/metric_localize.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace repeat {

auto MetricLocalize::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) return Parent::nextStep(newState);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void MetricLocalize::processGoals(Tactic *tactic,
                                  UpgradableLockGuard &goal_lock,
                                  const Event &event) {
  switch (event.signal_) {
    case Signal::Continue:
      break;
#if 0
    case Signal::LocalizeObs: {
      // We cannot localize here, so detour back to planning
      Event tmp(Action::AppendGoal, BasePtr(new repeat::Plan()));
      tmp.signal_ = event.signal_;
      return Parent::processGoals(tactic, goal_lock, tmp);
    }
    case Signal::Localized: {
      // We have successfully localized, so pop this goal from the stack
      Event tmp(Action::EndGoal);
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
      // For now we can exit as long as the localization status is Confident
      if (/* tactic->status().localization_ == LocalizationStatus::Confident &&
           */ /// \todo add this confidence flag back
          tactic->persistentLoc().successes > 5) {
        LOG(INFO) << "Metric localization is successful, existing the state.";
        tactic->connectToTrunk(false);
        return Parent::processGoals(tactic, goal_lock, Event(Action::EndGoal));
      } else {
        auto T = tactic->persistentLoc().T;
        double ex = -1, ey = -1, et = -1;
        if (T.covarianceSet()) {
          ex = std::sqrt(T.cov()(0, 0));
          ey = std::sqrt(T.cov()(1, 1));
          et = std::sqrt(T.cov()(5, 5));
        }
        LOG(INFO) << "Not exiting; state: "
                  << int(tactic->status().localization_)
                  << ", loc count: " << int(tactic->persistentLoc().successes)
                  << ", std dev: " << ex << ", " << ey << ", " << et;
      }
      // NOTE: the lack of a break statement here is intentional, to allow
      // unhandled cases to percolate up the chain
      [[fallthrough]];
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}

void MetricLocalize::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) return;

  // Clean up any temporary runs we added
  //  tactic->removeEphemeralRuns();

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

void MetricLocalize::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // If we do not have a confident localization, add an ephemeral run so we can
  // run localization
  // tactic->incrementLocCount(-5);
  //  if (tactic->status().localization_ != LocalizationStatus::Confident) {
  //    tactic->addRun(true);
  //  }
}

}  // namespace repeat
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
