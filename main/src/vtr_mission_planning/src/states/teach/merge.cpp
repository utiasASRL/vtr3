// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file merge.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_mission_planning/states/teach/merge.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

namespace teach {

auto Merge::nextStep(const Base *newState) const -> BasePtr {
  // If where we are going is not a child, delegate to the parent
  if (!InChain(newState)) return Parent::nextStep(newState);

  // If we aren't changing to a different chain, there is no intermediate step
  return nullptr;
}

void Merge::processGoals(Tactic *tactic, UpgradableLockGuard &goal_lock,
                         const Event &event) {
  switch (event.signal_) {
    case Signal::Continue:
      break;
    case Signal::SwitchMergeWindow: {
      auto goal = std::static_pointer_cast<Merge>(event.goal_);
      setTarget(goal->matchWindow_, goal->targetVertex_);
      tactic->setPath(matchWindow_);
      return Parent::processGoals(tactic, goal_lock, Event());
    }
    case Signal::AttemptClosure: {
      bool canClose = tactic->canCloseLoop();
      if (canClose) {
        cancelled_ = false;
        Event tmp(Action::EndGoal);
        tmp.signal_ = event.signal_;
        return Parent::processGoals(tactic, goal_lock, tmp);
      } else {
        cancelled_ = true;
        // NOTE: We intentionally fall through to the next case to go back into
        // Merge mode on failure
        [[fallthrough]];
      }
    }
    case Signal::ContinueTeach: {
      // We are done merging but want to keep teaching, so swap this state with
      // the Branch state
      Event tmp(Action::SwapGoal, BasePtr(new teach::Branch()));
      tmp.signal_ = event.signal_;
      return Parent::processGoals(tactic, goal_lock, tmp);
    }
    default:
      // Any unhandled signals percolate upwards
      return Parent::processGoals(tactic, goal_lock, event);
  }

  switch (event.type_) {
    case Action::AppendGoal:
    case Action::NewGoal:
    case Action::Abort:
      cancelled_ = true;
      // NOTE: the lack of a break statement here is intentional, to allow
      // unhandled cases to percolate up the chain
      [[fallthrough]];
    default:
      // Delegate all goal swapping/error handling to the base class
      return Parent::processGoals(tactic, goal_lock, event);
  }
}

bool Merge::canCloseLoop_(Tactic *tactic) {
  auto status = tactic->status();

  // Offset in the x, y, and yaw directions
  auto &T = tactic->targetLoc().T;
  double dx = T.r_ba_ina()(0), dy = T.r_ba_ina()(1), dt = T.vec()(5);

  if (status.localization_ == LocalizationStatus::Confident &&
      status.targetLocalization_ == LocalizationStatus::Confident) {
    if (dx > 0.5 || dy > 0.25 || dt > 0.2) {
      LOG(WARNING) << "Offset from path is too large to merge (x, y, th): "
                   << dx << ", " << dy << " " << dt;
      return false;
    } else {
      LOG(INFO) << "Localization is good; adding closure!";
      return true;
    }
  } else {
    LOG(WARNING) << "Cannot merge due to lack of successful localization";
    return false;
  }
}

void Merge::onExit(Tactic *tactic, Base *newState) {
  // If the new target is a derived class, we are not exiting
  if (InChain(newState)) return;

  // Note: This is called *before* we call up the tree, as we destruct from
  // leaves to root If we localized, add a loop closure to whatever match we
  // found.  Otherwise, do nothing.
  if (!cancelled_) {
    tactic->connectToTrunk(true, true);
  } else {
    LOG(INFO) << "Not merging due to localization conditions/goal termination";
  }

  // Clear the path for merging
  tactic->setPath(PathType());

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onExit(tactic, newState);
}

void Merge::onEntry(Tactic *tactic, Base *oldState) {
  // If the previous state was a derived class, we did not leave
  if (InChain(oldState)) return;

  // Recursively call up the inheritance chain until we get to the least common
  // ancestor
  Parent::onEntry(tactic, oldState);

  // Note: This is called after we call up the tree, as we construct from root
  // to leaves
  tactic->setPath(matchWindow_);

  // Reset this in case we re-enter the same instance of this goal
  cancelled_ = true;
}

}  // namespace teach
}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
