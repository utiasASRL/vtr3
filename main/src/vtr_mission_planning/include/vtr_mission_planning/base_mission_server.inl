#pragma once

#include <chrono>
#include <thread>
#include <vtr_mission_planning/base_mission_server.hpp>
#if 0
#include <exception>

#include <asrl/common/utils/ContainerTools.hpp>
#endif

namespace vtr {
namespace mission_planning {

template <class GoalHandle>
BaseMissionServer<GoalHandle>::BaseMissionServer(const StateMachine::Ptr& sm)
    : status_(ServerState::Paused),
      state_machine_(sm == nullptr ? StateMachine::InitialState() : sm) {
  state_machine_->setCallbacks(this);
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::addGoal(const GoalHandle& gh, int idx) {
  LockGuard lck(lock_);

  if (common::utils::contains(goal_map_, Iface::id(gh))) {
    std::string err{"Attempting to add a goal that already exists"};
    LOG(ERROR) << err;
    throw std::invalid_argument(err);
  }

  if (idx <= 0) {
    goal_queue_.push_front(gh);
    goal_map_.insert({Iface::id(gh), goal_queue_.begin()});
  } else if (idx >= int(goal_queue_.size())) {
    goal_queue_.push_back(gh);
    goal_map_.insert({Iface::id(gh), --goal_queue_.end()});
  } else {
    // We have to iterate to get to position $idx, but a list is still better
    // here as it doesn't invalidate references on insertion/deletion
    auto it = goal_queue_.begin();
    for (int i = 0; i < idx; ++i) ++it;
    goal_map_.insert({Iface::id(gh), goal_queue_.insert(it, gh)});
  }

  // If we had nothing in the queue before and we are not paused, accept this
  // goal now
  if (status_ == ServerState::Empty) {
    status_ = ServerState::Processing;
    executeGoal(gh);
  }
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::addGoal(const GoalHandle& gh,
                                            const typename Iface::Id& before) {
  LockGuard lck(lock_);

  if (common::utils::contains(goal_map_, Iface::id(gh)))
    throw std::invalid_argument("Attempting to add a goal that already exists");
  if (!common::utils::contains(goal_map_, before))
    throw std::invalid_argument(
        "Trying to add a goal before a goal that does not exist");

  goal_map_.insert(
      {Iface::id(gh), goal_queue_.insert(goal_map_.at(before), gh)});

  // If we had nothing in the queue before and we are not paused, accept this
  // gh now
  if (status_ == ServerState::Empty) {
    status_ = ServerState::Processing;
    executeGoal(gh);
  }
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::cancelGoal(const typename Iface::Id& id) {
  LockGuard lck(lock_);
  cancelGoal(*goal_map_[id]);
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::cancelAll() {
  LockGuard lck(lock_);
  // We can't use a normal iterator, because the underlying list is being purged
  // as we iterate.  We need to call the function explicitly here, as a subclass
  // may need to perform extra tasks when a goal is cancelled
  while (goal_queue_.size() > 0) cancelGoal(goal_queue_.back());
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::reorderGoals(
    const std::list<typename Iface::Id>& order) {
  LockGuard lck(lock_);

  if (order.size() != goal_queue_.size()) {
    std::string err{"Goal reordering is missing some existing goals"};
    LOG(ERROR) << err;
    throw std::invalid_argument(err);
  }

  std::list<GoalHandle> new_queue;
  for (auto&& it : order) {
    try {
      new_queue.push_back(*goal_map_.at(it));
    } catch (std::exception& e) {
      std::string err{"Goal reordering list contianed unknown goal id"};
      LOG(ERROR) << err;
      throw std::invalid_argument(err);
    }
  }
  goal_queue_ = new_queue;

  // Swapping the entire goal queue *did* invalidate our references/iterators
  goal_map_.clear();
  for (auto it = goal_queue_.begin(); it != goal_queue_.end(); ++it) {
    goal_map_.insert({Iface::id(*it), it});
  }
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::moveGoal(const typename Iface::Id& id,
                                             int idx) {
  LockGuard lck(lock_);

  if (!common::utils::contains(goal_map_, id)) {
    std::string err{"Attempting to move a non-existent goal"};
    LOG(ERROR) << err << ": " << id;
    throw std::invalid_argument(err);
  }

  auto target = *goal_map_.at(id);
  goal_queue_.erase(goal_map_.at(id));

  // Any negative index moves to the beginning, and any out of range index moves
  // to the end
  auto before = goal_queue_.begin();
  if (idx >= int(goal_queue_.size()) || idx < 0) {
    before = goal_queue_.end();
  } else {
    // We have to iterate to get to position $idx, but a list is still better
    // here as it doesn't invalidate references on insertion/deletion
    for (int i = 0; i < idx; ++i) ++before;
  }

  goal_map_[Iface::id(target)] = goal_queue_.insert(before, target);
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::moveGoal(const typename Iface::Id& id,
                                             const typename Iface::Id& before) {
  LockGuard lck(lock_);

  if (!common::utils::contains(goal_map_, id)) {
    std::string err{"Attempting to move a non-existent goal"};
    LOG(ERROR) << err << ": " << id;
    throw std::invalid_argument(err);
  }
  if (!common::utils::contains(goal_map_, before)) {
    std::string err{"Attempting to place before a non-existent goal"};
    LOG(ERROR) << err << ": " << id << " --> " << before;
    throw std::invalid_argument(err);
  }
  if (id == before) {
    LOG(WARNING) << "Attempting to move goal " << id
                 << " to before itself... something is fishy";

    // Don't throw here, because the desired outcome WAS achieved, even if what
    // we were asked to do was weird
    return;
  }

  auto target = *goal_map_.at(id);
  goal_queue_.erase(goal_map_.at(id));
  goal_map_[Iface::id(target)] =
      goal_queue_.insert(goal_map_.at(before), target);
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::setPause(bool pause, bool async) {
  LockGuard lck(lock_);

  if (pause && status_ == ServerState::Empty) {
    // If there are no goals, we can just pause
    LOG(INFO) << "State: Empty --> Paused";
    status_ = ServerState::Paused;
  } else if (pause && status_ == ServerState::Processing) {
    // If we are processing a goal, finish it but don't start another
    LOG(INFO) << "State: Processing --> PendingPause";
    status_ = ServerState::PendingPause;
  } else if (!pause && status_ == ServerState::Paused) {
    if (goal_queue_.size() > 0) {
      // If there are goals left, resume processing them
      LOG(INFO) << "State: Paused --> Processing";
      status_ = ServerState::Processing;

      if (async) {
        // Check if we had a background task going already, and block if we do
        if (deferred_.valid()) deferred_.get();

        // Goal acceptance is launched asynchronously as it involves a pause
        deferred_ = std::async(std::launch::async,
                               [&] { executeGoal(goal_queue_.front()); });
      } else {
        executeGoal(goal_queue_.front());
      }
    } else {
      // If there are no goals left, flag the server as empty
      LOG(INFO) << "State: Paused --> Empty";
      status_ = ServerState::Empty;
    }
  } else if (!pause && status_ == ServerState::PendingPause) {
    // We have not actually paused yet, so cancel the pending pause
    LOG(INFO) << "State: PendingPause --> Processing";
    status_ = ServerState::Processing;
  }
}

#if 0
template <class GoalHandle>
void BaseMissionServer<GoalHandle>::addRun(bool extend) {
  if (status_ != ServerState::Empty && status_ != ServerState::Paused &&
      state_machine_->name() != "::Idle") {
    throw std::runtime_error("Cannot add a run while a mission is active!");
  }

  bool ephemeral = false;
  state_machine_->tactic()->addRun(ephemeral, extend);
}
#endif

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::abortGoal(GoalHandle gh,
                                              const std::string&) {
  LockGuard lck(lock_);

  // There was a error, so don't proceed until the user says we can go
  status_ = ServerState::Paused;

  // Erase the aborted goal from the queue and map
  goal_queue_.erase(goal_map_.at(Iface::id(gh)));
  goal_map_.erase(Iface::id(gh));
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::cancelGoal(GoalHandle gh) {
  LockGuard lck(lock_);
  LOG(INFO) << "Canceling goal: " << Iface::id(gh);

  bool was_active = (Iface::id(gh) == Iface::id(goal_queue_.front()));

  goal_queue_.erase(goal_map_.at(Iface::id(gh)));
  goal_map_.erase(Iface::id(gh));

  if (was_active) {
    if (status_ == ServerState::Processing) {
      // Stop what we are doing and drop to idle. Going to idle is important in
      // case we were moving.
      state_machine_->handleEvents(Event::Reset());

      if (goal_queue_.empty()) {
        status_ = ServerState::Empty;
        LOG(INFO) << "Queue is empty; staying in IDLE";
      } else {
        executeGoal(goal_queue_.front());
        LOG(INFO) << "Accepting next goal: " << Iface::id(goal_queue_.front());
      }
    } else if (status_ == ServerState::PendingPause) {
      // Stop what we are doing and drop to idle.  Going to idle is important in
      // case we were moving.
      state_machine_->handleEvents(Event::Reset());
      status_ = ServerState::Paused;
      LOG(INFO) << "State: PendingPause --> Paused";
    }
  }
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::executeGoal(GoalHandle gh) {
  LockGuard lck(lock_);

  setGoalStarted(gh);

  LOG(INFO) << "Pausing at start for: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   Iface::pauseBefore(gh))
                       .count() /
                   1000.f
            << "s";

  setGoalWaiting(gh, true);

  // std::async does not work as returned future blocks the destructor.
  std::thread th([this, gh]() {
    // Don't lock before the pause, as it can be arbitrarily long
    auto start = std::chrono::system_clock::now();
    // The goal may be canceled during the waiting, so check periodically.
    while (std::chrono::system_clock::now() - start < Iface::pauseBefore(gh)) {
      std::this_thread::sleep_for(std::min(Iface::pauseBefore(gh), 100ms));
      LockGuard lck(lock_);
      if (!isTracking(Iface::id(gh))) {
        LOG(INFO) << "Goal already canceled.";
        return;
      }
    }

    if (goalExecStart_.valid()) goalExecStart_.get();
    goalExecStart_ = std::async(std::launch::async, [this, gh]() {
      LockGuard lck(lock_);
      // The goal may have been canceled.
      if (!isTracking(Iface::id(gh))) {
        LOG(INFO) << "Goal already canceled.";
        return;
      }

      // Clear waiting status
      setGoalWaiting(gh, false);
      LOG(INFO) << "Done pause; actually accepting the goal";

      switch (Iface::target(gh)) {
        case Target::Idle:
          state_machine_->handleEvents(Event::StartIdle());
          break;
        case Target::Teach:
          state_machine_->handleEvents(Event::StartTeach());
          break;
        case Target::Repeat:
          state_machine_->handleEvents(Event::StartRepeat(Iface::path(gh)));
          break;
        case Target::Merge:
          state_machine_->handleEvents(
              Event::StartMerge(Iface::path(gh), Iface::vertex(gh)));
          break;
        case Target::Localize:
          state_machine_->handleEvents(
              Event::StartLocalize(Iface::path(gh), Iface::vertex(gh)));
          break;
        case Target::Unknown:
        default:
          throw std::runtime_error(
              "An unknown goal type made it's way into the "
              "BaseMissionServer... "
              "Why wasn't that handled?");
      }
    });
  });
  th.detach();
}

template <class GoalHandle>
void BaseMissionServer<GoalHandle>::transitionToNextGoal(GoalHandle gh) {
  LockGuard lck(lock_);

  LOG(INFO) << "Pausing at end for: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   Iface::pauseAfter(gh))
                       .count() /
                   1000.f
            << "s";
  setGoalWaiting(gh, true);

  std::thread th([this, gh]() {
    // Don't lock before the pause, as it can be arbitrarily long
    auto start = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() - start < Iface::pauseAfter(gh)) {
      std::this_thread::sleep_for(std::min(Iface::pauseAfter(gh), 100ms));
      LockGuard lck(lock_);
      // The goal may have been canceled.
      if (!isTracking(Iface::id(gh))) {
        LOG(INFO) << "Goal already canceled.";
        return;
      }
    }

    if (goalExecEnd_.valid()) goalExecEnd_.get();
    goalExecEnd_ = std::async(std::launch::async, [this, gh]() {
      LockGuard lck(lock_);

      // The goal may have been canceled.
      if (!isTracking(Iface::id(gh))) {
        LOG(INFO) << "Goal already canceled.";
        return;
      }

      setGoalWaiting(gh, false);
      LOG(INFO) << "Done pause; actually finishing goal";
      finishGoal(gh);

      // Erase the completed goal from the queue
      goal_queue_.erase(goal_map_.at(Iface::id(gh)));
      goal_map_.erase(Iface::id(gh));

      if (status_ == ServerState::Processing) {
        // Keep processing goals as long as we have them and didn't request a
        // pause
        if (goal_queue_.empty())
          status_ = ServerState::Empty;
        else
          executeGoal(goal_queue_.front());
      } else if (status_ == ServerState::PendingPause) {
        // Pause between two goals if one has been requested
        status_ = ServerState::Paused;
      }
    });
  });
  th.detach();
}

}  // namespace mission_planning
}  // namespace vtr
