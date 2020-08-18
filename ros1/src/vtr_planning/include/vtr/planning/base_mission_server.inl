#pragma once

#include <vtr/planning/base_mission_server.h>
#if 0
#include <exception>
#include <thread>

#include <asrl/common/utils/ContainerTools.hpp>
#endif

namespace vtr {
namespace planning {

template <class GoalType>
BaseMissionServer<GoalType>::BaseMissionServer(const StateMachine::Ptr& state)
    : status_(ServerState::Paused),
      state_(state == nullptr ? StateMachine::InitialState() : state) {
  state_->setCallbacks(this);
}

template <class GoalType>
void BaseMissionServer<GoalType>::addGoal(const GoalType& goal, int idx) {
  LockGuard lck(lock_);

  if (asrl::common::utils::contains(goal_map_, Iface::id(goal))) {
    auto s = "Attempting to add a goal that already exists";
    LOG(ERROR) << s;
    throw std::invalid_argument(s);
  }

  if (idx <= 0) {
    goal_queue_.push_front(goal);
    goal_map_.insert({Iface::id(goal), goal_queue_.begin()});
  } else if (idx >= int(goal_queue_.size())) {
    goal_queue_.push_back(goal);
    goal_map_.insert({Iface::id(goal), --goal_queue_.end()});
  } else {
    // We have to iterate to get to position $idx, but a list is still better
    // here as it doesn't invalidate references on insertion/deletion
    auto it = goal_queue_.begin();
    for (int i = 0; i < idx; ++i) {
      ++it;
    }
    goal_map_.insert({Iface::id(goal), goal_queue_.insert(it, goal)});
  }

  // If we had nothing in the queue before and we are not paused, accept this
  // goal now
  if (status_ == ServerState::Empty) {
    status_ = ServerState::Processing;
    this->goalAccepted(goal);
  }
}

template <class GoalType>
void BaseMissionServer<GoalType>::addGoal(const GoalType& goal,
                                          const std::string& before) {
  LockGuard lck(lock_);

  if (asrl::common::utils::contains(goal_map_, Iface::id(goal))) {
    throw std::invalid_argument("Attempting to add a goal that already exists");
  }
  if (!asrl::common::utils::contains(goal_map_, before)) {
    throw std::invalid_argument(
        "Trying to add a goal before a goal that does not exist");
  }

  goal_map_.insert(
      {Iface::id(goal), goal_queue_.insert(goal_map_.at(before), goal)});

  // If we had nothing in the queue before and we are not paused, accept this
  // goal now
  if (status_ == ServerState::Empty) {
    status_ = ServerState::Processing;
    this->goalAccepted(goal);
  }
}

template <class GoalType>
void BaseMissionServer<GoalType>::cancelAll() {
  LockGuard lck(lock_);
  // We can't use a normal iterator, because the underlying list is being purged
  // as we iterate.  We need to call the function explicitly here, as a subclass
  // may need to perform extra tasks when a goal is cancelled
  while (goal_queue_.size() > 0) {
    this->goalCancelled(goal_queue_.back());
  }
}

#if 0
template <class GoalType>
void BaseMissionServer<GoalType>::cancelGoal(const std::string& id) {
  LockGuard lck(lock_);
  this->goalCancelled(*goal_map_[id]);
}

/// @brief Reorder the goal queue to match the order of the list of goal ids
/// provided
template <class GoalType>
void BaseMissionServer<GoalType>::reorderGoals(
    const std::list<std::string>& order) {
  LockGuard lck(lock_);

  if (order.size() != goal_queue_.size()) {
    LOG(ERROR) << "Goal reordering is missing some existing goals";
    throw std::invalid_argument(
        "Goal reordering is missing some existing goals");
  }

  std::list<GoalType> new_queue;

  for (auto&& it : order) {
    try {
      new_queue.push_back(*goal_map_.at(it));
    } catch (std::exception& e) {
      LOG(ERROR) << "Goal reordering list contianed unknown goal id";
      throw std::invalid_argument(
          "Goal reordering list contianed unknown goal id");
    }
  }

  goal_queue_ = new_queue;
  goal_map_.clear();

  // Swapping the entire goal queue *did* invalidate our references/iterators
  for (auto it = goal_queue_.begin(); it != goal_queue_.end(); ++it) {
    goal_map_.insert({Iface::id(*it), it});
  }
}

template <class GoalType>
void BaseMissionServer<GoalType>::moveGoal(const std::string& id, int idx) {
  LockGuard lck(lock_);

  if (!asrl::common::utils::contains(goal_map_, id)) {
    LOG(ERROR) << "Attempting to move a non-existent goal: " << id;
    throw std::invalid_argument("Attempting to move a non-existent goal");
  }

  GoalType target = *goal_map_.at(id);
  goal_queue_.erase(goal_map_.at(id));
  auto before = goal_queue_.begin();

  // Any negative index moves to the beginning, and any out of range index moves
  // to the end
  if (idx >= int(goal_queue_.size()) || idx < 0) {
    before = goal_queue_.end();
  } else {
    // We have to iterate to get to position $idx, but a list is still better
    // here as it doesn't invalidate references on insertion/deletion
    for (int i = 0; i < idx; ++i) {
      ++before;
    }
  }

  goal_map_[Iface::id(target)] = goal_queue_.insert(before, target);
}

template <class GoalType>
void BaseMissionServer<GoalType>::moveGoal(const std::string& id,
                                           const std::string& before) {
  LockGuard lck(lock_);

  if (!asrl::common::utils::contains(goal_map_, id)) {
    LOG(ERROR) << "Attempting to move a non-existent goal: " << id;
    throw std::invalid_argument("Attempting to move a non-existent goal");
  }
  if (!asrl::common::utils::contains(goal_map_, before)) {
    LOG(ERROR) << "Invalid move to before a non-existent goal: " << id
               << " --> " << before;
    throw std::invalid_argument(
        "Attempting to place before a non-existent goal");
  }
  if (id == before) {
    LOG(WARNING) << "Attempting to move goal " << id
                 << " to before itself... something is fishy";

    // Don't throw here, because the desired outcome WAS achieved, even if what
    // we were asked to do was weird
    return;
  }

  GoalType target = *goal_map_.at(id);
  goal_queue_.erase(goal_map_.at(id));
  goal_map_[Iface::id(target)] =
      goal_queue_.insert(goal_map_.at(before), target);
}
#endif

template <class GoalType>
void BaseMissionServer<GoalType>::setPaused(bool paused, bool async) {
  LockGuard lck(lock_);

  if (paused && status_ == ServerState::Empty) {
    // If there are no goals, we can just pause
    LOG(INFO) << "State: Empty --> Paused";
    status_ = ServerState::Paused;
  } else if (paused && status_ == ServerState::Processing) {
    // If we are processing a goal, finish it but don't start another
    LOG(INFO) << "State: Processing --> PendingPause";
    status_ = ServerState::PendingPause;
  } else if (!paused && status_ == ServerState::Paused) {
    if (goal_queue_.size() > 0) {
      // If there are goals left, resume processing them
      LOG(INFO) << "State: Paused --> Processing";
      status_ = ServerState::Processing;

      if (async) {
        // Check if we had a background task going already, and block if we do
        if (deferred_.valid()) {
          deferred_.get();
        }

        // Goal acceptance is launched asynchronously as it involves a pause
        deferred_ = std::async(std::launch::async, [&] {
          this->goalAccepted(goal_queue_.front());
        });
      } else {
        this->goalAccepted(goal_queue_.front());
      }
    } else {
      // If there are no goals left, flag the server as empty
      LOG(INFO) << "State: Paused --> Empty";
      status_ = ServerState::Empty;
    }
  } else if (!paused && status_ == ServerState::PendingPause) {
    // We have not actually paused yet, so cancel the pending pause
    LOG(INFO) << "State: PendingPause --> Processing";
    status_ = ServerState::Processing;
  }
}

template <class GoalType>
void BaseMissionServer<GoalType>::goalAccepted(GoalType goal) {
  lock_.lock();

  LOG(INFO) << "Pausing at start for: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   Iface::pauseBefore(goal))
                       .count() /
                   1000.f
            << "s";
  this->goalWaiting(goal);

  if (goalWaitStart_.valid()) {
    goalWaitStart_.get();
  }

  // Goal acceptance is launched asynchronously as it involves a pause
  goalWaitStart_ = std::async(std::launch::async, [this, goal]() {
    std::this_thread::sleep_for(Iface::pauseBefore(goal));
    LOG(INFO) << "Done pause; actually accepting the goal";

    // Don't lock before the pause, as it can be arbitrarily long
    LockGuard lck(this->lock_);

    // Clear waiting statuses
    this->finishAccept(goal);

    switch (Iface::target(goal)) {
      case Target::Idle:
        this->state_->handleEvents(Event::StartIdle());
        break;
      case Target::Teach:
        this->state_->handleEvents(Event::StartTeach());
        break;
      case Target::Repeat:
        this->state_->handleEvents(Event::StartRepeat(Iface::path(goal)));
        break;
      case Target::Merge:
        this->state_->handleEvents(
            Event::StartMerge(Iface::path(goal), Iface::vertex(goal)));
        break;
      case Target::Localize:
        this->state_->handleEvents(
            Event::StartLocalize(Iface::path(goal), Iface::vertex(goal)));
        break;
#if 0
      case Target::Learn:
        this->state_->handleEvents(Event::StartLearn());
        break;
      case Target::Loiter:
        this->state_->handleEvents(Event::StartLoiter());
        break;
      case Target::Return:
        this->state_->handleEvents(Event::StartReturn(Iface::path(goal)));
        break;
      case Target::Hover:
        this->state_->handleEvents(Event::StartHover());
        break;
#endif
      case Target::Unknown:
      default:
        throw std::runtime_error(
            "An unknown goal type made it's way into the BaseMissionServer... "
            "Why wasn't that handled?");
    }
  });

  lock_.unlock();
}
#if 0
/// @brief Callback when a new goal is rejected
template <class GoalType>
void BaseMissionServer<GoalType>::goalRejected(GoalType) {
  // This doesn't do anything at the base level
}
#endif
/// @brief Callback when the current goal completes successfully
template <class GoalType>
void BaseMissionServer<GoalType>::goalSucceeded() {
  lock_.lock();
  auto goal = goal_queue_.front();

  LOG(INFO) << "Pausing at end for: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   Iface::pauseAfter(goal))
                       .count() /
                   1000.f
            << "s";
  this->goalWaiting(goal);

  if (goalWaitEnd_.valid()) {
    goalWaitEnd_.get();
  }

  // Goal acceptance is launched asynchronously as it involves a pause
  goalWaitEnd_ = std::async(std::launch::async, [this, goal]() {
    std::this_thread::sleep_for(Iface::pauseAfter(goal));
    LOG(INFO) << "Done pause; actually removing goal";

    // Don't lock before the pause, as it can be arbitrarily long
    LockGuard lck(lock_);

    this->finishSuccess(goal);

    // Erase the completed goal from the queue
    this->goal_map_.erase(Iface::id(goal));
    this->goal_queue_.pop_front();

    if (this->status_ == ServerState::Processing) {
      // Keep processing goals as long as we have them and didn't request a
      // pause
      if (this->goal_queue_.empty()) {
        this->status_ = ServerState::Empty;
      } else {
        this->goalAccepted(this->goal_queue_.front());
      }
    } else if (this->status_ == ServerState::PendingPause) {
      // Pause between two goals if one has been requested
      this->status_ = ServerState::Paused;
    }
  });

  lock_.unlock();
}

template <class GoalType>
void BaseMissionServer<GoalType>::goalAborted(const std::string&) {
  LockGuard lck(lock_);

  // There was a error, so don't proceed until the user says we can go
  status_ = ServerState::Paused;

  // Erase the aborted goal from the queue
  goal_map_.erase(Iface::id(goal_queue_.front()));
  goal_queue_.pop_front();
}

template <class GoalType>
void BaseMissionServer<GoalType>::goalCancelled(GoalType goal) {
  LockGuard lck(lock_);

  bool was_active = Iface::id(goal) == Iface::id(goal_queue_.front());

  LOG(INFO) << "Cancelling goal: " << Iface::id(goal);

  goal_queue_.erase(goal_map_.at(Iface::id(goal)));
  goal_map_.erase(Iface::id(goal));

  if (was_active) {
    if (status_ == ServerState::Processing) {
      // Stop what we are doing and drop to idle.  Going to idle is important in
      // case we were moving.
      state_->handleEvents(Event::StartIdle());

      if (goal_queue_.empty()) {
        status_ = ServerState::Empty;
        LOG(INFO) << "Queue is empty; dropping to IDLE";
      } else {
        this->goalAccepted(goal_queue_.front());
        LOG(INFO) << "Accepting next goal: " << Iface::id(goal_queue_.front());
      }
    } else if (status_ == ServerState::PendingPause) {
      // Stop what we are doing and drop to idle.  Going to idle is important in
      // case we were moving.
      state_->handleEvents(Event::StartIdle());
      status_ = ServerState::Paused;
      LOG(INFO) << "State: PendingPause --> Paused";
    }
  }
}
/// @brief Callback when the state machine is finished executing a goal
template <class GoalType>
void BaseMissionServer<GoalType>::stateSuccess() {
  this->goalSucceeded();
}
/// @brief Callback when the state machine must abort a goal
template <class GoalType>
void BaseMissionServer<GoalType>::stateAbort(const std::string& msg) {
  this->goalAborted(msg);
}
#if 0
/// @brief Perform state checks and add a run
template <class GoalType>
void BaseMissionServer<GoalType>::addRun(bool extend) {
  if (status_ != ServerState::Empty && status_ != ServerState::Paused &&
      state_->name() != "::Idle") {
    throw std::runtime_error("Cannot add a run while a mission is active!");
  }

  bool ephemeral = false;
  state_->tactic()->addRun(ephemeral, extend);
}
#endif

}  // namespace planning
}  // namespace vtr
