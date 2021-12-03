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
 * \file mission_server.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning_v2/state_machine/state_machine.hpp"

namespace vtr {
namespace mission_planning {

enum class GoalState : int8_t {
  Empty = 0,
  Starting = 1,
  Running = 2,
  Finishing = 3,
};

/** \brief Miniature state machine to allow graceful pause of goal execution */
enum class ServerState : int8_t {
  Empty = -1,      // No goals exist
  Processing = 0,  // We are working on 1 or more goals
  PendingPause,    // We are working on a goal, but will pause when done
  Paused           // Execution is paused, and will resume with more goals
};

enum class GoalTarget : int8_t {
  Unknown = -1,
  Idle = 0,
  Teach = 1,
  Repeat = 2,
};

enum class CommandTarget : int8_t {
  Unknown = -1,
  Localize = 0,
  StartMerge = 1,
  ConfirmMerge = 2,
  ContinueTeach = 3,
};

struct Command {
  using VertexId = tactic::VertexId;
  CommandTarget target;
  VertexId vertex;
  std::vector<VertexId> path;
};

/** \brief Template specialization to standardize the goal interface */
template <class GoalHandle>
class GoalInterface {
 public:
  // clang-format off
  using Id = typename std::remove_reference<decltype(GoalHandle::id)>::type;
  static const Id& InvalidId() {
    static Id invalid_id{-1};
    return invalid_id;
  }
  static Id id(const GoalHandle& gh) { return gh.id; }
  static GoalTarget target(const GoalHandle& gh) { return gh.target; }
  static std::list<tactic::VertexId> path(const GoalHandle& gh) { return gh.path; }
  static std::chrono::milliseconds pause_before(const GoalHandle& gh) { return gh.pause_before; }
  static std::chrono::milliseconds pause_after(const GoalHandle& gh) { return gh.pause_after; }
  // clang-format on
};

template <class GoalHandle>
class MissionServer : public StateMachineCallback {
 public:
  PTR_TYPEDEFS(MissionServer);

  using GoalId = typename GoalInterface<GoalHandle>::Id;

  using Mutex = std::mutex;
  using CondVar = std::condition_variable;
  using LockGuard = std::lock_guard<Mutex>;
  using UniqueLock = std::unique_lock<Mutex>;

  MissionServer();
  virtual ~MissionServer();

  /** \brief RAII does not work due to circular dependency with state macine */
  void start(const StateMachineInterface::Ptr& state_machine);

  /** \brief Pause or un-pause the mission. (without halting current goal) */
  void setPause(const bool pause = true);

  /** \brief Goal interfaces */
  void addGoal(const GoalHandle& gh, int idx = std::numeric_limits<int>::max());
  void cancelGoal(const GoalHandle& gh);
#if false
  void cancelAllGoals();
#endif
  bool hasGoal(const GoalId& goal_id);

  /** \brief Command interface */
  void processCommand(const Command& command);

  /** \brief State machine callbacks */
  void stateSuccess() override;
  void stateUpdate(const double progress) override;

 protected:
  /** \brief Derived class must call this upon destruction */
  void stop();

 private:
  /** \brief wait and start the current goal */
  void startGoal();
  /** \brief wait and finish the current goal */
  void finishGoal();

 private:
  virtual void serverStateChanged(const ServerState&) {}
  virtual void goalStarted(const GoalHandle&) {}
  virtual void goalWaiting(const GoalHandle&, bool) {}
  virtual void goalFinished(const GoalHandle&) {}
  virtual void goalCanceled(const GoalHandle&) {}
  virtual void goalUpdated(const GoalHandle&, const double) {}

 private:
  void clearGoals();
  void clearCurrentGoal();

  /** \brief Protects all class members */
  mutable Mutex mutex_;
  /** \brief wait until stop or current goal has changed */
  mutable CondVar cv_stop_or_goal_changed_;
  mutable CondVar cv_thread_finish_;

  StateMachineInterface::Ptr state_machine_ = nullptr;

  /** \brief SimpleGoal processing queue */
  std::list<GoalId> goal_queue_;
  /** \brief Quick lookup map between id and SimpleGoal */
  std::unordered_map<GoalId, GoalHandle> goal_map_;

  /** \brief signal the process thread to stop */
  bool stop_ = false;

  /** \brief tracking the current goal */
  GoalId current_goal_id_ = GoalInterface<GoalHandle>::InvalidId();
  GoalState current_goal_state_ = GoalState::Empty;
  ServerState current_server_state_ = ServerState::Empty;

  size_t thread_count_ = 0;
  /** \brief thread to wait and then start the current goal */
  std::thread goal_starting_thread_;
  /** \brief thread to wait and then finishing the current goal */
  std::thread goal_finishing_thread_;
};

template <class GoalHandle>
MissionServer<GoalHandle>::MissionServer() {
  thread_count_ = 2;
  goal_starting_thread_ =
      std::thread(&MissionServer<GoalHandle>::startGoal, this);
  goal_finishing_thread_ =
      std::thread(&MissionServer<GoalHandle>::finishGoal, this);
}

template <class GoalHandle>
MissionServer<GoalHandle>::~MissionServer() {
  stop();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::start(
    const StateMachineInterface::Ptr& state_machine) {
  LockGuard lock(mutex_);
  state_machine_ = state_machine;
}

template <class GoalHandle>
void MissionServer<GoalHandle>::stop() {
  UniqueLock lock(mutex_);
  //
  clearGoals();
  //
  stop_ = true;
  cv_stop_or_goal_changed_.notify_all();
  cv_thread_finish_.wait(lock, [&] { return thread_count_ == 0; });
  if (goal_starting_thread_.joinable()) goal_starting_thread_.join();
  if (goal_finishing_thread_.joinable()) goal_finishing_thread_.join();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::setPause(const bool pause) {
  LockGuard lock(mutex_);

  if (pause) {
    if (current_server_state_ == ServerState::Empty) {
      CLOG(INFO, "mission.server") << "State: Empty --> Paused";
      current_server_state_ = ServerState::Paused;
    } else if (current_server_state_ == ServerState::Processing) {
      CLOG(INFO, "mission.server") << "State: Processing --> PendingPause";
      current_server_state_ = ServerState::PendingPause;
    }
  } else {
    if (current_server_state_ == ServerState::Paused) {
      if (goal_queue_.size() > 0) {
        // If there are goals left, resume processing them
        CLOG(INFO, "mission.server") << "State: Paused --> Processing";
        current_goal_id_ = goal_queue_.front();
        current_goal_state_ = GoalState::Starting;
        current_server_state_ = ServerState::Processing;
        cv_stop_or_goal_changed_.notify_all();
      } else {
        // If there are no goals left, flag the server as empty
        CLOG(INFO, "mission.server") << "State: Paused --> Empty";
        current_server_state_ = ServerState::Empty;
      }
    } else if (current_server_state_ == ServerState::PendingPause) {
      CLOG(INFO, "mission.server") << "State: PendingPause --> Processing";
      current_server_state_ = ServerState::Processing;
    }
  }
}

template <class GoalHandle>
void MissionServer<GoalHandle>::addGoal(const GoalHandle& gh, int idx) {
  LockGuard lock(mutex_);

  const auto goal_id = GoalInterface<GoalHandle>::id(gh);

  if (goal_map_.find(goal_id) != goal_map_.end()) {
    CLOG(ERROR, "mission.server")
        << "Goal with id " << goal_id << " already exists";
    throw std::runtime_error("Trying to add a goal that already exists");
  }

  if (idx <= 0) {
    goal_queue_.push_front(goal_id);
    goal_map_.insert({goal_id, gh});
  } else if (idx >= int(goal_queue_.size())) {
    goal_queue_.push_back(goal_id);
    goal_map_.insert({goal_id, gh});
  } else {
    auto iter = goal_queue_.begin();
    for (int i = 0; i < idx; ++i) ++iter;
    goal_map_.insert({goal_id, gh});
  }

  // check if only goal
  if (current_server_state_ == ServerState::Empty) {
    // consistency check
    if (goal_queue_.size() != 1) {
      std::stringstream ss;
      ss << "Goal queue size is " << goal_queue_.size() << " but should be 1";
      CLOG(ERROR, "mission.server") << ss.str();
      throw std::runtime_error(ss.str());
    }
    current_goal_id_ = goal_queue_.front();
    current_goal_state_ = GoalState::Starting;
    current_server_state_ = ServerState::Processing;
    cv_stop_or_goal_changed_.notify_all();
  }
}

template <class GoalHandle>
void MissionServer<GoalHandle>::cancelGoal(const GoalHandle& gh) {
  LockGuard lock(mutex_);

  const auto goal_id = GoalInterface<GoalHandle>::id(gh);

  for (auto iter = goal_queue_.begin(); iter != goal_queue_.end(); ++iter) {
    if (*iter == goal_id) {
      goal_queue_.erase(iter);
      break;
    }
  }
  goal_map_.erase(goal_id);

  if (current_goal_id_ == goal_id) clearCurrentGoal();
}

#if false
template <class GoalHandle>
void MissionServer<GoalHandle>::cancelAllGoals() {
  LockGuard lock(mutex_);
  clearGoals();
}
#endif

template <class GoalHandle>
bool MissionServer<GoalHandle>::hasGoal(const GoalId& goal_id) {
  LockGuard lock(mutex_);
  return goal_map_.find(goal_id) != goal_map_.end();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::processCommand(const Command& command) {
  LockGuard lock(mutex_);
  switch (command.target) {
    case CommandTarget::Localize: {
      state_machine_->handle(Event::StartIdle(command.vertex));
      return;
    }
    case CommandTarget::StartMerge: {
      state_machine_->handle(Event::StartMerge(command.path));
      return;
    }
    case CommandTarget::ConfirmMerge: {
      state_machine_->handle(std::make_shared<Event>(Signal::AttemptClosure));
      return;
    }
    case CommandTarget::ContinueTeach: {
      state_machine_->handle(std::make_shared<Event>(Signal::ContinueTeach));
      return;
    }
    default:
      CLOG(ERROR, "mission.server") << "Unknown command encountered.";
      throw std::runtime_error("Unknown command encountered.");
      return;
  }
}

template <class GoalHandle>
void MissionServer<GoalHandle>::stateSuccess() {
  LockGuard lock(mutex_);

  // consistency check
  if (current_goal_id_ == GoalInterface<GoalHandle>::InvalidId()) {
    std::string err{
        "State success called but no goal is currently being processed."};
    CLOG(ERROR, "mission.server") << err;
    throw std::runtime_error(err);
  }
  if (current_goal_state_ != GoalState::Running) {
    std::string err{"State success called but goal is not in running state."};
    CLOG(ERROR, "mission.server") << err;
    throw std::runtime_error(err);
  }
  if (current_server_state_ != ServerState::Processing &&
      current_server_state_ != ServerState::PendingPause) {
    std::string err{
        "State success called but server is not in processing state."};
    CLOG(ERROR, "mission.server") << err;
    throw std::runtime_error(err);
  }

  current_goal_state_ = GoalState::Finishing;
  cv_stop_or_goal_changed_.notify_all();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::stateUpdate(const double progress) {
  LockGuard lock(mutex_);
  goalUpdated(goal_map_.at(current_goal_id_), progress);
}

template <class GoalHandle>
void MissionServer<GoalHandle>::clearCurrentGoal() {
  if (current_goal_id_ == GoalInterface<GoalHandle>::InvalidId()) return;

  // consistency check
  if ((current_server_state_ != ServerState::PendingPause) &&
      current_server_state_ != ServerState::Processing) {
    std::stringstream ss;
    ss << "Server state not in processing or pending pause when there is a "
          "goal being processed";
    CLOG(ERROR, "mission.server") << ss.str();
    throw std::runtime_error(ss.str());
  }
  if (current_goal_state_ == GoalState::Empty) {
    std::stringstream ss;
    ss << "Current goal state is empty when there is a goal being processed";
    CLOG(ERROR, "mission.server") << ss.str();
    throw std::runtime_error(ss.str());
  }

  // no matter what state the state machine is in, drop back to idle
  state_machine_->handle(Event::Reset());

  if (current_server_state_ == ServerState::PendingPause) {
    current_goal_id_ = GoalInterface<GoalHandle>::InvalidId();
    current_goal_state_ = GoalState::Empty;
    current_server_state_ = ServerState::Paused;
  } else {
    if (goal_queue_.empty()) {
      current_goal_id_ = GoalInterface<GoalHandle>::InvalidId();
      current_goal_state_ = GoalState::Empty;
      current_server_state_ = ServerState::Empty;
    } else {
      current_goal_id_ = goal_queue_.front();
      current_goal_state_ = GoalState::Starting;
    }
  }
  cv_stop_or_goal_changed_.notify_all();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::clearGoals() {
  goal_queue_.clear();
  goal_map_.clear();

  if (current_goal_id_ != GoalInterface<GoalHandle>::InvalidId())
    clearCurrentGoal();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::startGoal() {
  el::Helpers::setThreadName("mission.server.goal_starting");
  CLOG(INFO, "mission.server") << "Starting the finish goal thread.";
  while (true) {
    UniqueLock lock(mutex_);
    //
    cv_stop_or_goal_changed_.wait(lock, [&] {
      return stop_ ||
             (current_goal_id_ != GoalInterface<GoalHandle>::InvalidId() &&
              current_goal_state_ == GoalState::Starting);
    });
    if (stop_) {
      CLOG(INFO, "mission.server") << "Stopping the start goal thread.";
      --thread_count_;
      cv_thread_finish_.notify_all();
      return;
    }

    //
    const auto current_goal_id = current_goal_id_;
    const auto current_goal = goal_map_.at(current_goal_id);

    CLOG(INFO, "mission.server") << "Starting goal " << current_goal_id;
    goalStarted(current_goal);

    goalWaiting(current_goal, true);
    cv_stop_or_goal_changed_.wait_for(
        lock, GoalInterface<GoalHandle>::pause_before(current_goal),
        [&] { return stop_ || (current_goal_id_ != current_goal_id); });
    if (current_goal_id_ != current_goal_id) continue;
    goalWaiting(current_goal, false);

    // start the current goal
    switch (GoalInterface<GoalHandle>::target(current_goal)) {
      case GoalTarget::Idle:
        state_machine_->handle(Event::StartIdle());
        break;
      case GoalTarget::Teach:
        state_machine_->handle(Event::StartTeach());
        break;
      case GoalTarget::Repeat: {
        const auto path = GoalInterface<GoalHandle>::path(current_goal);
        state_machine_->handle(Event::StartRepeat(path));
        break;
      }
      default:
        throw std::runtime_error("Unknown goal target");
    }
    current_goal_state_ = GoalState::Running;
  }
}

template <class GoalHandle>
void MissionServer<GoalHandle>::finishGoal() {
  el::Helpers::setThreadName("mission.server.goal_finishing");
  CLOG(INFO, "mission.server") << "Starting the finish goal thread.";
  while (true) {
    UniqueLock lock(mutex_);

    //
    cv_stop_or_goal_changed_.wait(lock, [&] {
      return stop_ ||
             (current_goal_id_ != GoalInterface<GoalHandle>::InvalidId() &&
              current_goal_state_ == GoalState::Finishing);
    });
    if (stop_) {
      CLOG(INFO, "mission.server") << "Stopping the finish goal thread.";
      --thread_count_;
      cv_thread_finish_.notify_all();
      return;
    }

    //
    const auto current_goal_id = current_goal_id_;
    const auto current_goal = goal_map_.at(current_goal_id);

    goalWaiting(current_goal, true);
    cv_stop_or_goal_changed_.wait_for(
        lock, GoalInterface<GoalHandle>::pause_before(current_goal),
        [&] { return stop_ || (current_goal_id_ != current_goal_id); });
    if (current_goal_id_ != current_goal_id) continue;
    goalWaiting(current_goal, false);

    CLOG(INFO, "mission.server")
        << "Done pause; finishing goal " << current_goal_id;
    goalFinished(current_goal);

    // Erase the completed goal from the queue
    for (auto iter = goal_queue_.begin(); iter != goal_queue_.end(); ++iter) {
      if (*iter == current_goal_id) {
        goal_queue_.erase(iter);
        break;
      }
    }
    goal_map_.erase(current_goal_id);

    if (current_server_state_ == ServerState::PendingPause) {
      current_goal_id_ = GoalInterface<GoalHandle>::InvalidId();
      current_goal_state_ = GoalState::Empty;
      current_server_state_ = ServerState::Paused;
    } else if (current_server_state_ == ServerState::Processing) {
      if (goal_queue_.empty()) {
        current_goal_id_ = GoalInterface<GoalHandle>::InvalidId();
        current_goal_state_ = GoalState::Empty;
        current_server_state_ = ServerState::Empty;
      } else {
        current_goal_id_ = goal_queue_.front();
        current_goal_state_ = GoalState::Starting;
      }
    }
    cv_stop_or_goal_changed_.notify_all();
  }
}

}  // namespace mission_planning
}  // namespace vtr