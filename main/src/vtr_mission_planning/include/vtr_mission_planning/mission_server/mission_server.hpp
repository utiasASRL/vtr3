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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning/state_machine/state_machine.hpp"

namespace vtr {
namespace mission_planning {

enum class GoalState : int8_t {
  Empty = 0,
  Starting = 1,
  Running = 2,
  Finishing = 3,
};
std::ostream& operator<<(std::ostream& os, const GoalState& goal_state);

/** \brief Miniature state machine to allow graceful pause of goal execution */
enum class ServerState : int8_t {
  Empty = -1,      // No goals exist
  Processing = 0,  // We are working on 1 or more goals
  PendingPause,    // We are working on a goal, but will pause when done
  Paused,          // Execution is paused, and will resume with more goals
  Crashed          // Main process has died notify GUI
};
std::ostream& operator<<(std::ostream& os, const ServerState& server_state);

enum class GoalTarget : int8_t {
  Unknown = -1,
  Idle = 0,
  Teach = 1,
  Repeat = 2,
  Localize = 3,
  SelectController = 4,
};
std::ostream& operator<<(std::ostream& os, const GoalTarget& goal_target);

enum class CommandTarget : int8_t {
  Unknown = -1,
  Localize = 0,
  StartMerge = 1,
  ConfirmMerge = 2,
  ContinueTeach = 3,
  ForceAddVertex = 4,
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
  using IdHash = std::hash<Id>;

  static const Id& InvalidId() {
    static Id invalid_id{-1};
    return invalid_id;
  }
  static Id id(const GoalHandle& gh) { return gh.id; }
  static GoalTarget target(const GoalHandle& gh) { return gh.target; }
  static std::list<tactic::VertexId> path(const GoalHandle& gh) { return gh.path; }
  static std::chrono::milliseconds pause_before(const GoalHandle& gh) { return gh.pause_before; }
  static std::chrono::milliseconds pause_after(const GoalHandle& gh) { return gh.pause_after; }
  static std::string controller_name(const GoalHandle& gh) { return gh.controller_name;  }
  // clang-format on
};

template <class GoalHandle>
class MissionServer : public StateMachineCallback {
 public:
  PTR_TYPEDEFS(MissionServer);

  using GoalId = typename GoalInterface<GoalHandle>::Id;
  using GoalIdHash = typename GoalInterface<GoalHandle>::IdHash;

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

  /** \brief Goal and command interfaces */
  void addGoal(const GoalHandle& gh, int idx = std::numeric_limits<int>::max());
  void cancelGoal(const GoalHandle& gh);
  void beginGoals();
  bool hasGoal(const GoalId& goal_id);
  void processCommand(const Command& command);

  /** \brief State machine callbacks */
  void stateSuccess() override;

 protected:
  /** \brief Derived class must call this upon destruction */
  void stop();

 private:
  /** \brief wait and start the current goal */
  void startGoal();
  /** \brief wait and finish the current goal */
  void finishGoal();

  /// helper functions, they do not acquire lock inside function body, so the
  /// lock must be acquired before calling them
 private:
  bool clearCurrentGoal();
  virtual void serverStateChanged();

 private:
  /// \note state_machine_ is only assigned once in start(), so not made
  /// thread-safe
  StateMachineInterface::Ptr getStateMachine() const;
  StateMachineInterface::WeakPtr state_machine_;

 protected:
  /** \brief Protects all class members */
  mutable Mutex mutex_;
  /** \brief wait until stop or current goal has changed */
  mutable CondVar cv_stop_or_goal_changed_;
  mutable CondVar cv_thread_finish_;

  /** \brief SimpleGoal processing queue */
  std::list<GoalId> goal_queue_;
  /** \brief Quick lookup map between id and SimpleGoal */
  std::unordered_map<GoalId, GoalHandle, GoalIdHash> goal_map_;
  /** \brief tracking the current goal */
  GoalId current_goal_id_ = GoalInterface<GoalHandle>::InvalidId();
  GoalState current_goal_state_ = GoalState::Empty;
  ServerState current_server_state_ = ServerState::Empty;

 private:
  /** \brief signal the process thread to stop */
  bool stop_ = false;
  /** \brief used to wait until all threads finish */
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
  state_machine_ = state_machine;
  // initialize server state of subclasses
  LockGuard lock(mutex_);
  serverStateChanged();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::stop() {
  UniqueLock lock(mutex_);
  //
  goal_queue_.clear();
  goal_map_.clear();
  const auto reset_sm = clearCurrentGoal();
  //
  stop_ = true;
  cv_stop_or_goal_changed_.notify_all();
  cv_thread_finish_.wait(lock, [&] { return thread_count_ == 0; });
  if (goal_starting_thread_.joinable()) goal_starting_thread_.join();
  if (goal_finishing_thread_.joinable()) goal_finishing_thread_.join();

  current_server_state_ = mission_planning::ServerState::Crashed;
  CLOG(DEBUG, "mission.server") << "Setting state to crashed for closing";

  //
  serverStateChanged();
  lock.unlock();
  if (!reset_sm) return;
  if (const auto state_machine = getStateMachine())
    state_machine->handle(Event::Reset());
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
  //
  serverStateChanged();
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
  //
  serverStateChanged();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::cancelGoal(const GoalHandle& gh) {
  UniqueLock lock(mutex_);
  const auto goal_id = GoalInterface<GoalHandle>::id(gh);
  //
  for (auto iter = goal_queue_.begin(); iter != goal_queue_.end(); ++iter) {
    if (*iter == goal_id && current_goal_id_ != goal_id) {
      goal_queue_.erase(iter);
      goal_map_.erase(goal_id);
      break;
    }
  }
  bool needs_reset = clearCurrentGoal();
  
  serverStateChanged();
  lock.unlock();
  if (const auto state_machine = getStateMachine()){
    if (needs_reset) {
      state_machine->handle(Event::Reset());
    } else if (current_goal_id_ == goal_id) {
      state_machine->handle(Event::EndGoal());
    }
  }
    
}

template <class GoalHandle>
void MissionServer<GoalHandle>::beginGoals() {
  LockGuard lock(mutex_);

  // check if this is the only goal running
  if (current_server_state_ == ServerState::Empty) {
    current_goal_id_ = goal_queue_.front();
    current_goal_state_ = GoalState::Starting;
    current_server_state_ = ServerState::Processing;
    cv_stop_or_goal_changed_.notify_all();
  }
  //
  serverStateChanged();
}

template <class GoalHandle>
bool MissionServer<GoalHandle>::hasGoal(const GoalId& goal_id) {
  LockGuard lock(mutex_);
  return goal_map_.find(goal_id) != goal_map_.end();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::processCommand(const Command& command) {
  /// \note state_machine handle must *not* be called within mission server
  /// lock, otherwise deadlock
  /// consider the case: getStateMachine()->stateSuccess() tries to lock mission
  /// server mutex, while this function acquires the mission server mutex but is
  /// blocked at getStateMachine()->handle
  const auto state_machine = getStateMachine();
  if (state_machine == nullptr) {
    std::string err{"State machine is nullptr, cannot process command."};
    CLOG(ERROR, "mission.server") << err;
    throw std::runtime_error(err);
  };
  switch (command.target) {
    case CommandTarget::Localize: {
      state_machine->handle(Event::StartIdle(command.vertex));
      return;
    }
    case CommandTarget::StartMerge: {
      state_machine->handle(Event::StartMerge(command.path));
      return;
    }
    case CommandTarget::ConfirmMerge: {
      state_machine->handle(std::make_shared<Event>(Signal::AttemptClosure));
      return;
    }
    case CommandTarget::ContinueTeach: {
      state_machine->handle(std::make_shared<Event>(Signal::ContinueTeach));
      return;
    }
    case CommandTarget::ForceAddVertex: {
      state_machine->handle(std::make_shared<Event>(Action::ForceAddVertex));
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
  serverStateChanged();

  cv_stop_or_goal_changed_.notify_all();
}

template <class GoalHandle>
void MissionServer<GoalHandle>::startGoal() {
  el::Helpers::setThreadName("mission.server.goal_starting");
  CLOG(INFO, "mission.server") << "Starting the start goal thread.";
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

    CLOG(INFO, "mission.server")
        << "Starting goal " << current_goal_id << " with waiting time "
        << GoalInterface<GoalHandle>::pause_before(current_goal).count();
    cv_stop_or_goal_changed_.wait_for(
        lock, GoalInterface<GoalHandle>::pause_before(current_goal),
        [&] { return stop_ || (current_goal_id_ != current_goal_id); });
    if (current_goal_id_ != current_goal_id) {
      CLOG(INFO, "mission.server") << "Goal has changed, stop waiting.";
      continue;
    }
    CLOG(INFO, "mission.server") << "Actually start goal " << current_goal_id;

    current_goal_state_ = GoalState::Running;
    serverStateChanged();

    // state machine handle must not be called within mission server lock
    lock.unlock();

    // start the current goal
    const auto state_machine = getStateMachine();
    if (state_machine == nullptr) {
      std::string err{"State machine is nullptr, cannot start the goal."};
      CLOG(ERROR, "mission.server") << err;
      throw std::runtime_error(err);
    };
    switch (GoalInterface<GoalHandle>::target(current_goal)) {
      case GoalTarget::Idle:
        state_machine->handle(Event::StartIdle());
        break;
      case GoalTarget::Teach:
        state_machine->handle(Event::StartTeach());
        break;
      case GoalTarget::Repeat: {
        const auto path = GoalInterface<GoalHandle>::path(current_goal);
        state_machine->handle(Event::StartRepeat(path));
        break;
      }
      case GoalTarget::Localize: {
        state_machine->handle(Event::StartLocalize());
        break;
      }
      case GoalTarget::SelectController: {
        state_machine->handle(Event::SwitchController(GoalInterface<GoalHandle>::controller_name(current_goal)));
        break;
      }
      default:
        throw std::runtime_error("Unknown goal target " +
                                 std::to_string(int(GoalInterface<GoalHandle>::target(current_goal))));
    }
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

    CLOG(INFO, "mission.server")
        << "Finishing goal " << current_goal_id << " with waiting time "
        << GoalInterface<GoalHandle>::pause_after(current_goal).count();
    cv_stop_or_goal_changed_.wait_for(
        lock, GoalInterface<GoalHandle>::pause_after(current_goal),
        [&] { return stop_ || (current_goal_id_ != current_goal_id); });
    if (current_goal_id_ != current_goal_id) {
      CLOG(INFO, "mission.server") << "Goal has changed, stop waiting.";
      continue;
    }
    CLOG(INFO, "mission.server")
        << "Done waiting; finishing goal " << current_goal_id;

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
    serverStateChanged();

    cv_stop_or_goal_changed_.notify_all();
  }
}

template <class GoalHandle>
bool MissionServer<GoalHandle>::clearCurrentGoal() {
  if (current_goal_id_ == GoalInterface<GoalHandle>::InvalidId()) return false;

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
      // current_goal_id_ = goal_queue_.front();
      // current_goal_state_ = GoalState::Starting;
      return false;
    }
  }
  cv_stop_or_goal_changed_.notify_all();

  return true;  // state machine needs reset
}

template <class GoalHandle>
void MissionServer<GoalHandle>::serverStateChanged() {
  CLOG(INFO, "mission.server")
      << "Server state changed:" << std::endl
      << "server state - " << current_server_state_ << std::endl
      << "current goal id - " << current_goal_id_ << std::endl
      << "current goal state - " << current_goal_state_ << std::endl
      << "goal queue - " << goal_queue_;
}

template <class GoalHandle>
StateMachineInterface::Ptr MissionServer<GoalHandle>::getStateMachine() const {
  if (auto state_machine_acquired = state_machine_.lock())
    return state_machine_acquired;
  else {
    std::string err{"state machine has expired"};
    CLOG(WARNING, "mission.server") << err;
  }
  return nullptr;
}

}  // namespace mission_planning
}  // namespace vtr