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
 * \file base_mission_server.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <future>
#include <list>
#include <mutex>
#include <unordered_map>

#include <vtr_common/utils/container_tools.hpp>
#include <vtr_mission_planning/event.hpp>
#include <vtr_mission_planning/state_machine.hpp>

namespace vtr {
namespace mission_planning {

using namespace std::chrono_literals;

using state::Event;
using state::StateMachine;

enum class Target : int8_t {
  Unknown = -1,
  Idle = 0,
  Teach = 1,
  Repeat = 2,
  Merge = 3,
  Localize = 4,
};

/**
 * \brief Template specialization is used to standardize the interface of goal
 * types we do not control.
 *
 * The default assumes that the goal is a simple struct. Specialize the template
 * if it isn't.
 */
template <class GoalHandle>
struct GoalInterface {
  using Id = typename std::remove_reference<decltype(GoalHandle::id)>::type;
  static Id id(const GoalHandle& gh) { return gh.id; }
  static Target target(const GoalHandle& gh) { return gh.target; }
  static std::list<VertexId> path(const GoalHandle& gh) { return gh.path; }
  static VertexId vertex(const GoalHandle& gh) { return gh.vertex; }
  static std::chrono::milliseconds pauseBefore(const GoalHandle& gh) {
    return gh.pause_before;
  }
  static std::chrono::milliseconds pauseAfter(const GoalHandle& gh) {
    return gh.pause_after;
  }
};

/** \brief Miniature state machine to allow graceful pause of goal execution */
enum class ServerState : int8_t {
  Empty = -1,      // No goals exist
  Processing = 0,  // We are working on 1 or more goals
  PendingPause,    // We are working on a goal, but will pause when done
  Paused           // Execution is paused, and will resume with more goals
};

/** \brief Base mission server that manages a list of goals */
template <class GoalHandle>
class BaseMissionServer : StateMachineCallbacks {
 public:
  using Iface = GoalInterface<GoalHandle>;
  using GoalIter = typename std::list<GoalHandle>::iterator;
  using LockGuard = std::lock_guard<std::recursive_mutex>;

  PTR_TYPEDEFS(BaseMissionServer)

  BaseMissionServer(const StateMachine::Ptr& sm = nullptr);

  virtual ~BaseMissionServer() {}

  /**
   * \brief Add a goal, with optional position in the queue (defaults to the
   * end)
   */
  void addGoal(const GoalHandle& gh, int idx = std::numeric_limits<int>::max());
  /** \brief Add a goal before an existing goal */
  void addGoal(const GoalHandle& gh, const typename Iface::Id& before);
  /** \brief Cancel a goal by id */
  void cancelGoal(const typename Iface::Id& id);
  /** \brief Cancels all goals */
  void cancelAll();
  /** \brief Reorder the goal queue to match the order of the goal id list */
  void reorderGoals(const std::list<typename Iface::Id>& order);
  /**
   * \brief Move a target goal to a new position in the queue. Defaults to the
   * end.
   */
  void moveGoal(const typename Iface::Id& id, int idx = -1);
  /** \brief Move a target goal before an existing goal in the queue */
  void moveGoal(const typename Iface::Id& id, const typename Iface::Id& before);

  /**
   * \brief Pause or un-pause the mission.
   * \note this does not halt the current goal.
   */
  void setPause(bool pause = true, bool async = true);

  /** \brief Get the current server status */
  ServerState status() const { return status_; }
#if 0
  /** \brief Get the goal currently being processed */
  GoalHandle currentGoal() const {
    return goal_queue_.size() > 0 ? *goal_queue_.front() : GoalHandle{};
  }
#endif
  /** \brief Get the state machine associated with this mission server */
  StateMachine::Ptr& stateMachine() { return state_machine_; }

  /** \brief Look up a goal by ID */
  GoalHandle& goal(const typename Iface::Id& id) { return *goal_map_.at(id); }
  /** \brief Check if we are already tracking a goal */
  bool isTracking(const typename Iface::Id& id) {
    LockGuard lck(lock_);
    return common::utils::contains(goal_map_, id);
  }
  /** \brief Get the first/active goal */
  GoalHandle& top() {
    LockGuard lck(lock_);
    return goal_queue_.front();
  }

  /** \brief Callback when the state machine must abort a goal */
  void stateAbort(const std::string& msg) override {
    LockGuard lck(lock_);
    abortGoal(top(), msg);
  }
  /** \brief Callback when the state machine changes state */
  void stateChanged(const state::BaseState::Ptr&) override {}
  /** \brief Callback when the state machine is finished executing a goal */
  void stateSuccess() override {
    LockGuard lck(lock_);
    transitionToNextGoal(top());
  }
  /** \brief Callback when the state machine registers progress on a goal */
  void stateUpdate(double) override {}

  /** \brief Kill all goals and pause the server */
  virtual void halt() {
    cancelAll();
    setPause(true, true);
  }

 protected:
  /** \brief Terminates the goal due to an internal error */
  virtual void abortGoal(GoalHandle gh, const std::string&);
  /** \brief Callback when an existing goal is cancelled by a user */
  virtual void cancelGoal(GoalHandle gh);
  /** \brief Callback when a new goal is executed (accepted) */
  virtual void executeGoal(GoalHandle gh);
  /** \brief Callback when a goal is finished executing (succeeded) */
  virtual void finishGoal(GoalHandle) {}
  /** \brief Finish waiting of previous goal and start executing the next */
  virtual void transitionToNextGoal(GoalHandle gh);
  /** \brief Set waiting state of a goal */
  virtual void setGoalStarted(GoalHandle) {}
  /** \brief Set waiting state of a goal */
  virtual void setGoalWaiting(GoalHandle, bool) {}

  /** \brief SimpleGoal processing queue */
  std::list<GoalHandle> goal_queue_;
  /** \brief Quick lookup map between id and SimpleGoal */
  std::unordered_map<typename Iface::Id, GoalIter> goal_map_;

  /**
   * \brief Prevent priority inversions with goal addition/completion
   * \details Goal execution and cancellation are launched from separate
   * threads.
   */
  std::recursive_mutex lock_;

 private:
  /** \brief Flag that lets us pause goal execution */
  ServerState status_;
  /** \brief Pointer to the underlying state machine that we control */
  typename StateMachine::Ptr state_machine_;
  /**
   * \brief A future to store deferred tasks that must be executed after a
   * function exits.
   */
  std::future<void> deferred_;
  /** \brief A future to allow adding goals with pauses without blocking */
  std::future<void> goalExecStart_;
  /** \brief A future to allow adding goals with pauses without blocking */
  std::future<void> goalExecEnd_;
};

}  // namespace mission_planning
}  // namespace vtr

#include <vtr_mission_planning/base_mission_server.inl>