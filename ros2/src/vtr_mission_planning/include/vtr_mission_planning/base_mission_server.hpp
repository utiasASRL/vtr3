#pragma once

#include <future>
#include <list>
#include <mutex>
#include <unordered_map>

#include <vtr_common/utils/container_tools.hpp>
#include <vtr_mission_planning/event.hpp>
#include <vtr_mission_planning/state_machine.hpp>
#if 0
#include <chrono>

#include <asrl/common/logging.hpp>
#include <asrl/common/utils/CommonMacros.hpp>
#include <asrl/pose_graph/id/GraphId.hpp>
#endif

namespace vtr {
namespace mission_planning {

#if 0
using vtr::pose_graph::VertexId;
#endif
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

/** \brief Miniature state machine to allow graceful pausing of goal execution
 */
enum class ServerState : int8_t {
  Empty = -1,      // No goals exist
  Processing = 0,  // We are working on 1 or more goals
  PendingPause,    // We are working on a goal, but will pause when done
  Paused           // Execution is paused, and will resume with more goals
};

/** \brief Base mission server that manages a list of goals
 */
template <class GoalHandle>
class BaseMissionServer : StateMachineCallbacks {
 public:
#if 0
  DEFAULT_COPY_MOVE(BaseMissionServer)
#endif

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
  /** \brief Cancels all goals */
  void cancelAll();
  /** \brief Cancel a goal by id */
  void cancelGoal(const typename Iface::Id& id);
  /**
   * \brief Reorder the goal queue to match the order of the list of goal ids
   * provided.
   */
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
  /** \brief Get the state machine associated with this mission server
   */
  StateMachine::Ptr& stateMachine() { return state_machine_; }
#endif
  /**
   * \brief Callback when a new goal is in a waiting state
   * Nothing to do in the base class
   */
  virtual void goalWaiting(GoalHandle) {}
  /** \brief Callback when a new goal is accepted */
  virtual void goalAccepted(GoalHandle gh);
  /**
   * \brief Callback when a new goal is rejected
   * Nothing to do in the base class
   */
  virtual void goalRejected(GoalHandle) {}
  /** \brief Callback when the current goal completes successfully */
  virtual void goalSucceeded();
  /**
   * \brief Callback when the current goal terminates due to an internal error
   */
  virtual void goalAborted(const std::string&);
  /** \brief Callback when an existing goal is cancelled by a user */
  virtual void goalCanceled(GoalHandle gh);

  /** \brief Look up a goal by ID */
  GoalHandle& goal(const typename Iface::Id& id) { return *goal_map_.at(id); }

  /** \brief Check if we are already tracking a goal */
  bool isTracking(const typename Iface::Id& id) {
    LockGuard lck(lock_);
    return common::utils::contains(goal_map_, id);
  }

  /** \brief Get the first/active goal */
  GoalHandle& top() { return goal_queue_.front(); }

  /** \brief Callback when the state machine must abort a goal */
  void stateAbort(const std::string& msg) override { goalAborted(msg); }
  /** \brief Callback when the state machine changes state */
  void stateChanged(const state::BaseState::Ptr&) override {}
  /** \brief Callback when the state machine is finished executing a goal */
  void stateSuccess() override { goalSucceeded(); }
  /** \brief Callback when the state machine registers progress on a goal */
  void stateUpdate(double) override {}

  /** \brief Kill all goals and pause the server */
  virtual void halt() {
    cancelAll();
    setPause(true, true);
  }
#if 0
  /** \brief Perform state checks and add a run */
  void addRun(bool extend = false);
#endif
 protected:
  /** \brief Callback when a goal is finished waiting */
  virtual void finishAccept(GoalHandle) {}
  /** \brief Callback when a goal is finished waiting at the end */
  virtual void finishSuccess(GoalHandle) {}

  /** \brief SimpleGoal processing queue */
  std::list<GoalHandle> goal_queue_;
  /** \brief Quick lookup map between id and SimpleGoal */
  std::unordered_map<typename Iface::Id, GoalIter> goal_map_;

#if 0
  /**
   * \brief Flag to indicate that the server is waiting for a predefined goal
   * pause.
   */
  bool waiting_;
#endif

  /**
   * \brief Mutex to prevent priority inversions with goal addition/completion
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
  std::future<void> goalWaitStart_;
  /** \brief A future to allow adding goals with pauses without blocking */
  std::future<void> goalWaitEnd_;
};

}  // namespace mission_planning
}  // namespace vtr

#include <vtr_mission_planning/base_mission_server.inl>