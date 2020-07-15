#pragma once

#include <future>
#include <list>
#include <mutex>
#include <unordered_map>

#include <vtr/planning/event.h>
#include <vtr/planning/state_machine.h>
#if 0
#include <chrono>

#include <asrl/common/logging.hpp>
#include <asrl/common/utils/CommonMacros.hpp>
#include <asrl/common/utils/ContainerTools.hpp>
#include <asrl/pose_graph/id/GraphId.hpp>
#endif

namespace vtr {
namespace planning {

#if 0
using asrl::pose_graph::VertexId;
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
#if 0
  // Lancaster specific
  Learn = 5,
  Return = 6,
  Loiter = 7,
  // multicopter specific
  Hover = 8
#endif
};

/** \brief Template specialization is used to standardize the interface of goal
 * types we do not control.
 *
 * The default assumes that the goal is a simple struct. Specialize the template
 * if it isn't.
 */
template <class GoalHandle>
struct GoalInterface {
  static inline std::string id(const GoalHandle& gh) { return gh.id_; }
  static inline Target target(const GoalHandle& gh) { return gh.target_; }
  static inline std::list<VertexId> path(const GoalHandle& gh) {
    return gh.path_;
  }
  static inline VertexId vertex(const GoalHandle& gh) { return gh.vertex_; }
  static inline std::chrono::milliseconds pauseBefore(const GoalHandle& gh) {
    return gh.pauseBefore_;
  }
  static inline std::chrono::milliseconds pauseAfter(const GoalHandle& gh) {
    return gh.pauseAfter_;
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
template <class GoalType>
class BaseMissionServer : StateMachineCallbacks {
 public:
#if 0
  PTR_TYPEDEFS(BaseMissionServer)
  DEFAULT_COPY_MOVE(BaseMissionServer)
#endif

  using Iface = GoalInterface<GoalType>;
  using GoalIter = typename std::list<GoalType>::iterator;
  using LockGuard = std::lock_guard<std::recursive_mutex>;

  BaseMissionServer(const StateMachine::Ptr& state = nullptr);
  virtual ~BaseMissionServer() {}

  /** \brief Add a goal, with optional position in the queue (defaults to the
   * end)
   */
  void addGoal(const GoalType& goal, int idx = std::numeric_limits<int>::max());

  /** \brief Add a goal before an existing goal
   */
  void addGoal(const GoalType& goal, const std::string& before);

  /** \brief Cancels all goals
   */
  void cancelAll();

#if 0
  /** \brief Cancel a goal by id
   */
  void cancelGoal(const std::string& id);

  /** \brief Reorder the goal queue to match the order of the list of goal ids provided.
   */
  void reorderGoals(const std::list<std::string>& order);

  /** \brief Move a target goal to a new position in the queue.  Defaults to the end.
   */
  void moveGoal(const std::string& id, int idx = -1);

  /** \brief Move a target goal before an existing goal in the queue
   */
  void moveGoal(const std::string& id, const std::string& before);
#endif

  /** \brief Pause or un-pause the mission.  NOTE: this does not halt the
   * current goal.
   */
  void setPaused(bool paused = true, bool async = true);
  /** \brief Get the current server status
   */
  inline ServerState status() const { return status_; }
#if 0
  /** \brief Get the goal currently being processed
   */
  inline GoalType currentGoal() const {
    return goal_queue_.size() > 0 ? *goal_queue_.front() : GoalType();
  }
  /** \brief Get the state machine associated with this mission server
   */
  inline StateMachine::Ptr& stateMachine() { return state_; }
#endif
  /** \brief Callback when a new goal is in a waiting state
   */
  virtual void goalWaiting(GoalType) { /*Nothing to do in the base class*/
  }
  /** \brief Callback when a new goal is accepted
   */
  virtual void goalAccepted(GoalType goal);
#if 0
  /** \brief Callback when a new goal is rejected
   */
  virtual void goalRejected(GoalType);
#endif
  /** \brief Callback when the current goal completes successfully
   */
  virtual void goalSucceeded();
  /** \brief Callback when the current goal terminates due to an internal error
   */
  virtual void goalAborted(const std::string&);
  /** \brief Callback when an existing goal is cancelled by a user
   */
  virtual void goalCancelled(GoalType goal);

  /** \brief Look up a goal by ID
   */
  inline GoalType& goal(const std::string& id) { return *goal_map_.at(id); }

  /** \brief Check if we are already tracking a goal
   */
  inline bool isTracking(const std::string& id) {
    return asrl::common::utils::contains(goal_map_, id);
  }
  /** \brief Get the first/active goal
   */
  inline GoalType& top() { return goal_queue_.front(); }
  /** \brief Callback when the state machine is finished executing a goal
   */
  virtual void stateSuccess();
  /** \brief Callback when the state machine must abort a goal
   */
  virtual void stateAbort(const std::string& msg);
  /** \brief Kill all goals and pause the server
   */
  virtual void halt() {
    this->cancelAll();
    this->setPaused(true, true);
  }
#if 0
  /** \brief Perform state checks and add a run
   */
  void addRun(bool extend = false);
#endif
 protected:
  /** \brief Callback when a goal is finished waiting
   */
  virtual void finishAccept(GoalType) {}
  /** \brief Callback when a goal is finished waiting at the end
   */
  virtual void finishSuccess(GoalType) {}

  /** \brief SimpleGoal processing queue
   */
  std::list<GoalType> goal_queue_;
  /** \brief Quick lookup map between id and SimpleGoal
   */
  std::unordered_map<std::string, GoalIter> goal_map_;

#if 0
  /** \brief Flag to indicate that the server is waiting for a predefined goal pause.
   */
  bool waiting_;
#endif

  /** \brief Mutex to prevent priority inversions with goal addition/completion
   */
  std::recursive_mutex lock_;

 private:
  /** \brief Flag that lets us pause goal execution
   */
  ServerState status_;
  /** \brief Pointer to the underlying state machine that we control
   */
  typename StateMachine::Ptr state_;

  /** \brief A future to store deferred tasks that must be executed after a
   * function exits.
   */
  std::future<void> deferred_;

  /** \brief A future to allow adding goals with pauses without blocking
   */
  std::future<void> goalWaitStart_;

  /** \brief A future to allow adding goals with pauses without blocking
   */
  std::future<void> goalWaitEnd_;
};

}  // namespace planning
}  // namespace vtr

#include <vtr/planning/base_mission_server.inl>

// struct SimpleGoal {
//  typedef std::chrono::milliseconds msecs;
//
//  SimpleGoal(const Target& target, const msecs& before = msecs(0), const
//  msecs& after = msecs(0)) :
//      id_(std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch()
//      / std::chrono::nanoseconds(1))), target_(target), before_(before),
//      after_(after) { }
//
//  SimpleGoal(const Target& target, const std::list<VertexId>& path =
//  std::list<VertexId>(),
//             const msecs& before = msecs(0), const msecs& after = msecs(0)) :
//      id_(std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch()
//      / std::chrono::nanoseconds(1))), target_(target), path_(path),
//      before_(before), after_(after) { }
//
//  SimpleGoal(std::string id, const Target& target, const std::list<VertexId>&
//  path = std::list<VertexId>(),
//             const msecs& before = msecs(0), const msecs& after = msecs(0)) :
//      id_(id), target_(target), path_(path), before_(before), after_(after) {
//      }
//
//  std::string id_;
//  Target target_;
//  std::list<VertexId> path_;
//  std::chrono::milliseconds before_;
//  std::chrono::milliseconds after_;
//};
