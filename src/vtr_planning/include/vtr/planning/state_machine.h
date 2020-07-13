#pragma once

#include <boost/thread/shared_mutex.hpp>
#include <mutex>
#if 0
#include <list>
#include <ostream>
#include <sstream>
#include <typeinfo>
#endif

#include <vtr/planning/event.h>
#include <vtr/planning/state_machine_interface.h>

#if 0
#include <asrl/common/logging.hpp>
#include <asrl/common/utils/CommonMacros.hpp>
#include <asrl/pose_graph/id/GraphId.hpp>

#include <asrl/planning/PlanningInterface.hpp>
#endif

namespace vtr {
namespace planning {
namespace state {

#if 0
// class Event;

enum class Signal : int8_t;
enum class Action : int8_t;
#endif

class StateMachine;

/** \brief Base State.  Implements high level logic and is the ancestor of all
/// other states.
 */
class BaseState {
 public:
  PTR_TYPEDEFS(BaseState)
  DEFAULT_COPY_MOVE(BaseState)

  using Base = BaseState;

  using Tactic = StateMachineInterface;
  using UpgradableLockGuard = boost::upgrade_lock<boost::shared_mutex>;
#if 0
  using GoalStackPtr = std::shared_ptr<std::list<Ptr> >;
#endif

  BaseState() {}
  virtual ~BaseState() {}

  /** \brief Gets an enum representing the type of pipeline that this state
   * requires.
   */
  virtual PipelineType pipeline() const { return PipelineType::Idle; }
  /** \brief Return a string representation of the state
   */
  virtual std::string name() const { return ""; }
  /** \brief Set the containing StateMachine
   */
  inline void setContainer(StateMachine* container) { container_ = container; }
  /** \brief Get the next intermediate state, for when no direct transition is
   * possible.
   */
  virtual Ptr nextStep(const BaseState* newState) const;
  /** \brief State through which we must always enter this meta-state
   */
  virtual Ptr entryState(const BaseState*) const;
  /** \brief Check the navigation state and perform necessary state transitions.
   * Global interrupts go here.
   */
  virtual void processGoals(Tactic*, UpgradableLockGuard& goal_lock,
                            const Event& event = Event());
  /** \brief Called as a cleanup method when the state exits.  The base state
   * never exits.
   */
  virtual void onExit(Tactic*, BaseState*) {}

  /** \brief Called as a setup method when the state is entered.  The base state
   * is never entered explicitly.
   */
  virtual void onEntry(Tactic*, BaseState*) {}

 protected:
  /** \brief The container class that holds this wobbly tack of pointers
   * together
   */
  StateMachine* container_;
#if 0
  /** \brief Perform run addition checks and modify the graph
   */
  void addRunInternal_(bool ephemeral = false, bool extend = false,
                       bool save = true);
#endif
};

/** \brief Convenience name output to stream
 */
std::ostream& operator<<(std::ostream& os, const BaseState& s);

/** \brief Container class for a stack of states.  Implements functions for
 * event processing loops.
 */
class StateMachine {
 public:
  /** \brief A set of callbacks that do nothing, so that we don't segfault when
   * no callbacks are set
   */
  class NullCallbacks : public StateMachineCallbacks {
   public:
    PTR_TYPEDEFS(NullCallbacks)

    virtual void stateChanged(const __shared_ptr<state::BaseState>&) {}
    virtual void stateSuccess() {}
    virtual void stateAbort(const std::string&) {}
    virtual void stateUpdate(double) {}
  };

  PTR_TYPEDEFS(StateMachine)
  using Tactic = StateMachineInterface;
  using GoalStack = std::list<BaseState::Ptr>;

  using LockGuard = std::unique_lock<std::recursive_mutex>;
  using UpgradableLockGuard = boost::upgrade_lock<boost::shared_mutex>;
  using UpgradedLockGuard = boost::upgrade_to_unique_lock<boost::shared_mutex>;
  using SharedLockGuard = boost::shared_lock<boost::shared_mutex>;
#if 0
  typedef boost::unique_lock<boost::shared_mutex> UniqueLockGuard;
#endif

  // Yes.  I know.  But the alternative is having every state in the queue have
  // a copy of a pointer to every other state in the queue, and that was really
  // messy.
  friend class BaseState;

  StateMachine(StateMachineCallbacks* callbacks = nullptr)
      : nullCallbacks_(new NullCallbacks()),
        callbacks_(callbacks == nullptr ? nullCallbacks_.get() : callbacks),
        tactic_(nullptr),
        runNeededOnRepeat_(true),
        triggerSuccess_(false) {}

  StateMachine(Tactic* tactic, StateMachineCallbacks* callbacks = nullptr)
      : nullCallbacks_(new NullCallbacks()),
        callbacks_(callbacks == nullptr ? nullCallbacks_.get() : callbacks),
        tactic_(tactic),
        runNeededOnRepeat_(true),
        triggerSuccess_(false) {}
#if 0
  /** \brief Set the current planner
   *
   * Note: this is not thread-safe, only call this if no threads are trying to
   * access it
   */
  void setPlanner(const PlanningInterface::Ptr& planner) {
    // The planner is used while processing events, lock it here for safety
    LockGuard event_lock(events_mutex_, std::defer_lock);
    planner_ = planner;
  }
#endif

  /** \brief Performs state transitions in a loop until a stable state is
   * reached
   */
  void handleEvents(const Event& event = Event(), bool blocking = false);
  /** \brief Build a new state machine with the initial state
   */
  static Ptr InitialState(StateMachineCallbacks* callbacks = nullptr);
  /** \brief Build a new state machine with the initial state
   */
  static Ptr InitialState(Tactic* tactic,
                          StateMachineCallbacks* callbacks = nullptr);

  /** \brief Return the current state pointer
   */
  inline const BaseState::Ptr& state() {
    SharedLockGuard lck(goal_mutex_);
    return goals_.front();
  }

  /** \brief Get the goal stack.  We need this because of weirdness in how C++
   * handles protected scope.
   */
  inline const GoalStack& goals() { return goals_; }

  /** \brief Gets an enum representing the type of pipeline that the current
   * state requires
   */
  inline PipelineType pipeline() {
    SharedLockGuard lck(goal_mutex_);
    return goals_.front()->pipeline();
  }

  /** \brief Return a string representation of the current state
   */
  inline std::string name() {
    SharedLockGuard lck(goal_mutex_);
    return goals_.front()->name();
  }

  /** \brief Direct callbacks to a specific object.
   *
   * Note: this is not thread-safe, only call if no threads are accessing it
   */
  inline void setCallbacks(StateMachineCallbacks* callbacks) {
    callbacks_ = callbacks;
  }
#if 0
  /** \brief Clear callbacks
   * Note: this is not thread-safe, only call if no threads are accessing it
   */
  inline void clearCallbacks() { callbacks_ = nullCallbacks_.get(); }
#endif
  /** \brief Set the tactic being managed by this state machine
   * Note: this is not thread-safe, and should be done once on startup.
   */
  inline void setTactic(Tactic* tactic) {
    tactic_ = tactic;
    tactic_->setPipeline(this->pipeline());
  }
  inline void triggerSuccess() { triggerSuccess_ = true; }
#if 0
  /** \brief Get the tactic being managed by this state machine
   */
  inline Tactic* tactic() const { return tactic_; }

  /** \brief Get a shared pointer to the current path planner
   */
  inline const PlanningInterface::Ptr& planner() const { return planner_; }

  /** \brief Get a shared pointer to the current path planner
   */
  inline StateMachineCallbacks* callbacks() const { return callbacks_; }
#endif

 protected:
  /** \brief Performs a single step of transition iteration
   */
  void step(BaseState::Ptr& oldState, UpgradableLockGuard& goal_lock);
  /** \brief A stack of intermediate goals, needed to reach a given user goal
   */
  GoalStack goals_;
  /** \brief Ugly way to get around managing the lifecycle of a NullCallbacks
   * object
   */
  __shared_ptr<StateMachineCallbacks> nullCallbacks_;
  /** \brief Hooks back into mission planning when things automatically succeed
   * or fail.  StateMachine does NOT own this object.
   */
  StateMachineCallbacks* callbacks_;

  /** \brief Pointer to the active tactic.  The StateMachine does NOT own this
   * object.
   */
  Tactic* tactic_;

#if 0
  /** \brief Pointer to the path planner
   */
  PlanningInterface::Ptr planner_;
#endif

  /** \brief Flag to indicate whether a repeat should trigger a new run
   *
   * This flag is set to true whenever ::Teach is entered.  It is also true on
   * construction as the first repeat in any period of operation must generate a
   * new run.
   */
  bool runNeededOnRepeat_;

  /** \brief Flag to indicate when we trigger a success
   */
  bool triggerSuccess_;

  /** \brief Keeps state modifications thread safe
   */
  boost::shared_mutex goal_mutex_;
  std::recursive_mutex events_mutex_;
};

}  // namespace state
}  // namespace planning
}  // namespace vtr

// We chain includes here for convenience
#include <vtr/planning/states/idle.h>
#if 0
#include <asrl/planning/states/Repeat.hpp>
#include <asrl/planning/states/Teach.hpp>
// UAV
#include <asrl/planning/states/Hover.hpp>
#include <asrl/planning/states/Learn.hpp>
#include <asrl/planning/states/Loiter.hpp>
#include <asrl/planning/states/Return.hpp>
#endif