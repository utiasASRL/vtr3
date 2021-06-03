#include <vtr_mission_planning/state_machine.hpp>

namespace vtr {
namespace mission_planning {
namespace state {

void StateMachine::handleEvents(const Event& event, bool blocking) {
  LockGuard event_lock(events_mutex_, std::defer_lock);

  // Only lock in a blocking way if the blocking flag is set
  if (blocking) {
    event_lock.lock();
  } else {
    event_lock.try_lock();
    if (!event_lock.owns_lock()) {
      LOG(DEBUG) << "Skipping event processing due to lock conflict";
      return;
    }
  }
  UpgradableLockGuard goal_lock(goal_mutex_);
  BaseState::Ptr state = goals_.front();
  state->processGoals(tactic_, goal_lock, event);

  if (state != goals_.front()) {
    LOG(DEBUG) << "[Lock Requested] handleEvents";
    auto lck = tactic_->lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] handleEvents";
    // Perform all state transitions until we get to a state that is stable
    while (state != goals_.front()) step(state, goal_lock);
    if (triggerSuccess_) {
      triggerSuccess_ = false;
      callbacks_->stateSuccess();
    }
    LOG(DEBUG) << "[Lock Released] handleEvents";
  }
}

void StateMachine::step(BaseState::Ptr& oldState,
                        UpgradableLockGuard& goal_lock) {
  // The target state is always at the top of the stack
  BaseState::Ptr newState = goals_.front();

  LOG(INFO) << "Transitioning from " << oldState->name() << " to "
            << newState->name();

  // Invoke exit/entry logic for the old/new state
  oldState->onExit(tactic_, newState.get());
  tactic_->setPipeline(newState->pipeline());
  newState->onEntry(tactic_, oldState.get());
  oldState = newState;

  LOG(INFO) << "In state " << newState->name();

  // Perform one processing step to see if the state will remain stable
  newState->processGoals(tactic_, goal_lock);
}

auto StateMachine::InitialState(StateMachineCallbacks* callbacks) -> Ptr {
  Ptr rval(new StateMachine(callbacks));

  BaseState::Ptr state(new Idle());
  state->setContainer(rval.get());

  rval->goals_ = GoalStack({{state}});
  return rval;
}

auto StateMachine::InitialState(Tactic* tactic,
                                StateMachineCallbacks* callbacks) -> Ptr {
  Ptr rval(new StateMachine(tactic, callbacks));

  BaseState::Ptr state(new Idle());
  state->setContainer(rval.get());

  rval->goals_ = GoalStack({{state}});
  return rval;
}

void BaseState::addRunInternal_(bool ephemeral, bool extend, bool save) {
  (void)ephemeral;
  //  if (teaching || container_->runNeededOnRepeat_) {
  //    container_->tactic_->addRun();
  //  }
  //  container_->runNeededOnRepeat_ = teaching;

  // \todo Make sure we can do multiple repeats in a single run!
  container_->tactic_->addRun(false, extend, save);
}

void BaseState::processGoals(Tactic*,
                             StateMachine::UpgradableLockGuard& goal_lock,
                             const Event& event) {
  // The reset flag stops us from triggering a success callback on an Abort or
  // Reset
  bool reset = false;
  {
    // Make sure no one else is accessing goals
    StateMachine::UpgradedLockGuard goal_unique_lock(goal_lock);

    switch (event.type_) {
      case Action::Continue:
        if (event.signal_ != Signal::Continue) {
          std::stringstream ss;
          ss << "[Base] Received unhandled signal: " << event.signal_;
          throw std::runtime_error(ss.str());
        }
        break;
      case Action::Abort:
        // Abort: Something bad happened; stop whatever we were doing
        container_->callbacks_->stateAbort("Internal error");
        [[fallthrough]];
      case Action::Reset:
        // Goal canceled so we reset the statemachine
        container_->goals_ = std::list<Ptr>();
        reset = true;
        break;
      case Action::EndGoal:
        // EndGoal: This goal ended normally, so remove it from the stack and
        // move on
        container_->goals_.pop_front();
        break;
      case Action::SwapGoal:
        // SwapGoal: This goal ended normally, but needs to be replaced with
        // another goal
        container_->goals_.pop_front();
        container_->goals_.push_front(event.goal_);
        container_->goals_.front()->setContainer(container_);
        break;
      case Action::NewGoal:
        // NewGoal: We are changing tracks completely, so reset the goal stack
        container_->goals_ = std::list<Ptr>();
        [[fallthrough]];  // NOTE: we do not break here intentionally
      case Action::AppendGoal:
        // A new goal must be added to the stack and transitioned to
        container_->goals_.push_front(event.goal_);
        container_->goals_.front()->setContainer(container_);
        break;
    }

    // If we ever finish our list of goals, drop into Idle automatically
    if (container_->goals_.empty()) {
      container_->goals_.push_front(Ptr(new Idle()));
      container_->goals_.front()->setContainer(container_);
    }

    // Check if there is another goal we must transition through first to get
    // to the target goal. We don't need to check if the target was changed,
    // as goal->nextStep(goal) always returns nullptr.
    Ptr intermediateState = nextStep(container_->goals_.front().get());
    if (intermediateState) container_->goals_.push_front(intermediateState);
  }  // We are now done modifying the goals

  // Raise appropriate callbacks for state changes/successful goal completion
  if (container_->goals_.front().get() != this) {
    if (container_->goals_.size() == 1 &&
        Idle::InChain(container_->goals_.front().get()) && !reset) {
      container_->triggerSuccess();
    }
    container_->callbacks_->stateChanged(container_->goals_.front());
  }
}

auto BaseState::nextStep(const BaseState* newState) const -> Ptr {
  if (typeid(BaseState) == typeid(*newState)) {
    auto s = "Transitioning to the base state is not allowed.";
    LOG(ERROR) << s;
    throw std::runtime_error(s);
  }

  if (Idle::IsType(newState)) {
    // We can always go straight to Idle
    return nullptr;
  } else if (Teach::InChain(newState) || Repeat::InChain(newState)) {
    // We must use the entry state of the correct child for composite states
    return newState->entryState(this);
  } else {
    // One of two things screwed up if we got here:
    //    - We attempted to transition to some weird, unknown state
    //    - A transition that should have been handled lower in the tree
    //    wasn't
    std::stringstream ss;
    ss << "Transitioning to state " << newState->name()
       << " should be handled before the base state.";
    throw std::runtime_error(ss.str());
  }
}

auto BaseState::entryState(const BaseState*) const -> Ptr {
  Ptr rptr(new Idle(*this));
  rptr->setContainer(this->container_);
  return rptr;
}

std::ostream& operator<<(std::ostream& os, const BaseState& s) {
  os << s.name();
  return os;
}

}  // namespace state
}  // namespace mission_planning
}  // namespace vtr
