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
 * \file state_machine.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_mission_planning_v2/state_machine/state_machine.hpp"

#include "vtr_mission_planning_v2/state_machine/base_state.hpp"  // Idle

namespace vtr {
namespace mission_planning {

StateMachineInterface::StateMachineInterface(
    const StateMachineCallback::Ptr& callback)
    : callback_(callback) {}

StateMachineCallback::Ptr StateMachineInterface::callback() const {
  if (auto callback_acquired = callback_.lock())
    return callback_acquired;
  else {
    std::string err{"Callback has expired"};
    CLOG(WARNING, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

StateMachine::StateMachine(const Tactic::Ptr& tactic,
                           const RoutePlanner::Ptr& planner,
                           const StateMachineCallback::Ptr& callback)
    : StateMachineInterface(callback), tactic_(tactic), planner_(planner) {
  // initialize to idle state
  goals_.push_front(std::make_shared<Idle>());
  //
  thread_count_ = 1;
  process_thread_ = std::thread(&StateMachine::process, this);
}

StateMachine::~StateMachine() {
  UniqueLock lock(mutex_);
  // wait until no more event is being handled
  cv_empty_or_stop_.wait(lock, [this] { return stop_ || (event_ == nullptr); });
  // send stop signal
  stop_ = true;
  cv_set_or_stop_.notify_all();
  //
  cv_thread_finish_.wait(lock, [this] { return thread_count_ == 0; });
  if (process_thread_.joinable()) process_thread_.join();
}

void StateMachine::process() {
  el::Helpers::setThreadName("mission.state_machine");
  CLOG(INFO, "mission.state_machine") << "Starting the state machine thread.";
  while (true) {
    UniqueLock lock(mutex_);

    cv_set_or_stop_.wait(lock, [this] { return stop_ || (event_ != nullptr); });

    if (stop_) {
      --thread_count_;
      CLOG(INFO, "mission.state_machine")
          << "Stopping the state machine thread.";
      cv_thread_finish_.notify_all();
      return;
    }

    CLOG(DEBUG, "mission.state_machine") << "Processing event " << *event_;

    auto curr_state = goals_.front();
    // acquire the tactic, route planner and callback so that they are not
    // expired during handling the event
    const auto tactic_acquired = tactic();
    const auto planner_acquired = planner();
    const auto callback_acquired = callback();

    curr_state->processGoals(*this, *event_);

    // no transition, keep current state
    if (curr_state == goals_.front()) continue;

    // perform all state transitions until we get to a state that is stable
    CLOG(INFO, "mission.state_machine") << "Lock the tactic pipeline.";
    auto lck = tactic()->lockPipeline();
    while (curr_state != goals_.front()) {
      // The target state is always at the top of the stack
      auto new_state = goals_.front();

      CLOG(INFO, "mission.state_machine")
          << "Transitioning from " << curr_state->name() << " to "
          << new_state->name();

      // Invoke exit/entry logic for the old/new state
      curr_state->onExit(*this, *new_state);
      tactic()->setPipeline(new_state->pipeline());
      new_state->onEntry(*this, *curr_state);
      curr_state = new_state;

      CLOG(INFO, "mission.state_machine") << "In state " << curr_state->name();

      // Perform one processing step to see if the state will remain stable
      curr_state->processGoals(*this, Event());
    }

    if (trigger_success_) {
      trigger_success_ = false;
      callback()->stateSuccess();
    }

    event_ = nullptr;
    cv_empty_or_stop_.notify_one();
    CLOG(INFO, "mission.state_machine") << "Unlock the tactic pipeline.";
  }
}

void StateMachine::handle(const Event::Ptr& event, const bool block) {
  UniqueLock lock(mutex_, std::defer_lock);
  if (block) {
    lock.lock();
    cv_empty_or_stop_.wait(lock,
                           [this] { return stop_ || (event_ == nullptr); });
  } else {
    lock.try_lock();

    if (!lock.owns_lock()) {
      CLOG(WARNING, "mission.state_machine")
          << "Skipping event " << event << " due to lock conflict.";
      return;
    }

    if (event_ != nullptr) {
      CLOG(WARNING, "mission.state_machine")
          << "Skipping event " << event << " because there is already one.";
      return;
    }
  }

  if (stop_) {
    CLOG(WARNING, "mission.state_machine")
        << "Dropping event " << event
        << " because the state machine is stopped.";
    return;
  }

  event_ = event;

  CLOG(DEBUG, "mission.state_machine") << "Handling event " << *event_;
  cv_set_or_stop_.notify_one();
}

void StateMachine::wait() const {
  UniqueLock lock(mutex_);
  cv_empty_or_stop_.wait(lock, [this] { return stop_ || (event_ == nullptr); });
}

std::string StateMachine::name() const {
  LockGuard lock(mutex_);
  return goals_.front()->name();
}

auto StateMachine::tactic() const -> Tactic::Ptr {
  if (auto tactic_acquired = tactic_.lock())
    return tactic_acquired;
  else {
    std::string err{"Tactic has expired"};
    CLOG(WARNING, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

auto StateMachine::planner() const -> RoutePlanner::Ptr {
  if (auto planner_acquired = planner_.lock())
    return planner_acquired;
  else {
    std::string err{"Planner has expired"};
    CLOG(WARNING, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

}  // namespace mission_planning
}  // namespace vtr
