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
 * \file state_machine.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_common/utils/macros.hpp"
#include "vtr_mission_planning_v2/state_machine/event.hpp"
#include "vtr_path_planning/planning_interface.hpp"
#include "vtr_tactic/state_machine_interface.hpp"

namespace vtr {
namespace mission_planning {

class StateInterface;

class StateMachineCallback {
 public:
  PTR_TYPEDEFS(StateMachineCallback);
  virtual void stateAbort(const std::string&) {}
  virtual void stateSuccess() {}
  virtual void stateUpdate(double) {}
};

class StateMachine {
 public:
  PTR_TYPEDEFS(StateMachine)

  using Tactic = tactic::StateMachineInterface;
  using PipelineMode = tactic::PipelineMode;

  using RoutePlanner = path_planning::PlanningInterface;

  using GoalStack = std::list<std::shared_ptr<StateInterface>>;

  using Mutex = std::mutex;
  using LockGuard = std::lock_guard<Mutex>;
  using UniqueLock = std::unique_lock<Mutex>;

  StateMachine(const tactic::StateMachineInterface::Ptr& tactic,
               const path_planning::PlanningInterface::Ptr& planner,
               const StateMachineCallback::Ptr& callback)
      : tactic_(tactic), planner_(planner), callback_(callback) {
    process_thread_ = std::thread(&StateMachine::process, this);
  }

  void process();

  /** \brief Performs state transitions until a stable state is reached */
  void handle(const Event::Ptr& event = std::make_shared<Event>(),
              const bool block = false);
#if false
  /** \brief Gets the type of pipeline that the current state requires */
  PipelineMode pipeline() {
    LockGuard lock(mutex_);
    return goals_.front()->pipeline();
  }

  /** \brief Return a string representation of the current state */
  std::string name() {
    LockGuard lock(mutex_);
    return goals_.front()->name();
  }
#endif
 private:
  /** \brief Get the goal stack. */
  GoalStack& goals() { return goals_; }
  /** \brief Gets the tactic being managed by this state machine */
  Tactic::Ptr tactic() const;
  /** \brief Gets a shared pointer to the current path planner */
  RoutePlanner::Ptr planner() const;
  /** \brief Gets a shared pointer to the current callbacks */
  StateMachineCallback::Ptr callback() const;
  /** \brief */
  void triggerSuccess() { trigger_success_ = true; }

  /** \brief Pointer to the active tactic. */
  const Tactic::WeakPtr tactic_;
  /** \brief Pointer to the path planner */
  const RoutePlanner::WeakPtr planner_;
  /** \brief Hooks back into mission planning server */
  const StateMachineCallback::WeakPtr callback_;

  /** \brief Flag to indicate when we trigger a success */
  bool trigger_success_ = false;

  /** \brief A stack of intermediate goals, needed to reach a given user goal */
  GoalStack goals_;

  /** \brief Current event being processed */
  Event::Ptr event_ = nullptr;

  /** \brief signal the process thread to stop */
  bool stop_ = false;

  /** \brief protects: event_, goals_, stop_ */
  Mutex mutex_;
  /** \brief wait until the event is processed */
  std::condition_variable cv_empty_or_stop_;
  /** \brief wait until the queue is full or stop*/
  std::condition_variable cv_set_or_stop_;

  /** \brief the event processing thread */
  std::thread process_thread_;

  friend class StateInterface;
};

}  // namespace mission_planning
}  // namespace vtr