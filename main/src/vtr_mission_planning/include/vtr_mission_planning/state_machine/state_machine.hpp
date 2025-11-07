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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <condition_variable>
#include <string>

#include "vtr_common/utils/macros.hpp"
#include "vtr_mission_planning/state_machine/event.hpp"
#include "vtr_path_planning/path_planner_interface.hpp"
#include "vtr_route_planning/route_planner_interface.hpp"
#include "vtr_tactic/tactic_interface.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace mission_planning {

class StateInterface;

class StateMachineCallback {
 public:
  PTR_TYPEDEFS(StateMachineCallback);
  virtual void stateSuccess() = 0;
  virtual void notifyRerouteStatus(const std::string& status) {}
};

class StateMachineInterface {
 public:
  PTR_TYPEDEFS(StateMachineInterface);

  StateMachineInterface(const StateMachineCallback::Ptr& callback);

  virtual ~StateMachineInterface() = default;

  /** \brief Performs state transitions until a stable state is reached */
  virtual void handle(const Event::Ptr& event = std::make_shared<Event>(),
                      const bool block = false) = 0;

 protected:
  /** \brief Gets a shared pointer to the current callbacks */
  StateMachineCallback::Ptr callback() const;

 private:
  /** \brief Hooks back into mission planning server */
  const StateMachineCallback::WeakPtr callback_;

  friend class StateInterface;
};

class StateMachine : public StateMachineInterface {
 public:
  PTR_TYPEDEFS(StateMachine);

  using Tactic = tactic::TacticInterface;
  using RoutePlanner = route_planning::RoutePlannerInterface;
  using PathPlanner = path_planning::PathPlannerInterface;

  using GoalStack = std::list<std::shared_ptr<StateInterface>>;

  using Mutex = std::mutex;
  using LockGuard = std::lock_guard<Mutex>;
  using UniqueLock = std::unique_lock<Mutex>;

  StateMachine(const Tactic::Ptr& tactic,
               const RoutePlanner::Ptr& route_planner,
               const PathPlanner::Ptr& path_planner,
               const StateMachineCallback::Ptr& callback);

  ~StateMachine();

  void process();

  /** \brief Performs state transitions until a stable state is reached */
  void handle(const Event::Ptr& event = std::make_shared<Event>(),
              const bool block = false) override;

  /** \brief Wait until the state machine has done handling events */
  void wait() const;

  /** \brief Return a string representation of the current state */
  std::string name() const;

 private:
  /** \brief Get the goal stack. */
  GoalStack& goals() { return goals_; }
  /** \brief Gets the tactic being managed by this state machine */
  Tactic::Ptr tactic() const;
  /** \brief Gets a shared pointer to the current route planner */
  RoutePlanner::Ptr route_planner() const;
  /** \brief Gets a shared pointer to the current path planner */
  PathPlanner::Ptr path_planner() const;
  /** \brief */
  void triggerSuccess() { trigger_success_ = true; }

  /** \brief Pointer to the active tactic. */
  const Tactic::WeakPtr tactic_;
  /** \brief Pointer to the route planner */
  const RoutePlanner::WeakPtr route_planner_;
  /** \brief Pointer to the path planner */
  const PathPlanner::WeakPtr path_planner_;

  /** \brief protects: event_, goals_, stop_, trigger_success_ */
  mutable Mutex mutex_;
  /** \brief wait until the event is processed */
  mutable std::condition_variable cv_empty_or_stop_;
  /** \brief wait until the queue is full or stop */
  mutable std::condition_variable cv_set_or_stop_;
  /** \brief wait until the process thread has finished */
  mutable std::condition_variable cv_thread_finish_;

  /** \brief Flag to indicate when we trigger a success */
  bool trigger_success_ = false;
  /** \brief A stack of intermediate goals, needed to reach a given user goal */
  GoalStack goals_;
  /** \brief Current event being processed */
  Event::Ptr event_ = nullptr;
  /** \brief signal the process thread to stop */
  bool stop_ = false;

  size_t thread_count_ = 0;
  /** \brief the event processing thread */
  std::thread process_thread_;

  friend class StateInterface;
};

}  // namespace mission_planning
}  // namespace vtr