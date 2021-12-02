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

class RoutePlannerInterface {
 public:
  using PathType = tactic::PathType;
  using VertexId = tactic::VertexId;
  PTR_TYPEDEFS(RoutePlannerInterface);

  virtual ~RoutePlannerInterface() = default;

  virtual PathType path(const VertexId& from, const VertexId& to) = 0;
  virtual PathType path(const VertexId& from, const VertexId::List& to,
                        std::list<uint64_t>* idx) = 0;
};

class TacticInterface {
 public:
  PTR_TYPEDEFS(TacticInterface);

  using Mutex = std::recursive_timed_mutex;
  using UniqueLock = std::unique_lock<Mutex>;

  virtual ~TacticInterface() = default;

  /**
   * \brief Clears the pipeline and stops callbacks.
   * \returns a lock that blocks the pipeline
   */
  virtual UniqueLock lockPipeline() = 0;
  /** \brief Set the pipeline used by the tactic */
  virtual void setPipeline(const tactic::PipelineMode& pipeline) = 0;
  /** \brief Set the path being followed */
  virtual void setPath(const tactic::PathType& path, bool follow = false) = 0;
  /** \brief Set the current privileged vertex (topological localization) */
  virtual void setTrunk(const tactic::VertexId& v) = 0;
  /** \brief Get distance between the current loc. chain to the target vertex */
  virtual double distanceToSeqId(const uint64_t& idx) = 0;
  /** \brief Add a new run to the graph and reset localization flags */
  virtual void addRun(bool ephemeral = false) = 0;
  virtual bool pathFollowingDone() = 0;
  /** \brief Whether or not can merge into existing graph. */
  virtual bool canCloseLoop() const = 0;
  /** \brief Add a new vertex, link it to the current trunk and branch */
  virtual void connectToTrunk(bool privileged = false, bool merge = false) = 0;
  /** \brief Trigger a graph relaxation */
  virtual void relaxGraph() = 0;
  /** \brief Save the graph */
  virtual void saveGraph() = 0;
  virtual const tactic::Localization& persistentLoc() const = 0;
};

class StateMachineCallback {
 public:
  PTR_TYPEDEFS(StateMachineCallback);
  virtual void stateAbort(const std::string&) = 0;
  virtual void stateSuccess() = 0;
  virtual void stateUpdate(double) = 0;
};

class StateMachine {
 public:
  PTR_TYPEDEFS(StateMachine);

  using Tactic = TacticInterface;
  using RoutePlanner = RoutePlannerInterface;

  using GoalStack = std::list<std::shared_ptr<StateInterface>>;

  using Mutex = std::mutex;
  using LockGuard = std::lock_guard<Mutex>;
  using UniqueLock = std::unique_lock<Mutex>;

  StateMachine(const Tactic::Ptr& tactic, const RoutePlanner::Ptr& planner,
               const StateMachineCallback::Ptr& callback);

  ~StateMachine();

  void process();

  /** \brief Performs state transitions until a stable state is reached */
  void handle(const Event::Ptr& event = std::make_shared<Event>(),
              const bool block = false);

  /** \brief Wait until the state machine has done handling events */
  void wait() const;

  /** \brief Return a string representation of the current state */
  std::string name() const;

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

  /** \brief the event processing thread */
  std::thread process_thread_;

  friend class StateInterface;
};

}  // namespace mission_planning
}  // namespace vtr