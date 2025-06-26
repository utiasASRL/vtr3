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
 * \file test_utils.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning/mission_server/mission_server.hpp"
#include "vtr_mission_planning/state_machine/state_machine.hpp"

using namespace std::chrono_literals;
using namespace vtr;
using namespace vtr::mission_planning;

namespace vtr {
namespace mission_planning {

struct TestTactic : public StateMachine::Tactic {
  PTR_TYPEDEFS(TestTactic);

  PipelineLock lockPipeline() override {
    LOG(WARNING) << "Locking pipeline";
    PipelineLock lock(mutex_);
    return lock;
  }

  void setPipeline(const tactic::PipelineMode& pipeline) override {
    LOG(WARNING) << "Switching pipeline to " << pipeline;
  }

  void setPath(
      const tactic::PathType& path, const unsigned& trunk_sid = 0,
      const tactic::EdgeTransform& T_twig_branch = tactic::EdgeTransform(true),
      bool publish = false) override {
    LOG(WARNING) << "Setting path to " << path << ", trunk sid " << trunk_sid
                 << ", T_twig_branch " << T_twig_branch << ", with publish "
                 << publish;
  }

  /// Called when starting a new teach/repeat
  void addRun(bool) override { LOG(WARNING) << "Adding a new run"; }
  /// Called when finishing a teach/repeat
  void finishRun() override { LOG(WARNING) << "Finishing the current run"; }
  void setTrunk(const tactic::VertexId&, const tactic::EdgeTransform&) override {}
  /// Called when trying to merge into existing path
  void connectToTrunk(const bool privileged) override {
    LOG(WARNING) << "Connecting to trunk with privileged " << privileged;
  }

  tactic::Localization getPersistentLoc() const override { return loc_; }
  bool isLocalized() const override { return true; }
  bool passedSeqId(const uint64_t&) const override { return true; }
  bool routeCompleted() const override { return true; }

  tactic::PipelineMode pipeline_;
  tactic::Localization loc_;
  PipelineMutex mutex_;
};

struct TestRoutePlanner : public StateMachine::RoutePlanner {
  PTR_TYPEDEFS(TestRoutePlanner);
  // clang-format off
  PathType path(const VertexId&, const VertexId&) override { return PathType{}; }
  PathType path(const VertexId&, const VertexId::List&, std::list<uint64_t>&) override { return PathType(); }
  // clang-format on
};

struct TestPathPlanner : public StateMachine::PathPlanner {
  PTR_TYPEDEFS(TestPathPlanner);
  // clang-format off
  void setRunning(const bool) override {};
  // clang-format on
};

struct TestCallback : public StateMachineCallback {
  PTR_TYPEDEFS(TestCallback);
  void stateSuccess() override {
    LOG(WARNING) << "State success has been notified!";
  }
};

struct TestStateMachine : public StateMachineInterface {
 public:
  PTR_TYPEDEFS(TestStateMachine);

  TestStateMachine(const StateMachineCallback::Ptr& callback)
      : StateMachineInterface(callback) {}

  void handle(const Event::Ptr& event = std::make_shared<Event>(),
              const bool block = false) override {
    CLOG(WARNING, "mission.state_machine")
        << "Handling event: " << *event << ", block: " << block;
  }

  StateMachineCallback::Ptr callback() const {
    return StateMachineInterface::callback();
  }
};

struct TestGoalHandle {
  TestGoalHandle(const int& id0 = 0,
                 const GoalTarget& target0 = GoalTarget::Unknown,
                 const std::chrono::milliseconds& pause_before0 = 0ms,
                 const std::chrono::milliseconds& pause_after0 = 0ms,
                 const std::list<tactic::VertexId>& path0 = {})
      : id(id0),
        target(target0),
        pause_before(pause_before0),
        pause_after(pause_after0),
        path(path0) {}

  int id;
  GoalTarget target;
  std::chrono::milliseconds pause_before;
  std::chrono::milliseconds pause_after;
  std::list<tactic::VertexId> path;
};

}  // namespace mission_planning
}  // namespace vtr