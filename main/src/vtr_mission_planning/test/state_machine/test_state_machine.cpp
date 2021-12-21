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
 * \file test_state_machine.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_mission_planning/state_machine/base_state.hpp"
#include "vtr_mission_planning/state_machine/state_machine.hpp"
#include "vtr_mission_planning/test_utils.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::mission_planning;

struct StateGenerator {
  // clang-format off
  static StateInterface::Ptr Idle() { return std::make_shared<mission_planning::Idle>(); }
  // teach states
  static StateInterface::Ptr TeachBranch() { return std::make_shared<teach::Branch>(); }
  static StateInterface::Ptr TeachMerge() { return std::make_shared<teach::Merge>(); }
  static StateInterface::Ptr TeachTopoLoc() { return std::make_shared<teach::TopologicalLocalize>(); }
  // repeat states
  static StateInterface::Ptr RepeatFollow() { return std::make_shared<repeat::Follow>(); }
  static StateInterface::Ptr RepeatMetrloc() { return std::make_shared<repeat::MetricLocalize>(); }
  static StateInterface::Ptr RepeatTopoloc() { return std::make_shared<repeat::TopologicalLocalize>(); }
  static StateInterface::Ptr RepeatPlan() { return std::make_shared<repeat::Plan>(); }
  // clang-format on
};

class StateMachineTest : public Test {
 protected:
  // clang-format off
  StateMachine::Tactic::Ptr tactic = std::make_shared<TestTactic>();
  StateMachine::RoutePlanner::Ptr planner = std::make_shared<TestRoutePlanner>();
  StateMachineCallback::Ptr callback = std::make_shared<TestCallback>();
  // clang-format on
};

TEST_F(StateMachineTest, constructor_destructor) {
  StateMachine sm(tactic, planner, callback);
}

TEST_F(StateMachineTest, end_goal) {
  StateMachine sm(tactic, planner, callback);
  // try ending the current state, idle in this case, a new idle is appended
  // automatically
  sm.handle(std::make_shared<Event>(Action::EndGoal));
}

TEST_F(StateMachineTest, idle_to_idle) {
  StateMachine sm(tactic, planner, callback);
  // idle --> idle, pipeline locked, no side effect
  sm.handle(std::make_shared<Event>(Action::NewGoal, StateGenerator::Idle()));
}

TEST_F(StateMachineTest, idle_to_teach_branch) {
  StateMachine sm(tactic, planner, callback);
  // idle --> teach_branch, pass through topo_loc, metric_loc
  sm.handle(
      std::make_shared<Event>(Action::NewGoal, StateGenerator::TeachBranch()));
}

TEST_F(StateMachineTest, teach_branch_to_teach_merge) {
  StateMachine sm(tactic, planner, callback);
  // idle --> teach_branch, pass through topo_loc, metric_loc
  sm.handle(
      std::make_shared<Event>(Action::NewGoal, StateGenerator::TeachBranch()),
      true);

  // teach_branch --> teach_merge
  const auto merge = StateGenerator::TeachMerge();
  std::dynamic_pointer_cast<teach::Merge>(merge)->setTarget(
      std::vector<tactic::VertexId>{{1, 50}, {1, 300}});
  sm.handle(std::make_shared<Event>(Action::SwapGoal, merge), true);
}

TEST_F(StateMachineTest, teach_merge_to_teach_branch) {
  StateMachine sm(tactic, planner, callback);

  // idle --> teach_branch, pass through topo_loc, metric_loc
  sm.handle(
      std::make_shared<Event>(Action::NewGoal, StateGenerator::TeachBranch()),
      true);

  // teach_branch --> teach_merge
  const auto merge = StateGenerator::TeachMerge();
  std::dynamic_pointer_cast<teach::Merge>(merge)->setTarget(
      std::vector<tactic::VertexId>{{1, 50}, {1, 300}});
  sm.handle(std::make_shared<Event>(Action::SwapGoal, merge), true);

  // continue teach
  sm.handle(std::make_shared<Event>(Signal::ContinueTeach), true);
}

TEST_F(StateMachineTest, teach_merge_to_teach_merge) {
  StateMachine sm(tactic, planner, callback);

  // idle --> teach_branch, pass through topo_loc, metric_loc
  sm.handle(
      std::make_shared<Event>(Action::NewGoal, StateGenerator::TeachBranch()),
      true);

  // teach_branch --> teach_merge
  const auto merge = StateGenerator::TeachMerge();
  std::dynamic_pointer_cast<teach::Merge>(merge)->setTarget(
      std::vector<tactic::VertexId>{{1, 50}, {1, 300}});
  sm.handle(std::make_shared<Event>(Action::SwapGoal, merge), true);

  // teach_merge --> teach_merge with different window
  const auto merge2 = StateGenerator::TeachMerge();
  std::dynamic_pointer_cast<teach::Merge>(merge2)->setTarget(
      std::vector<tactic::VertexId>{{5, 20}, {3, 100}});
  sm.handle(std::make_shared<Event>(Action::SwapGoal, merge2), true);
}

TEST_F(StateMachineTest, teach_merge_to_idle) {
  StateMachine sm(tactic, planner, callback);

  // idle --> teach_branch, pass through topo_loc, metric_loc
  sm.handle(
      std::make_shared<Event>(Action::NewGoal, StateGenerator::TeachBranch()),
      true);

  // teach_branch --> teach_merge
  const auto merge = StateGenerator::TeachMerge();
  std::dynamic_pointer_cast<teach::Merge>(merge)->setTarget(
      std::vector<tactic::VertexId>{{1, 50}, {1, 300}});
  sm.handle(std::make_shared<Event>(Action::SwapGoal, merge), true);

  // attempt closure
  sm.handle(std::make_shared<Event>(Signal::AttemptClosure), true);
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}