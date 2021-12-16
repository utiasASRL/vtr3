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
 * \file test_misson_server.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_mission_planning_v2/mission_server/mission_server.hpp"
#include "vtr_mission_planning_v2/test_utils.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::mission_planning;

class MissionServerTest : public Test {
 protected:
  MissionServerTest() {
    mission_server = std::make_shared<MissionServer<TestGoalHandle>>();
    state_machine = std::make_shared<TestStateMachine>(mission_server);
  }
  MissionServer<TestGoalHandle>::Ptr mission_server = nullptr;
  TestStateMachine::Ptr state_machine = nullptr;
};

TEST_F(MissionServerTest, constructor_destructor) {}

TEST_F(MissionServerTest, constructor_start_destructor) {
  mission_server->start(state_machine);
}

TEST_F(MissionServerTest, teach_goal_complete_life_cycle) {
  // assign state machine
  mission_server->start(state_machine);
  // add first goal, should start immediately
  TestGoalHandle gh(0, GoalTarget::Teach, 1000ms, 1000ms);
  LOG(WARNING) << "Add the first goal";
  mission_server->addGoal(gh);
  std::this_thread::sleep_for(2000ms);
  // manually cancel the goal
  LOG(WARNING) << "Cancel the first goal";
  mission_server->cancelGoal(gh);
}

TEST_F(MissionServerTest, teach_goal_stop_during_wait_before) {
  // assign state machine
  mission_server->start(state_machine);
  // add first goal, should start immediately
  TestGoalHandle gh(0, GoalTarget::Teach, 1000ms, 1000ms);
  LOG(WARNING) << "Add the first goal";
  mission_server->addGoal(gh);
  std::this_thread::sleep_for(500ms);
  // manually cancel the goal
  LOG(WARNING) << "Cancel the first goal";
  mission_server->cancelGoal(gh);
}

TEST_F(MissionServerTest, repeat_goal_complete_life_cycle) {
  // assign state machine
  mission_server->start(state_machine);
  // add first goal, should start immediately
  TestGoalHandle gh(0, GoalTarget::Repeat, 1000ms, 1000ms);
  LOG(WARNING) << "Add the first goal";
  mission_server->addGoal(gh);
  std::this_thread::sleep_for(2000ms);
  // state success
  LOG(WARNING) << "Fake state success";
  state_machine->callback()->stateSuccess();
  std::this_thread::sleep_for(2000ms);
}

TEST_F(MissionServerTest, pause) {
  // assign state machine
  mission_server->start(state_machine);
  //
  LOG(WARNING) << "Pause mission server";
  mission_server->setPause(true);
  // add first goal, does not start
  TestGoalHandle gh(0, GoalTarget::Teach, 1000ms, 1000ms);
  LOG(WARNING) << "Add the first goal";
  mission_server->addGoal(gh);
  // add second goal
  LOG(WARNING) << "Add the second goal";
  TestGoalHandle gh2(1, GoalTarget::Teach, 1000ms, 1000ms);
  mission_server->addGoal(gh2);
  std::this_thread::sleep_for(1000ms);
  // manually cancel the goal
  LOG(WARNING) << "Cancel the first goal";
  mission_server->cancelGoal(gh);
  // resume mission server, second goal should start
  LOG(WARNING) << "Resume mission server";
  mission_server->setPause(false);
  std::this_thread::sleep_for(2000ms);
  LOG(WARNING) << "Cancel the second goal";
  mission_server->cancelGoal(gh2);
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}