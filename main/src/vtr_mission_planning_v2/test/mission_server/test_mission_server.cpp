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
 * \brief
 *
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

TEST_F(MissionServerTest, constructor_destructor) {
  LOG(INFO) << "Destruct MissionServer without calling start";
}

TEST_F(MissionServerTest, constructor_start_destructor) {
  mission_server->start(state_machine);
  LOG(INFO) << "Destruct MissionServer after calling start";
}

#if false
TEST_F(MissionServerTest, end_goal) {
  StateMachine sm(tactic, planner, callback);
  // try ending the current state, (idle in this case)
  sm.handle(std::make_shared<Event>(Action::EndGoal));
}

TEST_F(MissionServerTest, direct_to_idle) {
  StateMachine sm(tactic, planner, callback);
  // try going to idle state (nothing happens because we start in idle state)
  sm.handle(std::make_shared<Event>(Action::NewGoal, StateGenerator::Idle()));
}
#endif

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}