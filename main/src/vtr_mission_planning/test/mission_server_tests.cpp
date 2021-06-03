#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_mission_planning/base_mission_server.hpp>
#include <vtr_mission_planning/test_utils.hpp>

using namespace vtr::mission_planning;

class TestMissionServer : public BaseMissionServer<TestGoalHandle> {
 public:
  PTR_TYPEDEFS(TestMissionServer)

  using Base = BaseMissionServer<TestGoalHandle>;

  TestMissionServer(const StateMachine::Ptr& state)
      : Base(state), state_idx(0), aborted(false), succeeded(false) {
  }

  /** \brief Callback when the state machine must abort a goal. */
  void stateAbort(const std::string& msg) override {
    LOG(INFO) << "[TEST] State abort";
    aborted = true;
    Base::stateAbort(msg);
  }

  /** \brief Callback when the state machine changes state. */
  void stateChanged(const state::BaseState::Ptr& state) override {
    LOG(INFO) << "[TEST] State changed: " << *state;
    state_idx++;
  }

  /** \brief Callback when the state machine is finished executing a goal. */
  void stateSuccess() override {
    LOG(INFO) << "[TEST] State success";
    succeeded = true;
    Base::stateSuccess();
  }

  /** \brief Callback when the state machine changes state. */
  void stateUpdate(double) override {
  }

  // void clear() {
  //   state_idx = 0;
  //   aborted = false;
  //   succeeded = false;
  // }

  int state_idx;
  bool aborted;
  bool succeeded;
};

TEST(MissionPlanning, forcedOperation) {
  StateMachine::Ptr sm{StateMachine::InitialState()};
  TestTactic::Ptr tactic{new TestTactic()};
  sm->setTactic(tactic.get());
  sm->setPlanner(TestPathPlanner::Ptr{new TestPathPlanner{}});

  TestMissionServer::Ptr server(new TestMissionServer(sm));

  // Add first goal
  TestGoalHandle gh;
  gh.id = 0;
  gh.target = Target::Teach;
  server->addGoal(gh);

  // The server starts paused, so it should not have moved at all.
  EXPECT_EQ(sm->name(), "::Idle");
  EXPECT_EQ(server->state_idx, 0);
  EXPECT_FALSE(server->aborted);
  EXPECT_FALSE(server->succeeded);

  server->setPause(false, false);
  std::this_thread::sleep_for(
      std::chrono::milliseconds{200});  // wait for async calls
  // \todo Check: seems currently we always go directly from teach:topo loc to
  // branch, ignoring VertexFound signal
#if 0
  // Unpausing the server should move us to the first step along the chain to
  // Mapping
  EXPECT_EQ(sm->name(), "::Teach::TopologicalLocalize");
  EXPECT_EQ(server->state_idx, 1);
  EXPECT_FALSE(server->aborted);
  EXPECT_FALSE(server->succeeded);

  // Make sure we can retrieve goals
  EXPECT_NO_THROW(server->goal(0));

  // Check that our goal didn't get misplaced somehow
  EXPECT_EQ(server->top().id, 0);
  EXPECT_TRUE(server->isTracking(0));

  sm->handleEvents(Event(Signal::VertexFound));
#endif
  EXPECT_EQ(sm->name(), "::Teach::Branch");
  EXPECT_EQ(server->state_idx, 2);
  EXPECT_FALSE(server->aborted);
  EXPECT_FALSE(server->succeeded);

  // Check that our goal didn't get misplaced somehow
  EXPECT_EQ(server->top().id, 0);
  EXPECT_TRUE(server->isTracking(0));

  // Add second goal
  TestGoalHandle gh1;
  gh1.id = 1;
  gh1.target = Target::Repeat;
  server->addGoal(gh1, int{1});  // add after the first goal

  // Add third goal
  TestGoalHandle gh2;
  gh2.id = 2;
  gh2.target = Target::Teach;
  server->addGoal(gh2, TestGoalHandle::Id{1});  // add before the second goal

  server->cancelGoal(0);  // cancel the first goal
  std::this_thread::sleep_for(
      std::chrono::milliseconds{200});  // wait for async calls
  EXPECT_EQ(sm->name(), "::Teach::Branch");
  EXPECT_EQ(server->state_idx, 5);

  server->setPause();
  server->cancelGoal(2);  // cancel the third goal (active)
  std::this_thread::sleep_for(
      std::chrono::milliseconds{200});  // wait for async calls
  EXPECT_EQ(sm->name(), "::Idle");
  EXPECT_EQ(server->state_idx, 6);
  EXPECT_EQ(server->status(), ServerState::Paused);

  server->setPause(false, false);  // start the second goal
  std::this_thread::sleep_for(
      std::chrono::milliseconds{200});  // wait for async calls
  // cannot repeat due to no persistent localization set
  EXPECT_EQ(sm->name(), "::Idle");
  EXPECT_TRUE(server->aborted);
  EXPECT_EQ(server->state_idx, 8);
  EXPECT_EQ(server->status(), ServerState::Paused);

  // Check that our goal is gone
  EXPECT_FALSE(server->isTracking(0));
  EXPECT_FALSE(server->isTracking(1));
  EXPECT_FALSE(server->isTracking(2));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}