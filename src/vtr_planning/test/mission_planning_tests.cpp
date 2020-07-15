#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vtr/planning/base_mission_server.h>

#include <asrl/common/logging.hpp>
// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING
// easylogging++.h **
INITIALIZE_EASYLOGGINGPP

#include <test_utils.h>

using namespace vtr::planning;

/** Test goal handle to ensure that the mission planner makes the correct calls.
 */
struct TestGoalHandle {
  std::string id_;
  Target target_;
  std::list<VertexId> path_;
  VertexId vertex_;
  std::chrono::milliseconds pauseBefore_{0};
  std::chrono::milliseconds pauseAfter_{0};
};

class TestMissionServer : public BaseMissionServer<TestGoalHandle> {
 public:
  PTR_TYPEDEFS(TestMissionServer)

  using Base = BaseMissionServer<TestGoalHandle>;

  TestMissionServer(const StateMachine::Ptr& state)
      : Base(state), state_idx(0), aborted(false), succeeded(false) {}

  /** \brief Callback when the state machine is finished executing a goal. */
  virtual void stateSuccess() {
    succeeded = true;
    Base::stateSuccess();
  }

  /** \brief Callback when the state machine must abort a goal. */
  virtual void stateAbort(const std::string& msg) {
    aborted = true;
    Base::stateAbort(msg);
  }

  /** \brief Callback when the state machine changes state. */
  virtual void stateChanged(const state::BaseState::Ptr&) { state_idx++; }

  /** \brief Callback when the state machine changes state. */
  virtual void stateUpdate(double) {}

  int state_idx;
  bool aborted;
  bool succeeded;
};

TEST(MissionPlanning, forcedOperation) {
  StateMachine::Ptr sm{StateMachine::InitialState()};

  TestTactic::Ptr tactic{new TestTactic()};
  sm->setTactic(tactic.get());
  sm->setPlanner(TestPathPlanner::Ptr(new TestPathPlanner()));

  TestMissionServer::Ptr server(new TestMissionServer(sm));

  TestGoalHandle gh;
  gh.id_ = "test_gh";
  gh.target_ = Target::Teach;

  server->addGoal(gh);

  // The server starts paused, so it should not have moved at all.
  EXPECT_EQ(sm->name(), "::Idle");
  EXPECT_EQ(server->state_idx, 0);
  EXPECT_FALSE(server->aborted);
  EXPECT_FALSE(server->succeeded);

  server->setPaused(false, false);
  // wait for async calls
  std::this_thread::sleep_for(std::chrono::milliseconds{200});

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
  EXPECT_NO_THROW(server->goal("test_gh"));

  // Check that our goal didn't get misplaced somehow
  EXPECT_EQ(server->top().id_, "test_gh");
  EXPECT_TRUE(server->isTracking("test_gh"));

  sm->handleEvents(Event(Signal::VertexFound));
#endif
  EXPECT_EQ(sm->name(), "::Teach::Branch");
  EXPECT_EQ(server->state_idx, 2);
  EXPECT_FALSE(server->aborted);
  EXPECT_FALSE(server->succeeded);

  // Check that our goal didn't get misplaced somehow
  EXPECT_EQ(server->top().id_, "test_gh");
  EXPECT_TRUE(server->isTracking("test_gh"));

  sm->handleEvents(Event(Action::EndGoal));
  // wait for async calls
  std::this_thread::sleep_for(std::chrono::milliseconds{200});
  EXPECT_EQ(sm->name(), "::Idle");
  EXPECT_EQ(server->state_idx, 3);
  EXPECT_FALSE(server->aborted);
  EXPECT_TRUE(server->succeeded);

  // Check that our goal is gone
  EXPECT_FALSE(server->isTracking("test_gh"));
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
#if 0
  // run `roscore` if you need the follwoing in your test.
  ros::init(argc, argv, "mission_planning_tests");
  ros::NodeHandle nh;
#endif
  return RUN_ALL_TESTS();
}