#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vtr/planning/state_machine.h>

#include <asrl/common/logging.hpp>
// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING
// easylogging++.h **
INITIALIZE_EASYLOGGINGPP

using namespace vtr::planning;
using state::Action;
using state::BaseState;
using state::Event;
using state::StateMachine;

/** Test tactic to ensure that the state machine makes the correct calls to the
 * tactic.
 */
struct TestTactic : public StateMachineInterface {
 public:
  PTR_TYPEDEFS(TestTactic)

  TestTactic()
      : pipeline_(PipelineType::Idle)
#if 0
      ,closest_(VertexId::Invalid()),
        current_(VertexId::Invalid())
#endif
  {
#if 0
    status_.localization_ = LocalizationStatus::DeadReckoning;
    status_.safety_ = SafetyStatus::NotThatSafe;
#endif
  }

  void setPipeline(const PipelineType& pipeline) { pipeline_ = pipeline; }
  LockType lockPipeline() { return LockType(); }
  void setPath(const PathType&, bool) {}
#if 0
  const Localization& persistentLoc() const { return loc_; }
  const Localization& targetLoc() const { return loc_; }
  bool startHover(const PathType&) { return true; }
  bool startFollow(const PathType&) { return true; }
  void setTrunk(const VertexId&) {}
  double distanceToSeqId(const uint64_t&) { return 9001; }
  TacticStatus status() const { return status_; }
  const VertexId& closestVertexID() const { return closest_; }
  const VertexId& currentVertexID() const { return current_; }
  const VertexId& connectToTrunk(bool) { return closest_; }
  void addRun(bool, bool, bool) {}
  void removeEphemeralRuns() {}
  void relaxGraph() {}
#endif

  PipelineType pipeline_;
#if 0
  VertexId closest_;
  VertexId current_;
  TacticStatus status_;
  Localization loc_;
#endif
};

/** Test callback to ensure that the state machine makes the correct callbacks
 * to the mission planning server.
 */
class TestCallbacks : public StateMachineCallbacks {
 public:
  PTR_TYPEDEFS(TestCallbacks)

  void stateChanged(const __shared_ptr<state::BaseState>&) {}
  void stateSuccess() {}
  void stateAbort(const std::string&) {}
  void stateUpdate(double) {}
};

/** Convenience class to create one of every State. */
struct StateContainer {
  StateContainer()
      : idle(new state::Idle())
#if 0
      ,repeatTopoLoc(new state::repeat::TopologicalLocalize()),
        plan(new state::repeat::Plan()),
        metricLoc(new state::repeat::MetricLocalize()),
        follow(new state::repeat::Follow()),
        teachTopoLoc(new state::teach::TopologicalLocalize()),
        branch(new state::teach::Branch()),
        merge(new state::teach::Merge())
#endif
  {
  }

  StateContainer(const state::BaseState& sm)
      : idle(new state::Idle(sm))
#if 0
      ,repeatTopoLoc(new state::repeat::TopologicalLocalize(sm)),
        plan(new state::repeat::Plan(sm)),
        metricLoc(new state::repeat::MetricLocalize(sm)),
        follow(new state::repeat::Follow(sm)),
        teachTopoLoc(new state::teach::TopologicalLocalize(sm)),
        branch(new state::teach::Branch(sm)),
        merge(new state::teach::Merge(sm))
#endif
  {
  }

  BaseState::Ptr idle;
#if 0
  BaseState::Ptr repeatTopoLoc;
  BaseState::Ptr plan;
  BaseState::Ptr metricLoc;
  BaseState::Ptr follow;

  BaseState::Ptr teachTopoLoc;
  BaseState::Ptr branch;
  BaseState::Ptr merge;
#endif
};

#if 0
/** Convenience class to create one of every Event, and swap out the goals.*/
struct EventContainer {
  EventContainer(const BaseState::Ptr& goal = nullptr)
      : Abort(Action::Abort),
        Continue(Action::Continue),
        NewGoal(Action::NewGoal, goal),
        SwapGoal(Action::SwapGoal, goal),
        AppendGoal(Action::AppendGoal, goal),
        EndGoal(Action::EndGoal) {}

  void setGoal(const BaseState::Ptr& goal = nullptr) {
    NewGoal.goal_ = goal;
    SwapGoal.goal_ = goal;
    AppendGoal.goal_ = goal;
  }

  Event Abort;
  Event Continue;
  Event NewGoal;
  Event SwapGoal;
  Event AppendGoal;
  Event EndGoal;
};
#endif

/** Test transition from A state to B state. */
TEST(StateTransition, Idle) {
  StateContainer p;
  EXPECT_EQ(p.idle.get()->nextStep(p.idle.get()), nullptr);
}

/** Ensure the state machine can handle all events properly. */
TEST(EventHandling, EventHandling) {
  TestTactic::Ptr tactic(new TestTactic());
  TestCallbacks::Ptr callbacks(new TestCallbacks());
  StateMachine::Ptr state_machine = StateMachine::InitialState(callbacks.get());

  state_machine->setTactic(tactic.get());
#if 0
  state_machine->setPlanner(NullPlanner::Ptr(new NullPlanner()));
#endif

  // Start in idle
  EXPECT_EQ(state_machine->name(), "::Idle");
  EXPECT_EQ(state_machine->goals().size(), 1);

  // Handle idle: nothing should have changed
  state_machine->handleEvents(Event::StartIdle());
  EXPECT_EQ(state_machine->name(), "::Idle");
  EXPECT_EQ(state_machine->goals().size(), 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
#if 0
  ros::init(argc, argv, "state_machine_tests");
  ros::NodeHandle nh;
#endif
  return RUN_ALL_TESTS();
}
