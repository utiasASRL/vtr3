#pragma once

#include <vtr_mission_planning/base_mission_server.hpp>
#include <vtr_mission_planning/state_machine.hpp>

using namespace vtr::mission_planning;
using namespace vtr::path_planning;
using state::Action;
using state::BaseState;
using state::Event;
using state::Signal;
using state::StateMachine;

/** Test goal handle to ensure that the mission planner makes correct calls. */
struct TestGoalHandle {
  using Id = float;
  Id id;
  Target target;
  std::list<VertexId> path{VertexId{}, VertexId{}};
  VertexId vertex;
  std::chrono::milliseconds pause_before{0};
  std::chrono::milliseconds pause_after{0};
};

/**
 * Test tactic to ensure that the state machine makes the correct calls to the
 * tactic.
 */
struct TestTactic : public StateMachineInterface {
 public:
  PTR_TYPEDEFS(TestTactic);

  TestTactic()
      : pipeline_(PipelineMode::Idle),
        closest_(VertexId::Invalid()),
        current_(VertexId::Invalid()) {
    status_.localization_ = LocalizationStatus::DeadReckoning;
    status_.safety_ = SafetyStatus::NotThatSafe;
  }

  void setPipeline(const PipelineMode& pipeline) {
    pipeline_ = pipeline;
    LOG(INFO) << "Switching pipeline to " << static_cast<int>(pipeline_);
  }
  LockType lockPipeline() { return LockType(); }
  void setPath(const PathType&, bool) {}
  const Localization& persistentLoc() const { return loc_; }
  const Localization& targetLoc() const { return loc_; }
  void setTrunk(const VertexId&) {}  // not important for state machine testing
  double distanceToSeqId(const uint64_t&) { return 9001; }
  TacticStatus status() const { return status_; }
  LocalizationStatus tfStatus(
      const vtr::pose_graph::RCEdge::TransformType& tf) const {
    return LocalizationStatus::Forced;  // not important for state machine
                                        // testing
  }
  const VertexId& closestVertexID() const { return closest_; }  // not important
  const VertexId& currentVertexID() const { return current_; }  // not important
  const bool canCloseLoop() const { return false; }
  void connectToTrunk(bool) {}
  void addRun(bool, bool, bool) { LOG(INFO) << "Adding a new run"; }
#if 0
  void removeEphemeralRuns() {}
#endif
  void relaxGraph() {}
  void saveGraph() {}

  PipelineMode pipeline_;
  VertexId closest_;  // not important for state machine testing
  VertexId current_;  // not important for state machine testing
  TacticStatus status_;
  Localization loc_;
};

/** Test path planner to ensure that the state machine makes the correct
 * callbacks to the path planner.
 */
class TestPathPlanner : public vtr::path_planning::PlanningInterface {
 public:
  PTR_TYPEDEFS(TestPathPlanner)
  PathType path(const VertexId&, const VertexId&) { return PathType{}; }
  PathType path(const VertexId&, const VertexId::List&, std::list<uint64_t>*) {
    return PathType();
  }
  void updatePrivileged() {}
  double cost(const VertexId&) { return 0.0; }
  double cost(const EdgeId&) { return 0.0; }
  EvalPtr cost() {
    return vtr::pose_graph::eval::Weight::Const::MakeShared(0, 0);
  }
};