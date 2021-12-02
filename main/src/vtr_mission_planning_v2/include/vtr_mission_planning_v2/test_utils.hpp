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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_mission_planning_v2/state_machine/state_machine.hpp"

using namespace vtr;
using namespace vtr::mission_planning;

namespace vtr {
namespace mission_planning {

struct TestTactic : public StateMachine::Tactic {
  PTR_TYPEDEFS(TestTactic);

  UniqueLock lockPipeline() override {
    LOG(INFO) << "Locking pipeline";
    return UniqueLock();
  }

  void setPipeline(const tactic::PipelineMode& pipeline) override {
    LOG(INFO) << "Switching pipeline to " << static_cast<int>(pipeline);
  }

  void setPath(const tactic::PathType&, bool) override {}
  void setTrunk(const tactic::VertexId&) override {}
  double distanceToSeqId(const uint64_t&) override { return 9001; }
  void addRun(bool) override { LOG(INFO) << "Adding a new run"; }
  bool pathFollowingDone() override { return true; }
  bool canCloseLoop() const override { return false; }
  void connectToTrunk(bool, bool) override {}
  void relaxGraph() override {}
  void saveGraph() override {}
  const tactic::Localization& persistentLoc() const override { return loc_; }

  tactic::PipelineMode pipeline_;
  tactic::Localization loc_;
};

struct TestRoutePlanner : public RoutePlannerInterface {
  PTR_TYPEDEFS(TestRoutePlanner);
  // clang-format off
  PathType path(const VertexId&, const VertexId&) override { return PathType{}; }
  PathType path(const VertexId&, const VertexId::List&, std::list<uint64_t>*) override {return PathType();}
  // clang-format on
};

struct TestCallback : public StateMachineCallback {
  PTR_TYPEDEFS(TestCallback);
  void stateAbort(const std::string&) override {}
  void stateSuccess() override {}
  void stateUpdate(double) override {}
};

}  // namespace mission_planning
}  // namespace vtr