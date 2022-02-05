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
 * \file test_graph_traversal.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/index/graph.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;

class GraphStructureTestFixture : public Test {
 public:
  GraphStructureTestFixture() {}
  ~GraphStructureTestFixture() override {}

  BasicGraph::Ptr graph_ = std::make_shared<BasicGraph>();
};

TEST_F(GraphStructureTestFixture, graph_traversal_basics) {
  /// Make sure that we can traverse a sub-graph, and the traversal is correct
  /// when there's only 1 vertex in graph.
  graph_->addRun();
  graph_->addVertex();

  using TemporalEval = eval::mask::temporal::Eval<BasicGraph>;
  using DirectionEval = eval::mask::direction_from_vertex::Eval;

  auto tempeval = std::make_shared<TemporalEval>(*graph_);
  auto direval = std::make_shared<DirectionEval>(VertexId(0, 0), true);
  auto eval = eval::And(tempeval, direval);
  auto subgraph = graph_->getSubgraph(VertexId(0, 0), 3, eval);

  auto tempeval2 = std::make_shared<TemporalEval>(*graph_);
  auto direval2 = std::make_shared<DirectionEval>(VertexId(0, 0), true);
  auto eval2 = eval::And(tempeval2, direval2);
  auto itr = subgraph->beginDfs(VertexId(0, 0), 3, eval2);
  itr++;
}

int main(int argc, char **argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
