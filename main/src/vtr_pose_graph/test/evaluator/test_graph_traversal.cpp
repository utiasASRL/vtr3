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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/evaluator/evaluators.hpp>
#include <vtr_pose_graph/index/graph.hpp>

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;

class GraphStructureTestFixture : public Test {
 public:
  GraphStructureTestFixture() : graph_{new BasicGraph()} {}
  ~GraphStructureTestFixture() override {}

 protected:
  BasicGraph::Ptr graph_;
};

TEST_F(GraphStructureTestFixture, graph_traversal_basics) {
  /// Make sure that we can traverse a sub-graph, and the traversal is correct
  /// when there's only 1 vertex in graph.
  graph_->addRun();
  graph_->addVertex();

  using TemporalEvaluator = eval::Mask::TemporalDirect<BasicGraphBase>;
  auto tempeval = std::make_shared<TemporalEvaluator>();
  using DirectionEvaluator =
      eval::Mask::DirectionFromVertexDirect<BasicGraphBase>;
  auto direval = std::make_shared<DirectionEvaluator>(VertexId(0, 0), true);
  auto evaluator = eval::And(tempeval, direval);

  auto graph = std::dynamic_pointer_cast<BasicGraphBase>(graph_);

  evaluator->setGraph((void *)graph.get());
  const auto subgraph = graph->getSubgraph(VertexId(0, 0), 3, evaluator);

  using TemporalEvaluator2 = eval::Mask::TemporalDirect<BasicGraphBase>;
  auto tempeval2 = std::make_shared<TemporalEvaluator2>();
  using DirectionEvaluator2 =
      eval::Mask::DirectionFromVertexDirect<BasicGraphBase>;
  auto direval2 = std::make_shared<DirectionEvaluator2>(VertexId(0, 0), true);
  auto evaluator2 = eval::And(tempeval2, direval2);

  evaluator2->setGraph((void *)subgraph.get());
  auto itr = subgraph->beginDfs(VertexId(0, 0), 3, evaluator2);
  itr++;
}

int main(int argc, char **argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
