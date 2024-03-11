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
 * \file subgraph_tests.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/index/graph.hpp"

using namespace ::testing;
using namespace vtr::logging;
using namespace vtr::pose_graph;

class SubGraphTestFixture : public Test {
 public:
  SubGraphTestFixture() {}
  ~SubGraphTestFixture() override {}

  void SetUp() override {
    /* Create the following graph
     * R0: 0 --- 1 --- 2
     *       \
     *        \
     *         \
     * R1: 0 --- 1 --- 2 -- 250
     *                 |
     * R2: 0 --- 1 --- 2 -- 250
     *           |
     * R3: 0 --- 1 --- 2 -- 250
     *                 |
     * R4: 0 --- 1 --- 2 -- 250
     */
    // clang-format off
    // Add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      // Create the robochunk directories
      graph_->addRun();
      graph_->addVertex();
      for (int vidx = 1; vidx < 250; ++vidx) {
        graph_->addVertex();
        graph_->addEdge(VertexId(idx, vidx - 1), VertexId(idx, vidx), EdgeType::Temporal, (idx == 0 || idx == 2) ? true : false, EdgeTransform(true));
      }
    }
    // Add spatial edge across runs.
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), EdgeType::Spatial, false, EdgeTransform(true));
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), EdgeType::Spatial, false, EdgeTransform(true));
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), EdgeType::Spatial, false, EdgeTransform(true));
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), EdgeType::Spatial, false, EdgeTransform(true));
    // clang-format on
  }

  void TearDown() override {}

  BasicGraph::Ptr graph_ = std::make_shared<BasicGraph>();
};

TEST_F(SubGraphTestFixture, SubGraphPrivilgedMask) {
  using PrivilegedEval = eval::mask::privileged::Eval<BasicGraph>;
  auto evaluator = std::make_shared<PrivilegedEval>(*graph_);
  auto sub_graph = graph_->getSubgraph(VertexId(0, 0), evaluator);

  int count = 0;
  for (auto itr = sub_graph->beginEdge(); itr != sub_graph->endEdge(); ++itr)
    count++;
  EXPECT_EQ(count, 249);
}

TEST_F(SubGraphTestFixture, SubGraphPrivilegedPath) {
  // Get only the privileged edges.
  using PrivilegedEval = eval::mask::privileged::Eval<BasicGraph>;
  auto evaluator = std::make_shared<PrivilegedEval>(*graph_);
  auto sub_graph = graph_->getSubgraph(VertexId(0, 0), evaluator);

  // extract a path from the sub graph
  auto path = sub_graph->breadthFirstSearch(VertexId(0, 15), VertexId(0, 25));
  // start at the begining.
  auto itr = path->begin(VertexId(0, 15));
  EXPECT_EQ(itr->v()->id(), VertexId(0, 15));
  itr++;
  int count = 0;
  std::stringstream ss;
  for (; itr != path->end(); ++itr) {
    count++;
    ss << itr->e()->id() << ", ";
  }
  EXPECT_EQ(count, 10);
  CLOG(INFO, "test") << ss.str();
}


TEST_F(SubGraphTestFixture, SubGraphSpecificRunsPath) {
  // Get only the privileged edges.
  using RunSelector = eval::mask::runs::Eval<BasicGraph>;
  using RunId = BaseIdType;

  auto select_runs = std::set<RunId> {2, 4};

  CLOG(INFO, "test") << "selected runs" << select_runs.count(2);


  auto evaluator = std::make_shared<RunSelector>(*graph_, select_runs);
  auto sub_graph = graph_->getSubgraph(VertexId(2, 0), evaluator);
  CLOG(INFO, "test") << sub_graph->numberOfVertices();

  EXPECT_TRUE(sub_graph->contains(VertexId(2, 0)));
  EXPECT_FALSE(sub_graph->contains(VertexId(0, 0)));
  auto itr = sub_graph->begin(VertexId(2, 15));
  int count = 0;
  std::stringstream ss;
  for (; itr != sub_graph->end(); ++itr) {
    count++;
    ss << itr->e()->id() << ", ";
  }
  EXPECT_EQ(count, 235);
  CLOG(INFO, "test") << ss.str();
}

int main(int argc, char** argv) {
  configureLogging("", true, {"test"});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}