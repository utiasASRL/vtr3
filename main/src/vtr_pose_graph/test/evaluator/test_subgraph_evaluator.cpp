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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/evaluator/evaluators.hpp>

using namespace ::testing;
using namespace vtr::logging;
using namespace vtr::pose_graph;

class SubGraphTestFixture : public Test {
 public:
  SubGraphTestFixture() : graph_(new BasicGraph()) {}

  ~SubGraphTestFixture() {}

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

    // Add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      // Create the robochunk directories
      graph_->addRun();
      graph_->addVertex();
      for (int vidx = 1; vidx < 250; ++vidx) {
        graph_->addVertex();
        graph_->addEdge(VertexId(idx, vidx - 1), VertexId(idx, vidx), Temporal,
                        idx == 0 ? true : false);
      }
    }
    // Add spatial edge across runs.
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), Spatial);
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), Spatial);
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), Spatial);
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), Spatial);

    // set the edge's transform to something special;
    const auto& edge_map = graph_->edges()->unlocked().get();
    for (auto itr = edge_map.begin(); itr != edge_map.end(); ++itr) {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(0, 3) = itr->second->id().majorId2();
      transform(1, 3) = itr->second->id().minorId2();
      transform(2, 3) = itr->second->id().type();
      itr->second->setTransform(lgmath::se3::Transformation(transform));
    }
  }

  void TearDown() override {}

 protected:
  /// std::unique_ptr<RCGraph> graph_;
  BasicGraph::Ptr graph_;
};

TEST_F(SubGraphTestFixture, SubGraphPrivilgedMask) {
  using PrivilegedEvaluator = eval::Mask::Privileged<BasicGraph>::Direct;
  PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
  evaluator->setGraph(graph_.get());
  auto sub_graph = graph_->getSubgraph(VertexId(0, 0), evaluator);

  int count = 0;
  for (auto itr = sub_graph->beginEdge(); itr != sub_graph->endEdge(); ++itr)
    count++;

  EXPECT_EQ(count, 249);
}

TEST_F(SubGraphTestFixture, SubGraphPrivilegedPath) {
  // Get only the privileged edges.
  using PrivilegedEvaluator = eval::Mask::Privileged<BasicGraph>::Direct;
  auto evaluator = std::make_shared<PrivilegedEvaluator>();
  evaluator->setGraph(graph_.get());
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
  LOG(INFO) << ss.str();
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}