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

#include <iostream>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/index/graph.hpp>

using namespace ::testing;
using namespace vtr::logging;
using namespace vtr::pose_graph;

class SubGraphTestFixture : public Test {
 public:
  SubGraphTestFixture() {}
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
        graph_->addEdge(VertexId(idx, vidx - 1), VertexId(idx, vidx),
                        EdgeType::Temporal, idx == 0 ? true : false,
                        EdgeTransform(true));
      }
    }
    // Add spatial edge across runs.
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), EdgeType::Spatial, false,
                    EdgeTransform(true));
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), EdgeType::Spatial, false,
                    EdgeTransform(true));
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), EdgeType::Spatial, false,
                    EdgeTransform(true));
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), EdgeType::Spatial, false,
                    EdgeTransform(true));

    // set the edge's transform to something special;
    for (auto itr = graph_->beginEdge(); itr != graph_->beginEdge(); ++itr) {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(0, 3) = itr->id().majorId2();
      transform(1, 3) = itr->id().minorId2();
      transform(2, 3) = (double)itr->type();
      itr->setTransform(EdgeTransform(transform));
    }
  }

  void TearDown() override {}

 protected:
  BasicGraph::Ptr graph_ = BasicGraph::MakeShared();
};

TEST_F(SubGraphTestFixture, SubGraphFromVertexList) {
  std::vector<VertexId> vertices;
  vertices.push_back(VertexId(0, 0));
  vertices.push_back(VertexId(0, 1));
  vertices.push_back(VertexId(0, 2));
  auto sub_graph = graph_->getSubgraph(vertices);

  int count = 0;
  for (auto itr = sub_graph->beginEdge(); itr != sub_graph->endEdge(); ++itr)
    count++;
  EXPECT_EQ(count, 2);

  count = 0;
  for (auto itr = sub_graph->beginVertex(); itr != sub_graph->endVertex();
       ++itr)
    count++;
  EXPECT_EQ(count, 3);
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}