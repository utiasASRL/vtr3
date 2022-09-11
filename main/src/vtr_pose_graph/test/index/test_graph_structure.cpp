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
 * \file graph_structure_tests.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/index/graph.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;

class GraphStructureTestFixture : public Test {
 public:
  GraphStructureTestFixture() {}
  ~GraphStructureTestFixture() override {}

 protected:
  BasicGraph::Ptr graph_ = BasicGraph::MakeShared();
};

TEST_F(GraphStructureTestFixture, graph_construction) {
  // make sure we can give it a bogus ID and not break.
  const auto &run_id = graph_->addRun();
  EXPECT_EQ(run_id, (unsigned)0);

  VertexId vid(0, 200);
  EXPECT_THROW(graph_->at(vid), std::range_error);

  EdgeId eid(VertexId(0, 0), VertexId(0, 50));
  EXPECT_THROW(graph_->at(eid), std::range_error);
}

TEST_F(GraphStructureTestFixture, vertex_addition_with_run) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
}

TEST_F(GraphStructureTestFixture, vertex_addition_no_run) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
  VertexId vid(0, 0);
  auto vertex = graph_->at(vid);
  EXPECT_EQ(vertex->id(), vid);
  EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
  EXPECT_EQ(vertex->id().minorId(), (unsigned)0);
}

TEST_F(GraphStructureTestFixture, multiple_vertex_addition_and_access) {
  EXPECT_NO_THROW(graph_->addRun());
  for (int idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->addVertex());
  }

  for (uint32_t idx = 0; idx < 10; ++idx) {
    VertexId vid(0, idx);
    EXPECT_TRUE(graph_->contains(vid));
    auto vertex = graph_->at(vid);
    EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
    EXPECT_EQ(vertex->id().minorId(), (unsigned)idx);
  }
}

TEST_F(GraphStructureTestFixture, multiple_run_vertex_addition) {
  // add a run with 10 vertices.
  EXPECT_NO_THROW(graph_->addRun());
  for (int idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->addVertex());
  }

  // add a second run with 10 vertices.
  EXPECT_NO_THROW(graph_->addRun());
  for (int idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->addVertex());
  }

  // make sure vertices in run 0 and 1 exist, but do not in run 2.
  for (uint64_t idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->at(VertexId(0, idx)));
    EXPECT_NO_THROW(graph_->at(VertexId(1, idx)));
    EXPECT_THROW(graph_->at(VertexId(2, idx)), std::range_error);
  }
}

TEST_F(GraphStructureTestFixture, add_edge_to_empty_graph) {
  EXPECT_THROW(graph_->addEdge(VertexId(0, 0), VertexId(0, 1),
                               EdgeType::Temporal, false, EdgeTransform(true)),
               std::range_error);
}

class EdgeTestFixture : public Test {
 public:
  EdgeTestFixture() {
    // add a run with 10 vertices.
    graph_->addRun();
    for (int idx = 0; idx < 10; ++idx) graph_->addVertex();
    // add a run with 10 vertices.
    graph_->addRun();
    for (int idx = 0; idx < 10; ++idx) graph_->addVertex();
  }

 protected:
  BasicGraph::Ptr graph_ = BasicGraph::MakeShared();
};

TEST_F(EdgeTestFixture, iterator_test_bfs_dfs) {
  for (int idx = 0; idx < 9; ++idx)
    graph_->addEdge(VertexId(0, idx), VertexId(0, idx + 1), EdgeType::Temporal,
                    true, EdgeTransform(true));

  /// \note Never use iterator of a graph that may be used in a multi-threading
  /// context!! Call getsubgraph first to get only portion you need. This is
  /// because the underlying simple graph can change during iterating, but we
  /// have no easy way to protect it.

  {
    auto itr = graph_->beginBfs(VertexId(0, 9));
    std::stringstream ss;
    ss << "BFS child: ";
    for (; itr != graph_->end(); itr++) ss << itr->v()->id() << ", ";
    CLOG(INFO, "test") << ss.str();
  }
  {
    auto itr = graph_->beginDfs(VertexId(0, 9));
    std::stringstream ss;
    ss << "DFS child: ";
    for (; itr != graph_->end(); itr++) ss << itr->v()->id() << ", ";
    CLOG(INFO, "test") << ss.str();
  }
}

TEST_F(EdgeTestFixture, add_temporal_edge) {
  // make an edge from (0,0)--(0,1)
  VertexId from(0, 0);
  VertexId to(0, 1);

  auto edge =
      graph_->addEdge(from, to, EdgeType::Temporal, true, EdgeTransform(true));
  EXPECT_TRUE(edge != nullptr);
  EXPECT_TRUE(edge->isTemporal());
  EXPECT_FALSE(edge->isSpatial());
  EXPECT_TRUE(edge->isManual());
  EXPECT_FALSE(edge->isAutonomous());

  // Extract the edge based off of the vertex ids
  EdgeId eid(from, to);
  EXPECT_NO_THROW(graph_->at(eid));
}

TEST_F(EdgeTestFixture, add_spatial_edge) {
  // make an edge from (0,0)--(0,1)
  VertexId from(0, 0);
  VertexId to(0, 1);

  auto edge =
      graph_->addEdge(from, to, EdgeType::Spatial, false, EdgeTransform(true));
  EXPECT_TRUE(edge != nullptr);
  EXPECT_FALSE(edge->isTemporal());
  EXPECT_TRUE(edge->isSpatial());
  EXPECT_FALSE(edge->isManual());
  EXPECT_TRUE(edge->isAutonomous());
}

TEST_F(EdgeTestFixture, cross_run_edge) {
  // make an edge from (0,5)--(1,6)
  VertexId to(0, 5);
  VertexId from(1, 6);

  auto edge =
      graph_->addEdge(from, to, EdgeType::Spatial, false, EdgeTransform(true));
  EXPECT_TRUE(edge != nullptr);
  EXPECT_FALSE(edge->isTemporal());
  EXPECT_TRUE(edge->isSpatial());
  EXPECT_FALSE(edge->isManual());
  EXPECT_TRUE(edge->isAutonomous());

  // Cannot add from low runs to high ones
  EXPECT_THROW(
      graph_->addEdge(to, from, EdgeType::Spatial, true, EdgeTransform(true)),
      std::invalid_argument);
}

int main(int argc, char **argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
