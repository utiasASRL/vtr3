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
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include <vtr_logging/logging_init.hpp>
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

TEST_F(GraphStructureTestFixture, graph_construction) {
  // make sure vertex map is accessable (and empty)
  auto vertex_map = graph_->vertices();
  EXPECT_TRUE(vertex_map != nullptr);
  EXPECT_EQ(vertex_map->unlocked().get().size(), (unsigned)0);

  // make sure edge map is accessable (and empty)
  auto edge_map = graph_->edges();
  EXPECT_TRUE(edge_map != nullptr);
  EXPECT_EQ(edge_map->unlocked().get().size(), (unsigned)0);

  // make sure we can give it a bogus ID and not break.
  const auto &run_id = graph_->addRun();
  EXPECT_EQ(run_id, (unsigned)0);

  VertexBase::IdType vertex_id(0, 200);
  EXPECT_THROW(graph_->at(vertex_id), std::range_error);

  EdgeBase::IdType edge_id(0, 50, Temporal);
  EXPECT_THROW(graph_->at(edge_id), std::range_error);
}

TEST_F(GraphStructureTestFixture, vertex_addition_with_run) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
}

TEST_F(GraphStructureTestFixture, vertex_addition_to_run) {
  EXPECT_NO_THROW(graph_->addRun());

  /// \note we use argument forwarding when calling these functions so that type
  /// must match exactly. The input has to be "const BasicGraph::RunIdType"

  const BasicGraph::RunIdType run_id(0);
  EXPECT_NO_THROW(graph_->addVertex(run_id));

  const BasicGraph::RunIdType run_id2(1);
  EXPECT_THROW(graph_->addVertex(run_id2), std::out_of_range);
}

TEST_F(GraphStructureTestFixture, vertex_addition_no_run) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
  VertexBase::IdType vertex_id(0, 0);
  auto vertex = graph_->at(vertex_id);
  EXPECT_EQ(vertex->id(), vertex_id);
  EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
  EXPECT_EQ(vertex->id().minorId(), (unsigned)0);
  EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
  EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
}

TEST_F(GraphStructureTestFixture, vertex_addition_map_access) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
  /// \note lock access in multi-threading context
  const auto &vertex_map = graph_->vertices()->unlocked().get();
  EXPECT_FALSE(vertex_map.empty());
  for (auto itr = vertex_map.begin(); itr != vertex_map.end(); ++itr) {
    auto vertex = itr->second;
    EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
    EXPECT_EQ(vertex->id().minorId(), (unsigned)0);
    EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
    EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
  }
}

TEST_F(GraphStructureTestFixture, multiple_vertex_addition_and_access) {
  EXPECT_NO_THROW(graph_->addRun());

  for (int idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->addVertex());
  }

  for (uint32_t idx = 0; idx < 10; ++idx) {
    VertexBase::IdType vertex_id(0, idx);
    EXPECT_TRUE(graph_->contains(vertex_id));
    auto vertex = graph_->at(vertex_id);
    EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
    EXPECT_EQ(vertex->id().minorId(), (unsigned)idx);
    EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
    EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
  }
}

TEST_F(GraphStructureTestFixture, multiple_vertex_addition_map_access) {
  EXPECT_NO_THROW(graph_->addRun());

  auto &vertex_map = graph_->vertices()->unlocked().get();
  for (int idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->addVertex());
  }
  EXPECT_EQ(vertex_map.size(), (unsigned)10);

  uint32_t run_id = 0;
  uint32_t vertex_id = 0;
  for (auto itr = vertex_map.begin(); itr != vertex_map.end(); ++itr) {
    auto vertex = itr->second;
    EXPECT_EQ(vertex->id().majorId(), (unsigned)run_id);

    EXPECT_LE(vertex->id().minorId(), (unsigned)10);
    EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
    EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
    vertex_id++;
  }
  EXPECT_EQ(vertex_id, (unsigned)10);
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
    auto vertex0 = graph_->at(VertexBase::IdType(0, idx));
    auto vertex1 = graph_->at(VertexBase::IdType(1, idx));
    EXPECT_THROW(graph_->at(VertexBase::IdType(2, idx)), std::range_error);
    EXPECT_TRUE(vertex0 != nullptr);
    EXPECT_TRUE(vertex1 != nullptr);
  }
}

TEST_F(GraphStructureTestFixture, add_edge_to_empty_graph) {
  EXPECT_THROW(
      graph_->addEdge(VertexBase::IdType(0, 0), VertexBase::IdType(0, 1),
                      EdgeBase::EnumType::Temporal),
      std::out_of_range);
}

class EdgeTestFixture : public Test {
 public:
  EdgeTestFixture() : graph_{new BasicGraph()} {
    // add a run with 10 vertices.
    graph_->addRun();
    for (int idx = 0; idx < 10; ++idx) graph_->addVertex();
    // add a run with 10 vertices.
    graph_->addRun();
    for (int idx = 0; idx < 10; ++idx) graph_->addVertex();
  }

 protected:
  BasicGraph::Ptr graph_;
};

TEST_F(EdgeTestFixture, iterator_test_bfs_dfs) {
  for (int idx = 0; idx < 9; ++idx)
    graph_->addEdge(VertexBase::IdType(0, idx), VertexBase::IdType(0, idx + 1),
                    EdgeBase::EnumType::Temporal);

  /// \note Never use iterator of a graph that may be used in a multi-threading
  /// context!! Call getsubgraph first to get only portion you need. This is
  /// because the underlying simple graph can change during iterating, but we
  /// have no easy way to protect it.

  {
    auto itr = graph_->beginBfs(VertexBase::IdType(0, 9));
    std::stringstream ss;
    ss << "BFS child: ";
    for (; itr != graph_->end(); itr++) ss << itr->v()->id() << ", ";
    LOG(INFO) << ss.str();
  }
  {
    auto itr = graph_->beginDfs(VertexBase::IdType(0, 9));
    std::stringstream ss;
    ss << "DFS child: ";
    for (; itr != graph_->end(); itr++) ss << itr->v()->id() << ", ";
    LOG(INFO) << ss.str();
  }
}

TEST_F(EdgeTestFixture, add_temporal_edge) {
  // make an edge from (0,0)--(0,1)
  VertexBase::IdType from(0, 0);
  VertexBase::IdType to(0, 1);

  auto edge = graph_->addEdge(from, to, Temporal, true);
  EXPECT_TRUE(edge != nullptr);
  EXPECT_TRUE(edge->isTemporal());
  EXPECT_FALSE(edge->isSpatial());
  EXPECT_TRUE(edge->isManual());
  EXPECT_FALSE(edge->isAutonomous());

  auto incident_vertices = edge->incident();
  EXPECT_EQ(incident_vertices.first, from);
  EXPECT_EQ(incident_vertices.second, to);

  auto &edge_map = graph_->edges()->unlocked().get();
  EXPECT_EQ(edge_map.size(), (unsigned)1);

  EdgeBase::IdType edge_id(from, to, Temporal);
  EdgeBase::IdType edge_id_spatial(from, to, Spatial);

  // Check the vertices edge list.
  auto vertex = graph_->at(from);
  EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)1);
  EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);

  // Extract the edge based off of the vertex ids
  EXPECT_NO_THROW(graph_->at(*vertex->temporalEdges().rbegin()));
  EXPECT_NO_THROW(graph_->at(edge_id));
  EXPECT_THROW(graph_->at(edge_id_spatial), std::range_error);
}

TEST_F(EdgeTestFixture, add_spatial_edge) {
  // make an edge from (0,0)--(0,1)
  VertexBase::IdType from(0, 0);
  VertexBase::IdType to(0, 1);

  auto edge = graph_->addEdge(from, to, Spatial);
  EXPECT_TRUE(edge != nullptr);
  EXPECT_FALSE(edge->isTemporal());
  EXPECT_TRUE(edge->isSpatial());
  EXPECT_FALSE(edge->isManual());
  EXPECT_TRUE(edge->isAutonomous());

  auto incident_vertices = edge->incident();
  EXPECT_EQ(incident_vertices.first, from);
  EXPECT_EQ(incident_vertices.second, to);

  auto &edge_map = graph_->edges()->unlocked().get();
  EXPECT_EQ(edge_map.size(), (unsigned)1);
}

TEST_F(EdgeTestFixture, cross_run_edge) {
  // make an edge from (0,5)--(1,6)
  VertexBase::IdType to(0, 5);
  VertexBase::IdType from(1, 6);

  auto edge = graph_->addEdge(from, to, Spatial);
  EXPECT_TRUE(edge != nullptr);

  EXPECT_FALSE(edge->isTemporal());
  EXPECT_TRUE(edge->isSpatial());
  EXPECT_FALSE(edge->isManual());
  EXPECT_TRUE(edge->isAutonomous());

  // Cannot add from low runs to high ones
  EXPECT_THROW(graph_->addEdge(to, from, Spatial), std::invalid_argument);

  auto incident_vertices = edge->incident();
  EXPECT_EQ(incident_vertices.first, from);
  EXPECT_EQ(incident_vertices.second, to);

  auto &edge_map = graph_->edges()->unlocked().get();
  EXPECT_EQ(edge_map.size(), (unsigned)1);
}

int main(int argc, char **argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
