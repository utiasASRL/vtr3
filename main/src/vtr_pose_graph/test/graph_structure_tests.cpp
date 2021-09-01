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
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>

#include <vtr_pose_graph/index/graph.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_edge.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>

using namespace vtr::pose_graph;

class GraphTest : public ::testing::Test {
 public:
  GraphTest() : graph_{new Graph<RCVertex, RCEdge, RCRun>()} {}
  void SetUp() override {}
  void TearDown() override {}
  ~GraphTest() override {}

 protected:
  Graph<RCVertex, RCEdge, RCRun>::Ptr graph_;
};

TEST_F(GraphTest, GraphConstruction) {
  // make sure vertex map is accessable (and empty)
  auto vertex_map = graph_->vertices();
  EXPECT_TRUE(vertex_map != nullptr);
  EXPECT_EQ(vertex_map->size(), (unsigned)0);

  // make sure edge map is accessable (and empty)
  auto edge_map = graph_->edges();
  EXPECT_TRUE(edge_map != nullptr);
  EXPECT_EQ(edge_map->size(), (unsigned)0);

  // Make sure we can give it a bogus ID and not break.
  graph_->addRun();
  RCVertex::IdType vertex_id(0, 200);
  EXPECT_THROW(graph_->at(vertex_id), std::range_error);

  RCEdge::IdType edge_id(0, 50, Temporal);
  EXPECT_THROW(graph_->at(edge_id), std::range_error);
}

TEST_F(GraphTest, RunAddition) {
  auto run_id = graph_->addRun();
  EXPECT_EQ(run_id, (unsigned)0);
}

TEST_F(GraphTest, VertexAdditionWithRun) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
}

TEST_F(GraphTest, VertexAdditionToRun) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex(0));
}

TEST_F(GraphTest, VertexAdditionNoRun) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
  RCVertex::IdType vertex_id(0, 0);
  auto vertex = graph_->at(vertex_id);
  EXPECT_EQ(vertex->id(), vertex_id);
  EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
  EXPECT_EQ(vertex->id().minorId(), (unsigned)0);
  EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
  EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
}

TEST_F(GraphTest, VertexAdditionMapAccess) {
  EXPECT_NO_THROW(graph_->addRun());
  EXPECT_NO_THROW(graph_->addVertex());
  auto vertex_map = graph_->vertices();
  EXPECT_FALSE(vertex_map->empty());
  for (auto itr = vertex_map->begin(); itr != vertex_map->end(); ++itr) {
    auto vertex = itr->second;
    EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
    EXPECT_EQ(vertex->id().minorId(), (unsigned)0);
    EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
    EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
  }
}

TEST_F(GraphTest, MultiVertexAdditionAtaAccess) {
  EXPECT_NO_THROW(graph_->addRun());

  for (int idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->addVertex());
  }

  for (uint32_t idx = 0; idx < 10; ++idx) {
    RCVertex::IdType vertex_id(0, idx);
    EXPECT_TRUE(graph_->contains(vertex_id));
    auto vertex = graph_->at(vertex_id);
    EXPECT_EQ(vertex->id().majorId(), (unsigned)0);
    EXPECT_EQ(vertex->id().minorId(), (unsigned)idx);
    EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
    EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
  }
}

TEST_F(GraphTest, MultiVertexAdditionMapAccess) {
  EXPECT_NO_THROW(graph_->addRun());

  auto vertex_map = graph_->vertices();
  for (int idx = 0; idx < 10; ++idx) {
    EXPECT_NO_THROW(graph_->addVertex());
  }
  EXPECT_EQ(vertex_map->size(), (unsigned)10);

  uint32_t run_id = 0;
  uint32_t vertex_id = 0;
  for (auto itr = vertex_map->begin(); itr != vertex_map->end(); ++itr) {
    auto vertex = itr->second;
    EXPECT_EQ(vertex->id().majorId(), (unsigned)run_id);

    EXPECT_LE(vertex->id().minorId(), (unsigned)10);
    EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)0);
    EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);
    vertex_id++;
  }
  EXPECT_EQ(vertex_id, (unsigned)10);
}

TEST_F(GraphTest, MultiRunVertexAddition) {
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

  // make sure vertices in run 0 and 1 exsist, but do not in run 2.
  for (uint64_t idx = 0; idx < 10; ++idx) {
    auto vertex0 = graph_->at(RCVertex::IdType(0, idx));
    auto vertex1 = graph_->at(RCVertex::IdType(1, idx));
    EXPECT_THROW(graph_->at(RCVertex::IdType(2, idx)), std::range_error);
    EXPECT_TRUE(vertex0 != nullptr);
    EXPECT_TRUE(vertex1 != nullptr);
  }
}

TEST_F(GraphTest, AddEdgeEmptyGraph) {
  EXPECT_THROW(graph_->addEdge(RCVertex::IdType(0, 0), RCVertex::IdType(0, 1)),
               std::range_error);
}

class EdgeTest : public ::testing::Test {
 public:
  EdgeTest() : graph_{new Graph<RCVertex, RCEdge, RCRun>()} {
    // add a run with 10 vertices.
    graph_->addRun();
    for (int idx = 0; idx < 10; ++idx) graph_->addVertex();
    // add a run with 10 vertices.
    graph_->addRun();
    for (int idx = 0; idx < 10; ++idx) graph_->addVertex();
  }
  void SetUp() override {}
  void TearDown() override {}
  ~EdgeTest() override {}

 protected:
  Graph<RCVertex, RCEdge, RCRun>::Ptr graph_;
};

TEST_F(EdgeTest, BFSDFSTest) {
  for (int idx = 0; idx < 9; ++idx)
    graph_->addEdge(RCVertex::IdType(0, idx), RCVertex::IdType(0, idx + 1));

  auto itr = graph_->beginBfs(RCVertex::IdType(0, 9));
  for (; itr != graph_->end(); itr++) {
  }

  itr = graph_->beginDfs(RCVertex::IdType(0, 9));
  for (; itr != graph_->end(); itr++) {
  }
}

TEST_F(EdgeTest, AddTemporalEdge) {
  // make an edge from (0,0)--(0,1)
  RCVertex::IdType from(0, 0);
  RCVertex::IdType to(0, 1);

  auto edge = graph_->addEdge(from, to);
  EXPECT_TRUE(edge != nullptr);
  EXPECT_TRUE(edge->isTemporal());
  EXPECT_FALSE(edge->isSpatial());
  EXPECT_FALSE(edge->isManual());
  EXPECT_TRUE(edge->isAutonomous());

  edge->setManual();
  EXPECT_TRUE(edge->isManual());
  EXPECT_FALSE(edge->isAutonomous());

  auto incident_vertices = edge->incident();
  EXPECT_EQ(incident_vertices.first, from);
  EXPECT_EQ(incident_vertices.second, to);

  auto edge_map = graph_->edges();
  EXPECT_EQ(edge_map->size(), (unsigned)1);

  RCEdge::IdType edge_id(from, to, Temporal);
  RCEdge::IdType edge_id_spatial(from, to, Spatial);

  // Check the vertices edge list.
  auto vertex = graph_->at(from);
  EXPECT_EQ(vertex->temporalEdges().size(), (unsigned)1);
  EXPECT_EQ(vertex->spatialEdges().size(), (unsigned)0);

  // Extract the edge based off of the vertex ids
  EXPECT_NO_THROW(graph_->at(*vertex->temporalEdges().rbegin()));
  EXPECT_NO_THROW(graph_->at(edge_id));
  EXPECT_THROW(graph_->at(edge_id_spatial), std::range_error);
}

TEST_F(EdgeTest, AddSpatialEdge) {
  // make an edge from (0,0)--(0,1)
  RCVertex::IdType from(0, 0);
  RCVertex::IdType to(0, 1);

  auto edge = graph_->addEdge(from, to, Spatial);
  EXPECT_TRUE(edge != nullptr);
  EXPECT_FALSE(edge->isTemporal());
  EXPECT_TRUE(edge->isSpatial());
  EXPECT_FALSE(edge->isManual());
  EXPECT_TRUE(edge->isAutonomous());

  auto incident_vertices = edge->incident();
  EXPECT_EQ(incident_vertices.first, from);
  EXPECT_EQ(incident_vertices.second, to);

  auto edge_map = graph_->edges();
  EXPECT_EQ(edge_map->size(), (unsigned)1);
}

TEST_F(EdgeTest, CrossRunEdge) {
  // make an edge from (0,5)--(1,6)
  RCVertex::IdType to(0, 5);
  RCVertex::IdType from(1, 6);

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

  auto edge_map = graph_->edges();
  EXPECT_EQ(edge_map->size(), (unsigned)1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
