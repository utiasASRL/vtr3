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
 * \file graph_read_write_tests.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/graph_vertex.hpp>
#include <vtr_messages/msg/sensor_test.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

/// #include <robochunk/base/BaseChunkSerializer.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;

class GraphReadWriteTest : public ::testing::Test {
 public:
  /* Create the following graph
   * R0: 0 --- 1 --- 2
   *       \
   *        \
   *         \
   * R1: 0 --- 1 --- 2
   *                 |
   * R2: 0 --- 1 --- 2
   *           |
   * R3: 0 --- 1 --- 2
   *                 |
   * R4: 0 --- 1 --- 2
   */

  GraphReadWriteTest()
      : working_dir_(fs::temp_directory_path() / "vtr_pose_graph_test"),
        graph_index_file_("graph_index"),
        robot_id_(666) {}

  ~GraphReadWriteTest() override {}

  void SetUp() override {
    graph_.reset(new RCGraph(working_dir_ / graph_index_file_, 0));

    std::vector<std::string> stream_names;
    stream_names.push_back("test_data1");
    stream_names.push_back("test_data2");

    // add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      auto run_id = graph_->addRun(robot_id_);
      graph_->registerVertexStream<vtr_messages::msg::SensorTest>(
          run_id, stream_names[0]);
      graph_->registerVertexStream<vtr_messages::msg::SensorTest>(
          run_id, stream_names[1]);
      time_stamp_.nanoseconds_since_epoch++;
      graph_->addVertex(time_stamp_);
      time_stamp_.nanoseconds_since_epoch++;
      graph_->addVertex(time_stamp_);
      time_stamp_.nanoseconds_since_epoch++;
      graph_->addVertex(time_stamp_);
      graph_->addEdge(RCVertex::IdType(idx, 0), RCVertex::IdType(idx, 1));
      graph_->addEdge(RCVertex::IdType(idx, 1), RCVertex::IdType(idx, 2));
    }
    graph_->addEdge(RCVertex::IdType(1, 1), RCVertex::IdType(0, 0), Spatial);
    graph_->addEdge(RCVertex::IdType(2, 2), RCVertex::IdType(1, 2), Spatial);
    graph_->addEdge(RCVertex::IdType(3, 1), RCVertex::IdType(2, 1), Spatial);
    graph_->addEdge(RCVertex::IdType(4, 2), RCVertex::IdType(3, 2), Spatial);

    // Set the index data.
    uint64_t time = 0;
    uint64_t data_idx1 = 0;
    uint64_t data_idx2 = 0;
    for (int run_idx = 0; run_idx < 5; ++run_idx) {
      time = run_idx * 10;
      data_idx1 = 0;
      data_idx2 = 0;
      for (int vertex_idx = 0; vertex_idx < 3; ++vertex_idx) {
        // access the vertex
        RCVertex::IdType vertex_id(run_idx, vertex_idx);
        auto vertex = graph_->at(vertex_id);
        ASSERT_TRUE(vertex != nullptr);
        auto time2 = time + 1;
        vertex->setTimeRange(time, time2);
        vertex->addStreamIndices<vtr_messages::msg::SensorTest>(
            stream_names[0], {data_idx1, data_idx1 + 100}, true);
        vertex->addStreamIndices<vtr_messages::msg::SensorTest>(
            stream_names[1], {data_idx2, data_idx2 + 250}, true);

        // increase the indices.
        time++;
        data_idx1 += 100;
        data_idx2 += 250;
      }
    }

    // set the edge's transform to something special;
    auto edge_map = graph_->edges();
    for (auto itr = edge_map->begin(); itr != edge_map->end(); ++itr) {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(0, 3) = itr->second->id().majorId();
      transform(1, 3) = itr->second->id().minorId();
      transform(2, 3) = itr->second->id().type();
      lgmath::se3::TransformationWithCovariance edge_transform(transform);
      Eigen::Matrix<double, 6, 6> cov =
          Eigen::Matrix<double, 6, 6>::Identity() * 100;
      edge_transform.setCovariance(cov);
      itr->second->setTransform(edge_transform);
    }
  }

  void TearDown() override { fs::remove_all(working_dir_); }

 protected:
  fs::path working_dir_;
  fs::path graph_index_file_;
  std::unique_ptr<RCGraph> graph_;
  int robot_id_;
  vtr_messages::msg::TimeStamp time_stamp_;
};

void verifyVertex(RCGraph *graph, uint32_t run_idx, uint32_t vertex_idx) {
  RCVertex::IdType vertex_id(run_idx, vertex_idx);
  auto vertex = graph->at(vertex_id);
  ASSERT_TRUE(vertex != nullptr);

  // send to ros message, make sure it looks good.
  auto vertex_msg = vertex->toRosMsg();
  EXPECT_EQ(vertex_msg.id, vertex_idx);
  EXPECT_EQ(vertex_msg.stream_time.idx1, run_idx * 10 + vertex_idx);
  EXPECT_EQ(vertex_msg.stream_time.idx2, run_idx * 10 + vertex_idx + 1);
  EXPECT_EQ(vertex_msg.stream_idx.size(), (unsigned)2);
#if 0
  EXPECT_EQ(vertex_msg.point_idx.size(), 0);
#endif

  auto interval = vertex_msg.stream_idx[0];
  EXPECT_EQ(interval.name_idx, (unsigned)0);
  EXPECT_EQ(interval.idx1, vertex_idx * 100);
  EXPECT_EQ(interval.idx2, vertex_idx * 100 + 100);

  interval = vertex_msg.stream_idx[1];
  EXPECT_EQ(interval.name_idx, (unsigned)1);
  EXPECT_EQ(interval.idx1, vertex_idx * 250);
  EXPECT_EQ(interval.idx2, vertex_idx * 250 + 250);
}

void verifyEdges(RCGraph *graph) {
  // Check temporal edges
  std::vector<RCEdge::IdType> edge_ids;

  for (int run_idx = 0; run_idx < 5; ++run_idx) {
    for (int vertex_idx = 0; vertex_idx < 2; ++vertex_idx) {
      RCVertex::IdType from(run_idx, vertex_idx);
      RCVertex::IdType to(run_idx, vertex_idx + 1);
      edge_ids.push_back(
          RCEdge::IdType(RCVertex::IdType(run_idx, vertex_idx),
                         RCVertex::IdType(run_idx, vertex_idx + 1), Temporal));
    }
  }

  edge_ids.push_back(
      RCEdge::IdType(RCVertex::IdType(1, 1), RCVertex::IdType(0, 0), Spatial));
  edge_ids.push_back(
      RCEdge::IdType(RCVertex::IdType(2, 2), RCVertex::IdType(1, 2), Spatial));
  edge_ids.push_back(
      RCEdge::IdType(RCVertex::IdType(3, 1), RCVertex::IdType(2, 1), Spatial));
  edge_ids.push_back(
      RCEdge::IdType(RCVertex::IdType(4, 2), RCVertex::IdType(3, 2), Spatial));

  for (uint32_t idx = 0; idx < edge_ids.size(); ++idx) {
    auto &edge_id = edge_ids[idx];
    auto edge = graph->at(edge_id);

    // check the transform
    auto edge_transform = edge->T().matrix();
    EXPECT_EQ(edge_transform.rows(), 4);
    EXPECT_EQ(edge_transform.cols(), 4);
    EXPECT_EQ(edge_transform(0, 3), edge->id().majorId());
    EXPECT_EQ(edge_transform(1, 3), edge->id().minorId());
    EXPECT_EQ(edge_transform(2, 3), edge->id().type());

    EXPECT_TRUE(edge->T().covarianceSet());
    for (int idx = 0; idx < 6; ++idx) {
      EXPECT_EQ(edge->T().cov()(idx, idx), 100);
    }
  }
}

TEST_F(GraphReadWriteTest, VertexSerialization) {
  // vtr_messages::msg::GraphVertex vertex_msg;
  RCVertex vertex;
  auto vertex_msg = vertex.toRosMsg();
  EXPECT_EQ(vertex_msg.id, (unsigned)0);
  EXPECT_EQ(vertex_msg.stream_time.idx1, (unsigned)0);
  EXPECT_EQ(vertex_msg.stream_time.idx2, (unsigned)0);
  EXPECT_EQ(vertex_msg.stream_idx.size(), (unsigned)0);
#if 0
  EXPECT_EQ(vertex_msg.point_idx.size(), 0);
#endif
}

TEST_F(GraphReadWriteTest, MessageVertexMessage) {
  // Set up a vertex message.
  vtr_messages::msg::GraphVertex vertex_msg;
  vertex_msg.id = 1;
  vertex_msg.stream_time.idx1 = 123456;
  vertex_msg.stream_time.idx2 = 123457;

  vtr_messages::msg::UtilIntervalNamed stream_idx;
  stream_idx.name_idx = 0;
  stream_idx.idx1 = 5000;
  stream_idx.idx2 = 5100;
  vertex_msg.stream_idx.push_back(stream_idx);
  stream_idx.name_idx = 1;
  stream_idx.idx1 = 200;
  stream_idx.idx2 = 205;
  vertex_msg.stream_idx.push_back(stream_idx);

  /// RCStreamInterface::LockableFieldMapPtr field_map;
  /// RCStreamInterface::LockableStreamMapPtr stream_map;
  /// field_map.reset(new RCStreamInterface::LockableFieldMap());
  /// stream_map.reset(new RCStreamInterface::LockableStreamMap());
  auto field_map = std::make_shared<RCStreamInterface::LockableFieldMap>();
  auto stream_map =
      std::make_shared<RCStreamInterface::LockableDataStreamMap>();
  field_map->locked().get().insert({"test_data1", 0});
  field_map->locked().get().insert({"test_data2", 1});
  stream_map->locked().get()[0];
  stream_map->locked().get()[1];
  // Initialize the vertex with the message data.
  /// REQUIRE_NOTHROW(
  ///     vertex.reset(new RCVertex(vertex_msg, 0, field_map, stream_map)));
  /// std::unique_ptr<RCVertex> vertex;
  auto vertex = RCVertex::MakeShared(vertex_msg, 0, field_map, stream_map);

  // Take the message out of the vertex and compare.
  auto vertex_msg2 = vertex->toRosMsg();

  // Make sure the messages content is equivalent
  EXPECT_EQ(vertex_msg.id, vertex_msg2.id);
  EXPECT_EQ(vertex_msg.stream_time.idx1, vertex_msg2.stream_time.idx1);
  EXPECT_EQ(vertex_msg.stream_time.idx2, vertex_msg2.stream_time.idx2);
  for (unsigned idx = 0; idx < vertex_msg.stream_idx.size(); ++idx) {
    auto interval1 = vertex_msg.stream_idx[idx];
    auto interval2 = vertex_msg2.stream_idx[idx];

    EXPECT_EQ(interval1.name_idx, interval2.name_idx);
    EXPECT_EQ(interval1.idx1, interval2.idx1);
    EXPECT_EQ(interval1.idx2, interval2.idx2);
  }
}

#if false
TEST_F(GraphReadWriteTest, AddIndicesToVertex) {
  // add some stream indices, time interval
  RCVertex vertex;
  uint64_t time0 = 56789;
  uint64_t time1 = 56889;
  std::string stream0_name("test_data1");
  std::string stream1_name("test_data2");
  vertex.setTimeRange(time0, time1);
  vertex.addStreamIndices<vtr_messages::msg::SensorTest>(stream0_name,
                                                         {500, 501});
  vertex.addStreamIndices<vtr_messages::msg::SensorTest>(stream1_name,
                                                         {200, 205});

  // send to protobuf, make sure it looks good.
  auto vertex_msg = vertex.toRosMsg();
  EXPECT_EQ(vertex_msg.id, (unsigned)0);
  EXPECT_EQ(vertex_msg.stream_time.idx1, time0);
  EXPECT_EQ(vertex_msg.stream_time.idx2, time1);
  EXPECT_EQ(vertex_msg.stream_idx.size(), (unsigned)2);
#if 0
  EXPECT_EQ(vertex_msg.point_idx.size(), 0);
#endif

  auto interval = vertex_msg.stream_idx[0];
  EXPECT_EQ(interval.name_idx, (unsigned)0);
  EXPECT_EQ(interval.idx1, (unsigned)500);
  EXPECT_EQ(interval.idx2, (unsigned)501);

  interval = vertex_msg.stream_idx[1];
  EXPECT_EQ(interval.name_idx, (unsigned)1);
  EXPECT_EQ(interval.idx1, (unsigned)200);
  EXPECT_EQ(interval.idx2, (unsigned)205);
}
#endif

TEST_F(GraphReadWriteTest, TestOriginalGraph) {
  for (auto run : graph_->runs()) {
    ASSERT_EQ(run.second->robotId(), static_cast<uint32_t>(robot_id_));
  }

  for (int run_idx = 0; run_idx < 5; run_idx++)
    for (int vertex_idx = 0; vertex_idx < 3; vertex_idx++)
      verifyVertex(graph_.get(), run_idx, vertex_idx);
  verifyEdges(graph_.get());
  // check the top level maps.
  auto edges = graph_->edges();
  auto vertices = graph_->vertices();

  EXPECT_EQ(edges->size(), (unsigned)14);
  EXPECT_EQ(vertices->size(), (unsigned)15);
}

TEST_F(GraphReadWriteTest, GraphSerialization) {
  // save graph to file
  graph_->save(true);

  RCGraph::Ptr graph{new RCGraph{working_dir_ / graph_index_file_}};

  graph->load();

  for (auto run : graph->runs()) {
    EXPECT_EQ(run.second->robotId(), static_cast<uint32_t>(robot_id_));
  }

  for (int run_idx = 0; run_idx < 5; run_idx++)
    for (int vertex_idx = 0; vertex_idx < 3; vertex_idx++)
      verifyVertex(graph.get(), run_idx, vertex_idx);
  verifyEdges(graph.get());
  // check the top level maps.
  auto edges = graph->edges();
  auto vertices = graph->vertices();

  EXPECT_EQ(edges->size(), (unsigned)14);
  EXPECT_EQ(vertices->size(), (unsigned)15);
}

TEST_F(GraphReadWriteTest, PostLoadSubGraphExtraction) {
  // save graph to file
  graph_->save(true);

  // load the graph again
  RCGraph::Ptr graph{new RCGraph{working_dir_ / graph_index_file_}};
  graph->load();

  RCVertex::IdType root_id(0, 0);
  EXPECT_NO_THROW(
      graph_->getSubgraph(root_id, eval::Mask::Const::MakeShared()));
  EXPECT_NO_THROW(graph->getSubgraph(root_id, eval::Mask::Const::MakeShared()));

  int count = 0;
  for (auto it = graph->beginVertex(); it != graph->endVertex(); ++it) count++;

  EXPECT_EQ(count, 15);
}

TEST_F(GraphReadWriteTest, SaveLoadSaveLoad) {
  // save graph to file
  graph_->save(true);

  // load, modify and then save the graph
  {
    RCGraph::Ptr graph{new RCGraph{working_dir_ / graph_index_file_}};

    graph->load();
    graph->addRun(robot_id_);
    time_stamp_.nanoseconds_since_epoch++;
    graph->addVertex(time_stamp_);
    time_stamp_.nanoseconds_since_epoch++;
    graph->addVertex(time_stamp_);
    time_stamp_.nanoseconds_since_epoch++;
    graph->addVertex(time_stamp_);
    graph->addEdge(RCVertex::IdType(5, 2), RCVertex::IdType(0, 2), Spatial);
    graph->save(true);
  }

  // load the graph again
  RCGraph::Ptr graph{new RCGraph{working_dir_ / graph_index_file_}};
  graph->load();

  EXPECT_EQ(graph->vertices()->size(), (unsigned)18);
  EXPECT_NO_THROW(graph->at(
      RCEdge::IdType(RCVertex::IdType(5, 2), RCVertex::IdType(0, 2), Spatial)));
}

int main(int argc, char **argv) {
  vtr::logging::configureLogging();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
