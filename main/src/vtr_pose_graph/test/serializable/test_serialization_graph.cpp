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
 * \file test_serialization_run.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "rcpputils/filesystem_helper.hpp"

#include <vtr_logging/logging_init.hpp>

#include <vtr_pose_graph/serializable/rc_graph.hpp>

#include "std_msgs/msg/string.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;

using StringMsg = std_msgs::msg::String;

class TemporaryDirectoryFixture : public Test {
 public:
  TemporaryDirectoryFixture() {
    temp_dir_ = rcpputils::fs::create_temp_directory("tmp_test_dir_").string();
    // temp_dir_ = "/home/yuchen/ASRL/temp/test_posegraph";
    // (void)rcpputils::fs::create_directories(temp_dir_);
    graph_dir_ = (rcpputils::fs::path(temp_dir_) / "graph").string();
  }

  ~TemporaryDirectoryFixture() override {
    rcpputils::fs::remove_all(rcpputils::fs::path(temp_dir_));
  }

  std::string temp_dir_;
  std::string graph_dir_;
};

TEST_F(TemporaryDirectoryFixture, construct_empty_graph_and_save) {
  // constructor initializes the ros message
  RCGraph graph(graph_dir_, false);
  // destructor saves the graph
}

TEST_F(TemporaryDirectoryFixture, simulate_loading_graph_from_disk) {
  // constructor initializes the message
  auto graph = std::make_shared<RCGraph>(graph_dir_, false);

  RCGraph::MapInfoMsg map_info;
  graph->setMapInfo(map_info);  // this sets map info (map_info.set=true)

  // destructor saves the graph
  graph.reset();

  // constructor loads graph
  graph = std::make_shared<RCGraph>(graph_dir_);

  // destructor saves the graph
}

class GraphSerializationFixture : public TemporaryDirectoryFixture {
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

  GraphSerializationFixture() {
    graph_ = std::make_shared<RCGraph>(graph_dir_, false);

    // add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      graph_->addRun();
      graph_->addVertex(time_stamp_++);
      graph_->addVertex(time_stamp_++);
      graph_->addVertex(time_stamp_++);
      graph_->addEdge(RCVertex::IdType(idx, 0), RCVertex::IdType(idx, 1),
                      Temporal);
      graph_->addEdge(RCVertex::IdType(idx, 1), RCVertex::IdType(idx, 2),
                      Temporal);
    }
    graph_->addEdge(RCVertex::IdType(1, 1), RCVertex::IdType(0, 0), Spatial);
    graph_->addEdge(RCVertex::IdType(2, 2), RCVertex::IdType(1, 2), Spatial);
    graph_->addEdge(RCVertex::IdType(3, 1), RCVertex::IdType(2, 1), Spatial);
    graph_->addEdge(RCVertex::IdType(4, 2), RCVertex::IdType(3, 2), Spatial);

    // set the edge's transform to something special;
    auto& edge_map = graph_->edges()->unlocked().get();
    for (auto itr = edge_map.begin(); itr != edge_map.end(); ++itr) {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(0, 3) = itr->second->id().majorId2();
      transform(1, 3) = itr->second->id().minorId2();
      transform(2, 3) = itr->second->id().type();
      lgmath::se3::TransformationWithCovariance edge_transform(transform);
      Eigen::Matrix<double, 6, 6> cov =
          Eigen::Matrix<double, 6, 6>::Identity() * 100;
      edge_transform.setCovariance(cov);
      itr->second->setTransform(edge_transform);
    }
  }

 public:
  std::shared_ptr<RCGraph> graph_;
  Timestamp time_stamp_ = 0;
};

void verifyGraphStructure(RCGraph* graph) {
  for (int run_idx = 0; run_idx < 5; run_idx++)
    for (int vertex_idx = 0; vertex_idx < 3; vertex_idx++) {
      RCVertex::IdType vertex_id(run_idx, vertex_idx);
      auto vertex = graph->at(vertex_id);
      EXPECT_TRUE(vertex != nullptr);
    }

  // Check temporal edges
  std::vector<RCEdge::IdType> edgeids;
  for (int run_idx = 0; run_idx < 5; ++run_idx) {
    for (int vertex_idx = 0; vertex_idx < 2; ++vertex_idx) {
      edgeids.emplace_back(RCVertex::IdType(run_idx, vertex_idx),
                           RCVertex::IdType(run_idx, vertex_idx + 1), Temporal);
    }
  }

  edgeids.emplace_back(RCVertex::IdType(1, 1), RCVertex::IdType(0, 0), Spatial);
  edgeids.emplace_back(RCVertex::IdType(2, 2), RCVertex::IdType(1, 2), Spatial);
  edgeids.emplace_back(RCVertex::IdType(3, 1), RCVertex::IdType(2, 1), Spatial);
  edgeids.emplace_back(RCVertex::IdType(4, 2), RCVertex::IdType(3, 2), Spatial);

  for (uint32_t idx = 0; idx < edgeids.size(); ++idx) {
    auto& edge_id = edgeids[idx];
    auto edge = graph->at(edge_id);

    // check the transform
    auto edge_transform = edge->T().matrix();
    EXPECT_EQ(edge_transform.rows(), 4);
    EXPECT_EQ(edge_transform.cols(), 4);
    EXPECT_EQ(edge_transform(0, 3), edge->id().majorId2());
    EXPECT_EQ(edge_transform(1, 3), edge->id().minorId2());
    EXPECT_EQ(edge_transform(2, 3), edge->id().type());

    EXPECT_TRUE(edge->T().covarianceSet());
    for (int idx = 0; idx < 6; ++idx) {
      EXPECT_EQ(edge->T().cov()(idx, idx), 100);
    }
  }

  // check the top level maps.
  auto& edges = graph->edges()->unlocked().get();
  auto& vertices = graph->vertices()->unlocked().get();

  EXPECT_EQ(edges.size(), (unsigned)14);
  EXPECT_EQ(vertices.size(), (unsigned)15);
}

TEST_F(GraphSerializationFixture, SaveLoadSaveLoad) {
  verifyGraphStructure(graph_.get());

  // save graph to file
  graph_.reset();

  // load again
  RCGraph::Ptr graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(graph.get());

  // save again
  graph.reset();

  // load again
  graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(graph.get());
}

TEST_F(GraphSerializationFixture, PostLoadSubGraphExtraction) {
  // save graph to file
  graph_.reset();

  // load again
  auto graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(graph.get());

  RCVertex::IdType root_id(0, 0);
  EXPECT_NO_THROW(
      graph->getSubgraph(root_id, eval::Mask::Const::MakeShared(true, true)));

  int count = 0;
  for (auto it = graph->beginVertex(); it != graph->endVertex(); ++it) count++;
  EXPECT_EQ(count, 15);
}

TEST_F(GraphSerializationFixture, SaveLoadModifySaveLoad) {
  // save graph to file
  graph_.reset();

  // load, modify and then save the graph
  auto graph = std::make_shared<RCGraph>(graph_dir_);
  graph->addRun();
  graph->addVertex(time_stamp_++);
  graph->addVertex(time_stamp_++);
  graph->addVertex(time_stamp_++);
  graph->addEdge(RCVertex::IdType(5, 2), RCVertex::IdType(0, 2), Spatial);
  graph.reset();

  // load the graph again
  graph = std::make_shared<RCGraph>(graph_dir_);
  EXPECT_EQ(graph->vertices()->unlocked().get().size(), (unsigned)18);
  EXPECT_NO_THROW(graph->at(
      RCEdge::IdType(RCVertex::IdType(5, 2), RCVertex::IdType(0, 2), Spatial)));
  graph.reset();

  // load the graph again
  graph = std::make_shared<RCGraph>(graph_dir_);
  EXPECT_EQ(graph->vertices()->unlocked().get().size(), (unsigned)18);
  EXPECT_NO_THROW(graph->at(
      RCEdge::IdType(RCVertex::IdType(5, 2), RCVertex::IdType(0, 2), Spatial)));
  graph.reset();
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
