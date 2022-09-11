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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include "rcpputils/filesystem_helper.hpp"

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"

#include "std_msgs/msg/string.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;

using StringMsg = std_msgs::msg::String;

EdgeTransform trivialTransform(const VertexId& from, const VertexId& to) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  mat(0, 3) = from.majorId();
  mat(1, 3) = from.minorId();
  mat(2, 3) = to.minorId();
  EdgeTransform tf(mat);
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 3;
  tf.setCovariance(cov);
  return tf;
}

void verifyTransform(const VertexId& from, const VertexId& to,
                     const EdgeTransform& tf) {
  const auto mat = tf.matrix();
  EXPECT_EQ(mat(0, 3), from.majorId());
  EXPECT_EQ(mat(1, 3), from.minorId());
  EXPECT_EQ(mat(2, 3), to.minorId());
  EXPECT_TRUE(tf.covarianceSet());
  for (int idx = 0; idx < 6; ++idx) {
    EXPECT_EQ(tf.cov()(idx, idx), 3);
  }
}

class TemporaryDirectoryFixture : public Test {
 public:
  TemporaryDirectoryFixture() {
    temp_dir_ = rcpputils::fs::create_temp_directory("tmp_test_dir").string();
    // temp_dir_ = "/home/yuchen/ASRL/temp/test_pose_graph";
    // rcpputils::fs::create_directories(temp_dir_);
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
  map_info.set = true;
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
    // clang-format off
    for (int idx = 0; idx < 5; ++idx) {
      graph_->addRun();
      graph_->addVertex(time_stamp_++);
      graph_->addVertex(time_stamp_++);
      graph_->addVertex(time_stamp_++);
      graph_->addEdge(VertexId(idx, 0), VertexId(idx, 1), EdgeType::Temporal, false, trivialTransform(VertexId(idx, 0), VertexId(idx, 1)));
      graph_->addEdge(VertexId(idx, 1), VertexId(idx, 2), EdgeType::Temporal, false, trivialTransform(VertexId(idx, 1), VertexId(idx, 2)));
    }
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), EdgeType::Spatial, false, trivialTransform(VertexId(1, 1), VertexId(0, 0)));
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), EdgeType::Spatial, false, trivialTransform(VertexId(2, 2), VertexId(1, 2)));
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), EdgeType::Spatial, false, trivialTransform(VertexId(3, 1), VertexId(2, 1)));
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), EdgeType::Spatial, false, trivialTransform(VertexId(4, 2), VertexId(3, 2)));
    // clang-format on
  }

 public:
  std::shared_ptr<RCGraph> graph_;
  Timestamp time_stamp_ = 0;
};

void verifyGraphStructure(RCGraph& graph) {
  for (int run_idx = 0; run_idx < 5; run_idx++)
    for (int vertex_idx = 0; vertex_idx < 3; vertex_idx++) {
      EXPECT_NO_THROW(graph.at(VertexId(run_idx, vertex_idx)));
    }

  RCEdge::Ptr edge;
  for (int idx = 0; idx < 5; ++idx) {
    edge = graph.at(EdgeId(VertexId(idx, 0), VertexId(idx, 1)));
    verifyTransform(VertexId(idx, 0), VertexId(idx, 1), edge->T());
    edge = graph.at(EdgeId(VertexId(idx, 1), VertexId(idx, 2)));
    verifyTransform(VertexId(idx, 1), VertexId(idx, 2), edge->T());
  }
  edge = graph.at(EdgeId(VertexId(1, 1), VertexId(0, 0)));
  verifyTransform(VertexId(1, 1), VertexId(0, 0), edge->T());
  edge = graph.at(EdgeId(VertexId(2, 2), VertexId(1, 2)));
  verifyTransform(VertexId(2, 2), VertexId(1, 2), edge->T());
  edge = graph.at(EdgeId(VertexId(3, 1), VertexId(2, 1)));
  verifyTransform(VertexId(3, 1), VertexId(2, 1), edge->T());
  edge = graph.at(EdgeId(VertexId(4, 2), VertexId(3, 2)));
  verifyTransform(VertexId(4, 2), VertexId(3, 2), edge->T());

  EXPECT_EQ(graph.numberOfEdges(), (unsigned)14);
  EXPECT_EQ(graph.numberOfVertices(), (unsigned)15);
}

TEST_F(GraphSerializationFixture, SaveLoadSaveLoad) {
  verifyGraphStructure(*graph_);

  // save graph to file
  graph_.reset();

  // load again
  auto graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(*graph);

  // save again
  graph.reset();

  // load again
  graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(*graph);
}

TEST_F(GraphSerializationFixture, PostLoadSubGraphExtraction) {
  // save graph to file
  graph_.reset();

  // load again
  auto graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(*graph);

  VertexId root_id(0, 0);
  EXPECT_NO_THROW(graph->getSubgraph(
      root_id, std::make_shared<eval::mask::ConstEval>(true, true)));

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
  graph->addEdge(VertexId(5, 2), VertexId(0, 2), EdgeType::Spatial, false,
                 EdgeTransform(true));
  graph.reset();

  // load the graph again
  graph = std::make_shared<RCGraph>(graph_dir_);
  EXPECT_EQ(graph->numberOfVertices(), (unsigned)18);
  EXPECT_NO_THROW(graph->at(EdgeId(VertexId(5, 2), VertexId(0, 2))));
  graph.reset();

  // load the graph again
  graph = std::make_shared<RCGraph>(graph_dir_);
  EXPECT_EQ(graph->numberOfVertices(), (unsigned)18);
  EXPECT_NO_THROW(graph->at(EdgeId(VertexId(5, 2), VertexId(0, 2))));
  graph.reset();
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
