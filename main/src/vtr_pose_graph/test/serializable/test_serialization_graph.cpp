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
 * \file test_serialization_graph.cpp
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
    graph_dir_ = (rcpputils::fs::path(temp_dir_) / "graph").string();
  }

  ~TemporaryDirectoryFixture() override {
    rcpputils::fs::remove_all(rcpputils::fs::path(temp_dir_));
  }

  std::string temp_dir_;
  std::string graph_dir_;
};

TEST_F(TemporaryDirectoryFixture, construct_empty_graph_and_save) {
  RCGraph graph(graph_dir_, false);
}

TEST_F(TemporaryDirectoryFixture, simulate_loading_graph_from_disk) {
  auto graph = std::make_shared<RCGraph>(graph_dir_, false);

  RCGraph::MapInfoMsg map_info;
  map_info.set = true;
  graph->setMapInfo(map_info);

  graph.reset();

  graph = std::make_shared<RCGraph>(graph_dir_);
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
    for (int i = 0; i < 5; ++i) {
      graph_->addRun();
      vids_[i][0] = graph_->addVertex(time_stamp_++)->id();
      vids_[i][1] = graph_->addVertex(time_stamp_++)->id();
      vids_[i][2] = graph_->addVertex(time_stamp_++)->id();
      graph_->addEdge(vids_[i][0], vids_[i][1], EdgeType::Temporal, EdgeMode::Autonomous, trivialTransform(vids_[i][0], vids_[i][1]));
      graph_->addEdge(vids_[i][1], vids_[i][2], EdgeType::Temporal, EdgeMode::Autonomous, trivialTransform(vids_[i][1], vids_[i][2]));
    }
    graph_->addEdge(vids_[1][1], vids_[0][0], EdgeType::Spatial, EdgeMode::Autonomous, trivialTransform(vids_[1][1], vids_[0][0]));
    graph_->addEdge(vids_[2][2], vids_[1][2], EdgeType::Spatial, EdgeMode::Autonomous, trivialTransform(vids_[2][2], vids_[1][2]));
    graph_->addEdge(vids_[3][1], vids_[2][1], EdgeType::Spatial, EdgeMode::Autonomous, trivialTransform(vids_[3][1], vids_[2][1]));
    graph_->addEdge(vids_[4][2], vids_[3][2], EdgeType::Spatial, EdgeMode::Autonomous, trivialTransform(vids_[4][2], vids_[3][2]));
    // clang-format on
  }

  void verifyGraphStructure(RCGraph& graph) {
    for (int i = 0; i < 5; ++i)
      for (int j = 0; j < 3; ++j)
        EXPECT_NO_THROW(graph.at(vids_[i][j]));

    RCEdge::Ptr edge;
    for (int i = 0; i < 5; ++i) {
      edge = graph.at(EdgeId(vids_[i][0], vids_[i][1]));
      verifyTransform(vids_[i][0], vids_[i][1], edge->T());
      edge = graph.at(EdgeId(vids_[i][1], vids_[i][2]));
      verifyTransform(vids_[i][1], vids_[i][2], edge->T());
    }
    edge = graph.at(EdgeId(vids_[1][1], vids_[0][0]));
    verifyTransform(vids_[1][1], vids_[0][0], edge->T());
    edge = graph.at(EdgeId(vids_[2][2], vids_[1][2]));
    verifyTransform(vids_[2][2], vids_[1][2], edge->T());
    edge = graph.at(EdgeId(vids_[3][1], vids_[2][1]));
    verifyTransform(vids_[3][1], vids_[2][1], edge->T());
    edge = graph.at(EdgeId(vids_[4][2], vids_[3][2]));
    verifyTransform(vids_[4][2], vids_[3][2], edge->T());

    EXPECT_EQ(graph.numberOfEdges(), (unsigned)14);
    EXPECT_EQ(graph.numberOfVertices(), (unsigned)15);
  }

 public:
  std::shared_ptr<RCGraph> graph_;
  Timestamp time_stamp_ = 0;
  VertexId vids_[5][3];
};

TEST_F(GraphSerializationFixture, SaveLoadSaveLoad) {
  verifyGraphStructure(*graph_);

  graph_.reset();

  auto graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(*graph);

  graph.reset();

  graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(*graph);
}

TEST_F(GraphSerializationFixture, PostLoadSubGraphExtraction) {
  graph_.reset();

  auto graph = std::make_shared<RCGraph>(graph_dir_);
  verifyGraphStructure(*graph);

  EXPECT_NO_THROW(graph->getSubgraph(
      vids_[0][0], std::make_shared<eval::mask::ConstEval>(true, true)));

  int count = 0;
  for (auto it = graph->beginVertex(); it != graph->endVertex(); ++it) count++;
  EXPECT_EQ(count, 15);
}

TEST_F(GraphSerializationFixture, SaveLoadModifySaveLoad) {
  graph_.reset();

  auto graph = std::make_shared<RCGraph>(graph_dir_);
  graph->addRun();
  auto v0 = graph->addVertex(time_stamp_++);
  auto v1 = graph->addVertex(time_stamp_++);
  auto v2 = graph->addVertex(time_stamp_++);
  graph->addEdge(v2->id(), vids_[0][2], EdgeType::Spatial, EdgeMode::Autonomous,
                 EdgeTransform(true));
  graph.reset();

  graph = std::make_shared<RCGraph>(graph_dir_);
  EXPECT_EQ(graph->numberOfVertices(), (unsigned)18);
  EXPECT_NO_THROW(graph->at(EdgeId(v2->id(), vids_[0][2])));
  graph.reset();

  graph = std::make_shared<RCGraph>(graph_dir_);
  EXPECT_EQ(graph->numberOfVertices(), (unsigned)18);
  EXPECT_NO_THROW(graph->at(EdgeId(v2->id(), vids_[0][2])));
  graph.reset();
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
