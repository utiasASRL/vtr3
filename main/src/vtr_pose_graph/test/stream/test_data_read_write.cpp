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
 * \file data_read_write_tests.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <filesystem>
#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/serializable/rc_graph.hpp>

#include <std_msgs/msg/float64.hpp>

namespace fs = std::filesystem;
using namespace vtr;
using namespace vtr::logging;
using namespace vtr::pose_graph;

using TestMsg = std_msgs::msg::Float64;

constexpr int VERTICES = 10;

/* Create the following graph:
 *   R0: 0 --- 1 --- 2 --- ... --- 9
 * Store some dummy data in each vertex and then load the data back from
 * vertices.
 */
TEST(PoseGraph, ReadWrite) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.

  // Initialize pose graph
  auto graph = std::make_shared<RCGraph>(working_dir.string(), false);

  // Add the first run
  graph->addRun();

  // Register a data read&write stream named test_data.
  std::string stream_name = "test_data";

  // Add the first vertex
  Timestamp stamp = 0;
  graph->addVertex(stamp++);
  for (int idx = 1; idx < VERTICES; ++idx) {
    graph->addVertex(stamp++);
    graph->addEdge(RCVertex::IdType(0, idx - 1), RCVertex::IdType(0, idx),
                   Temporal);
  }

  // Generate random data
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  // cache data for comparison
  std::vector<TestMsg> data_vec;

  // Insert some dummy messages to each vertex
  stamp = 0;
  for (int vertex_idx = 0; vertex_idx < VERTICES; ++vertex_idx) {
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    EXPECT_TRUE(vertex != nullptr);
    TestMsg test_msg;
    auto data = std::make_shared<TestMsg>();
    data->data = distribution(generator);
    auto message =
        std::make_shared<storage::LockableMessage<TestMsg>>(data, stamp++);
    LOG(INFO) << "Store " << data->data << " into vertex " << vertex_id;
    vertex->insert<TestMsg>(stream_name, message);

    data_vec.push_back(*data);
  }

  // save all data to disk
  graph.reset();

  // load data
  graph = std::make_shared<RCGraph>(working_dir.string());

  // Now load all the data back from disk.
  LOG(INFO) << "Loading data from disk";
  for (int vertex_idx = 9; vertex_idx >= 0; --vertex_idx) {
    // access the vertex
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    auto message = vertex->retrieve<TestMsg>(stream_name);
    auto data = message->unlocked().get().getData();
    LOG(INFO) << "Vertex " << vertex_id << " has value " << data.data;
    EXPECT_EQ(data_vec[vertex_idx].data, data.data);
    data.data = distribution(generator);
    message->unlocked().get().setData(data);
    LOG(INFO) << "Store " << data.data << " into vertex " << vertex_id;
    data_vec[vertex_idx] = data;
  }

  // save all data to disk
  graph.reset();

  // load data
  graph = std::make_shared<RCGraph>(working_dir.string());

  // Now load all the data back from disk.
  LOG(INFO) << "Loading data from disk";
  for (int vertex_idx = 9; vertex_idx >= 0; --vertex_idx) {
    // access the vertex
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    auto message = vertex->retrieve<TestMsg>(stream_name);
    auto data = message->unlocked().get().getData();
    EXPECT_EQ(data_vec[vertex_idx].data, data.data);
    LOG(INFO) << "Vertex " << vertex_id << " has value " << data.data;
  }

  // save all data to disk
  graph.reset();

  // Cleanup
  fs::remove_all(working_dir);
}

int main(int argc, char** argv) {
  configureLogging("", false);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
