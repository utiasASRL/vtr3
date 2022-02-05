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
 * \file test_vertex_stream_time.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include <filesystem>
#include <random>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"

#include "std_msgs/msg/float64.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;

namespace fs = std::filesystem;
using TestMsg = std_msgs::msg::Float64;

TEST(TestSerializationVertex, construct_vertex_directly) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.

  // Register a data read&write stream named test_data.
  std::string stream_name = "test_data";

  // Initialize pose graph
  auto graph = std::make_shared<RCGraph>(working_dir.string(), false);
  graph->addRun();
  auto vertex = graph->addVertex(666);  // add a vertex with keyframetime = 666;
  ASSERT_TRUE(vertex != nullptr);

  // Generate random data
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  // cache data for comparison
  std::vector<TestMsg> data_vec;

  // insert some data to the vertex
  for (Timestamp time = 500; time < 600; time += 10) {
    auto data = std::make_shared<TestMsg>();
    data->data = distribution(generator);
    auto message = std::make_shared<LockableMessage<TestMsg>>(data, time);
    CLOG(INFO, "test") << "Store " << data->data << " with time stamp " << time;
    vertex->insert<TestMsg>(stream_name, "std_msgs/msg/Float64", message);
    data_vec.push_back(*data);
  }

  // check vertex range
  auto time_range = vertex->timeRange();
  EXPECT_EQ(time_range.first, 500);
  EXPECT_EQ(time_range.second, 666);

  // load all the data back from disk.
  CLOG(INFO, "test") << "Retrieving data (not yet saved to disk)";
  {
    auto time_range = vertex->timeRange();
    auto data_vec_loaded =
        vertex->retrieve<TestMsg>(stream_name, "std_msgs/msg/Float64",
                                  time_range.first, time_range.second);

    size_t i = 0;
    for (auto message : data_vec_loaded) {
      auto time = message->unlocked().get().getTimestamp();
      auto data = message->unlocked().get().getData();
      CLOG(INFO, "test") << "Time stamp " << time << " has value " << data.data;
      EXPECT_EQ(data_vec[i].data, data.data);
      // make some modification to the data
      data.data++;
      message->unlocked().get().setData(data);
      data_vec[i].data++;
      i++;
    }
  }

  // discard the vertex shared ptr
  auto vid = vertex->id();
  vertex.reset();

  // save all data to disk
  graph.reset();

  // load data
  graph = std::make_shared<RCGraph>(working_dir.string());

  // get the same vertex
  vertex = graph->at(vid);

  // load all the data back from disk.
  CLOG(INFO, "test") << "Loading data from disk";
  {
    auto time_range = vertex->timeRange();
    auto data_vec_loaded =
        vertex->retrieve<TestMsg>(stream_name, "std_msgs/msg/Float64",
                                  time_range.first, time_range.second);

    size_t i = 0;
    for (auto message : data_vec_loaded) {
      auto time = message->unlocked().get().getTimestamp();
      auto data = message->unlocked().get().getData();
      CLOG(INFO, "test") << "Time stamp " << time << " has value " << data.data;
      EXPECT_EQ(data_vec[i].data, data.data);
      // make some modification to the data
      data.data++;
      message->unlocked().get().setData(data);
      data_vec[i].data++;
      i++;
    }
  }

  // try retrieve data again should generate warnings
  /// \note when retrieving data from a range we always have to look for them
  /// from disk. There's no way to cache them because we won't know if the
  /// current cached data from [start, stop] contain all data within this range.
  /// To solve this, we somehow need to know all timestamps of data on disk
  /// within this range, resulting in a lot more information to cache for every
  /// vertex - not worth it. For now just make sure we do not accidentally
  /// overwrite cached data, sine they may have been modified.
  {
    auto data_vec_loaded =
        vertex->retrieve<TestMsg>(stream_name, "std_msgs/msg/Float64",
                                  time_range.first, time_range.second);

    size_t i = 0;
    for (auto message : data_vec_loaded) {
      auto time = message->unlocked().get().getTimestamp();
      auto data = message->unlocked().get().getData();
      CLOG(INFO, "test") << "Time stamp " << time << " has value " << data.data;
      EXPECT_EQ(data_vec[i++].data, data.data);
    }
  }

  // save all data to disk
  graph.reset();

  // load data
  graph = std::make_shared<RCGraph>(working_dir.string());

  // get the same vertex
  vertex = graph->at(vid);

  // Now load all the data back from disk.
  CLOG(INFO, "test") << "Loading data from disk again";

  auto data_vec_loaded = vertex->retrieve<TestMsg>(
      stream_name, "std_msgs/msg/Float64", time_range.first, time_range.second);

  size_t i = 0;
  for (auto message : data_vec_loaded) {
    auto time = message->unlocked().get().getTimestamp();
    auto data = message->unlocked().get().getData();
    CLOG(INFO, "test") << "Time stamp " << time << " has value " << data.data;
    EXPECT_EQ(data_vec[i++].data, data.data);
  }
}

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
