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
 * \file graph_read_write_performance_tests.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_logging/logging_init.hpp>

#include <vtr_common/timing/stopwatch.hpp>
#include <vtr_messages/msg/graph_vertex.hpp>
#include <vtr_messages/msg/sensor_test.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

/// #include <robochunk/base/BaseChunkSerializer.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;
using namespace vtr::common;

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
 * Store and then make sure everything can be load back.
 */
int main() {
  vtr::logging::configureLogging();

  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.
  fs::path graph_folder{"test_graph"};
  int robot_id{666};

  // Initialize pose graph
  std::unique_ptr<RCGraph> graph{new RCGraph{working_dir / graph_folder, 0}};

  std::vector<std::string> stream_names;
  stream_names.push_back("test_data1");
  stream_names.push_back("test_data2");

  // add a graph with 5 runs and 3 vertices per run.
  vtr_messages::msg::TimeStamp time_stamp;
  for (int idx = 0; idx < 5; ++idx) {
    auto run_id = graph->addRun(robot_id);
    graph->registerVertexStream<vtr_messages::msg::SensorTest>(run_id,
                                                               stream_names[0]);
    graph->registerVertexStream<vtr_messages::msg::SensorTest>(run_id,
                                                               stream_names[1]);
    time_stamp.nanoseconds_since_epoch++;
    graph->addVertex(time_stamp);
    time_stamp.nanoseconds_since_epoch++;
    graph->addVertex(time_stamp);
    time_stamp.nanoseconds_since_epoch++;
    graph->addVertex(time_stamp);
    graph->addEdge(RCVertex::IdType(idx, 0), RCVertex::IdType(idx, 1));
    graph->addEdge(RCVertex::IdType(idx, 1), RCVertex::IdType(idx, 2));
  }
  graph->addEdge(RCVertex::IdType(1, 1), RCVertex::IdType(0, 0), Spatial);
  graph->addEdge(RCVertex::IdType(2, 2), RCVertex::IdType(1, 2), Spatial);
  graph->addEdge(RCVertex::IdType(3, 1), RCVertex::IdType(2, 1), Spatial);
  graph->addEdge(RCVertex::IdType(4, 2), RCVertex::IdType(3, 2), Spatial);

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
      auto vertex = graph->at(vertex_id);
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
  auto edge_map = graph->edges();
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

  timing::Stopwatch stopwatch;

  stopwatch.start();
  graph->save(true);
  stopwatch.stop();
  auto save_time = stopwatch.count<std::chrono::milliseconds>();
  stopwatch.reset();

  graph.reset(new RCGraph{working_dir / graph_folder});
  stopwatch.start();
  graph->load();
  stopwatch.stop();
  auto load_time = stopwatch.count<std::chrono::milliseconds>();
  stopwatch.reset();

  std::cout << "Total save time: " << save_time << "ms" << std::endl;
  std::cout << "Total load time: " << load_time << "ms" << std::endl;
}
