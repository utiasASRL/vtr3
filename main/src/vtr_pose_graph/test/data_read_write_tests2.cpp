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
#include <iostream>
#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_storage/stream/data_bubble.hpp>
#include <vtr_storage/stream/data_stream_reader.hpp>
#include <vtr_storage/stream/data_stream_writer.hpp>

#if 0
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#endif

namespace fs = std::filesystem;
using namespace vtr::pose_graph;
using namespace vtr::common;

/* Create the following graph:
 *   R0: 0 --- 1 --- 2 --- ... --- 9
 * Store some dummy data in each vertex and then load the data back from
 * vertices.
 */
TEST(PoseGraph, readWrite) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.
#if 0
  fs::path image_dir{"/home/daniel/Desktop/ASRL/test/"};
  cv::namedWindow("output", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  cv::resizeWindow("output", 700, 700);
#endif
  fs::path graph_index_file{"graph_index"};
  int robot_id{666};

  // Initialize pose graph
  std::unique_ptr<RCGraph> graph{
      new RCGraph((working_dir / graph_index_file).string(), 0)};

  // Add the first run
  auto run_id = graph->addRun(robot_id);

  // Register a data read&write stream named test_data.
  std::string stream_name = "test_data";
  graph->registerVertexStream<vtr_messages::msg::RigImages>(run_id,
                                                            stream_name);

  // Add the first vertex
  vtr_messages::msg::TimeStamp stamp;
  stamp.nanoseconds_since_epoch = 0;
  graph->addVertex(stamp);
  for (int idx = 1; idx < 10; ++idx) {
    // Add following edges and vertexes
    stamp.nanoseconds_since_epoch = idx;
    graph->addVertex(stamp);
    graph->addEdge(RCVertex::IdType(0, idx - 1), RCVertex::IdType(0, idx));
  }

  // Insert some dummy messages to each vertex
  for (int vertex_idx = 0; vertex_idx < 10; ++vertex_idx) {
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    EXPECT_TRUE(vertex != nullptr);
    vtr_messages::msg::RigImages test_msg;
    test_msg.name = "image_" + std::to_string(vertex_idx);
    std::cout << "Store " << test_msg.name << " into vertex " << vertex_id
              << std::endl;
#if 0
    /// \todo load an actual image into test_msg
    auto image = cv::imread(
        (image_dir / (std::to_string(vertex_idx + 1) + ".jpg")).string(),
        cv::IMREAD_COLOR);
    cv::imshow("output", image);
    cv::waitKey(0);
    cv_bridge::CvImage cv_image{std_msgs::msg::Header(), "bgr8", image};
    auto img_msg = cv_image.toImageMsg();
#endif
    vtr_messages::msg::TimeStamp stamp;
    stamp.nanoseconds_since_epoch = vertex_idx;
    vertex->insert(stream_name, test_msg, stamp);
  }

  // Now save out the data for all but the last few vertices
  std::cout << "Saving data to disk" << std::endl;
  for (int vertex_idx = 0; vertex_idx < 10; ++vertex_idx) {
    // access the vertex
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    vertex->write();
    vertex->unload();
  }

  // Now load all the data back from disk.
  std::cout << "Loading data from disk" << std::endl;
  for (int vertex_idx = 9; vertex_idx >= 0; --vertex_idx) {
    // access the vertex
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    vertex->load(stream_name);

    auto message =
        vertex->retrieveKeyframeData<vtr_messages::msg::RigImages>(stream_name);
    std::cout << "Vertex " << vertex_id << " has name " << (message->name)
              << std::endl;
#if 0
    std::cout << "Showing vertex " << vertex_id << std::endl;
    auto cv_image = cv_bridge::toCvCopy(*message, "bgr8");
    cv::imshow("output", cv_image->image);
    cv::waitKey(0);
#endif
  }
  std::cout << "Finished: robot id: " << robot_id << " run id: " << run_id
            << std::endl;

  // Cleanup
  fs::remove_all(working_dir);
}

int main(int argc, char** argv) {
  vtr::logging::configureLogging();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
