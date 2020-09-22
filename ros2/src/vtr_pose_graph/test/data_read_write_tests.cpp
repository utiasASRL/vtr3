#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>
#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/sensor_test.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_storage/data_bubble.hpp>
#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;

/* Create the following graph:
 *   R0: 0 --- 1 --- 2 --- ... --- 9
 * Store some dummy data in each vertex and then load the data back from
 * vertices.
 */
TEST(PoseGraph, readWrite) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.
  fs::path graph_index_file{"graph_index"};
  int robot_id{666};

  // Initialize pose graph
  std::unique_ptr<RCGraph> graph{
      new RCGraph((working_dir / graph_index_file).string(), 0)};

  // Add the first run
  auto run_id = graph->addRun(robot_id);

  // Register a data read&write stream named test_data.
  std::string stream_name = "test_data";
  graph->registerVertexStream<vtr_messages::msg::SensorTest>(run_id,
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

  // Generate random data
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  // Insert some dummy messages to each vertex
  for (int vertex_idx = 0; vertex_idx < 10; ++vertex_idx) {
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    EXPECT_TRUE(vertex != nullptr);
    vtr_messages::msg::SensorTest test_msg;
    test_msg.value = distribution(generator);
    std::cout << "Store " << test_msg.value << " into vertex " << vertex_id
              << std::endl;
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

    auto message = vertex->retrieveKeyframeData<vtr_messages::msg::SensorTest>(
        stream_name);
    std::cout << "Vertex " << vertex_id << " has value " << (message->value)
              << std::endl;
  }
  std::cout << "Finished: robot id: " << robot_id << " run id: " << run_id
            << std::endl;

  // Cleanup
  fs::remove_all(working_dir);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
