#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>
#include <vtr_logging/logging_init.hpp>
/// #include <asrl/messages/GPSMeasurement.pb.h>
#include <vtr_messages/msg/sensor_gps.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include "vtr_storage/DataBubble.hpp"
#include "vtr_storage/DataStreamReader.hpp"
#include "vtr_storage/DataStreamWriter.hpp"

#if 0
#include <robochunk/base/BaseChunkSerializer.hpp>
#endif

#include "test_msgs/msg/basic_types.hpp"

namespace fs = std::filesystem;

/* Create the following graph:
 *   R0: 0 --- 1 --- 2 --- ... --- 9
 * Store some dummy data in each vertex and then load the data back from
 * vertices.
 */
TEST(PoseGraph, readWrite) {
  using namespace vtr::pose_graph;

  fs::path working_dir{fs::temp_directory_path() / "vtr_pose_graph_test"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.
  std::cout << "Temp dir is " << working_dir << std::endl;
  fs::path graph_index_file{"graph_index"};
  int robot_id{666};

  // Initialize pose graph
  std::unique_ptr<RCGraph> graph{
      new RCGraph((working_dir / graph_index_file).string(), 0)};

#if 0
  // add a graph with 1 runs and 100 vertices.
  // Create the robochunk directories
  std::string directory = working_directory_ + "/run_000000";
  robochunk::base::BaseChunkSerializer serializer(directory, stream_name,
                                                  true, 5.0);
#endif
  // Add the first run
  auto run_id = graph->addRun(robot_id);

  // Register a data read&write stream named test_data.
  std::string stream_name = "test_data";
  graph->registerVertexStream(run_id, stream_name);

  // Add the first vertex
#if 0
  /// robochunk::std_msgs::TimeStamp stamp;
  /// stamp.set_nanoseconds_since_epoch(0);
#endif
  auto stamp = vtr_messages::msg::TimeStamp();  // a custom ros2 message.
  stamp.nanoseconds_since_epoch = 0;
  graph->addVertex(stamp);
  for (int idx = 1; idx < 10; ++idx) {
    // Add following edges and vertexes
    stamp.nanoseconds_since_epoch = idx * 10000;
    graph->addVertex(stamp);
    graph->addEdge(RCVertex::IdType(0, idx - 1), RCVertex::IdType(0, idx));
  }

  // Make some test data
  // The fake sensor data we use in this test, `test_msgs::msg::BasicTypes{}`,
  // has only 1 entry called `float64_value`, which stores a `float64`.
  std::vector<double> test_data;
  for (int i = 0; i < 10; i++) test_data.push_back(static_cast<double>(i));

  // Insert some dummy messages to each vertex
  for (int vertex_idx = 0; vertex_idx < 10; ++vertex_idx) {
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
#if 0
    EXPECT_TRUE(vertex != nullptr);
    /// asrl::sensor_msgs::GPSMeasurement test_msg;
    // auto test_msg = vtr_messages::msg::SensorGps();
    // test_msg.altitude = test_val;
    // test_msg.latitude = test_val;
    // test_msg.longitude = test_val;
#endif
    auto test_msg = test_msgs::msg::BasicTypes{};
    test_msg.float64_value = test_data[vertex_idx];
    std::cout << "Store " << test_data[vertex_idx] << " into vertex "
              << vertex_id << std::endl;
    /// robochunk::std_msgs::TimeStamp stamp;
    auto stamp = vtr_messages::msg::TimeStamp();
    stamp.nanoseconds_since_epoch = vertex_idx * 10000;
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
#if 0
    /// auto message =
    ///     vertex->retrieveKeyframeData<asrl::sensor_msgs::GPSMeasurement>(
    ///         stream_name);
#endif
    auto message =
        vertex->retrieveData<test_msgs::msg::BasicTypes>(stream_name, 0);
    std::cout << "Vertex " << vertex_id << " has value "
              << (message->float64_value) << std::endl;
#if 0
    // this is the important bit. If message is a nullptr, we are not using
    // the fixes in robochunk to flush index files
    REQUIRE(message != nullptr);

    // double check that the read values are the same as the input
    REQUIRE(message->altitude() == test_val);
    REQUIRE(message->latitude() == test_val);
    REQUIRE(message->longitude() == test_val);
#endif
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
