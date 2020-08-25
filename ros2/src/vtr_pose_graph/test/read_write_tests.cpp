#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>
#include <vtr_logging/logging_init.hpp>
/// #include <asrl/messages/GPSMeasurement.pb.h>
#include <vtr_messages/msg/sensor_gps.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

#if 0
#include <robochunk/base/BaseChunkSerializer.hpp>
#endif

namespace fs = std::filesystem;

TEST(PoseGraph, readWrite) {
  /* Create the following graph
   * R0: 0 --- 1 --- 2 --- ... --- 100
   */
  using namespace vtr::pose_graph;

  fs::path working_dir{"/tmp/vtr_pose_graph_test"};
  fs::path graph_index_file{"graph_index"};
  int robot_id{666};
  std::unique_ptr<RCGraph> graph{
      new RCGraph((working_dir / graph_index_file).string(), 0)};

  std::string stream_name = "/gps/fix";

#if 0
  // add a graph with 1 runs and 100 vertices.
  // Create the robochunk directories
  std::string directory = working_directory_ + "/run_000000";
  robochunk::base::BaseChunkSerializer serializer(directory, stream_name,
                                                  true, 5.0);
#endif
  auto run_id = graph->addRun(robot_id);
#if 0
  graph->registerVertexStream(run_id, stream_name);
#endif
  // add the first vertex
  /// robochunk::std_msgs::TimeStamp stamp;
  /// stamp.set_nanoseconds_since_epoch(0);
  auto stamp = vtr_messages::msg::TimeStamp();
  stamp.nanoseconds_since_epoch = 0;
  graph->addVertex(stamp);
  for (int idx = 1; idx < 100; ++idx) {
    // add folloiwing edges and vertexes
    stamp.nanoseconds_since_epoch = idx * 10000;
    graph->addVertex(stamp);
    graph->addEdge(RCVertex::IdType(0, idx - 1), RCVertex::IdType(0, idx));
  }

  // make a test value
  double test_val = 100.0;

  // insert some dummy messages to each vertex
  for (int vertex_idx = 0; vertex_idx < 100; ++vertex_idx) {
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
    EXPECT_TRUE(vertex != nullptr);
    /// asrl::sensor_msgs::GPSMeasurement test_msg;
    auto test_msg = vtr_messages::msg::SensorGps();
    test_msg.altitude = test_val;
    test_msg.latitude = test_val;
    test_msg.longitude = test_val;
    /// robochunk::std_msgs::TimeStamp stamp;
    auto stamp = vtr_messages::msg::TimeStamp();
    stamp.nanoseconds_since_epoch = vertex_idx * 10000;
    vertex->insert(stream_name, test_msg, stamp);
  }
#if 0
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
#endif
  // now save out the data for all but the last few vertices
  for (int vertex_idx = 0; vertex_idx < 90; ++vertex_idx) {
    // access the vertex
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
#if 0
    vertex->write();
    vertex->unload();
#endif
  }
  // now try and load all the data
  for (int vertex_idx = 0; vertex_idx < 100; ++vertex_idx) {
    // access the vertex
    RCVertex::IdType vertex_id(0, vertex_idx);
    auto vertex = graph->at(vertex_id);
#if 0
    vertex->load(stream_name);
    auto message =
        vertex->retrieveKeyframeData<asrl::sensor_msgs::GPSMeasurement>(
            stream_name);

    // this is the important bit. If message is a nullptr, we are not using
    // the fixes in robochunk to flush index files
    REQUIRE(message != nullptr);

    // double check that the read values are the same as the input
    REQUIRE(message->altitude() == test_val);
    REQUIRE(message->latitude() == test_val);
    REQUIRE(message->longitude() == test_val);
#endif
  }
  std::cout << "Check: " << robot_id << " " << run_id << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#if 0
class ReadWriteTestFixture {
 public:
  ~ReadWriteTestFixture() {
    // Recursively remove the test directory tree.
    // TODO: Safer, better way to do this?
    std::stringstream command;
    command << "rm -r " << working_directory_.c_str();
    (void)(system(command.str().c_str()) +
           1);  // Literally the only way to suppress ignored return value
                // warnings in gcc46
  }
};
#endif
