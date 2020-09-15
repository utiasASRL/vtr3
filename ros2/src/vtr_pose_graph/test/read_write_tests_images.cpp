#include <filesystem>
#include <iostream>
#include <vtr_logging/logging_init.hpp>
#include <vtr_messages/msg/sensor_gps.hpp>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_storage/data_bubble.hpp>
#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

#include "test_msgs/msg/basic_types.hpp"

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

namespace fs = std::filesystem;

/* Create the following graph:
 *   R0: 0 --- 1 --- 2 --- ... --- 9
 * Store some dummy data in each vertex and then load the data back from
 * vertices.
 */
int main() {
  using namespace vtr::pose_graph;

  fs::path working_dir{"/home/daniel/Desktop/ASRL/temp"};
  fs::remove_all(working_dir);  // make sure the directoy is empty.
  std::cout << "Temp dir is " << working_dir << std::endl;

  fs::path image_dir{"/home/daniel/Desktop/ASRL/test/"};
  cv::namedWindow("output", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  cv::resizeWindow("output", 700, 700);

  fs::path graph_index_file{"graph_index"};
  int robot_id{666};

  // Initialize pose graph
  std::unique_ptr<RCGraph> graph{
      new RCGraph((working_dir / graph_index_file).string(), 0)};

  // Add the first run
  auto run_id = graph->addRun(robot_id);

  // Register a data read&write stream named test_data.
  std::string stream_name = "test_data";
  graph->registerVertexStream<sensor_msgs::msg::Image>(run_id, stream_name);

  auto stamp = vtr_messages::msg::TimeStamp();  // a custom ros2 message.
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
    /// robochunk::std_msgs::TimeStamp stamp;
    cv::Mat image = cv::imread(
        (image_dir / (std::to_string(vertex_idx + 1) + ".jpg")).string(),
        cv::IMREAD_COLOR);
    cv::imshow("output", image);
    cv::waitKey(0);
    cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", image);
    auto img_msg = cv_image.toImageMsg();

    auto stamp = vtr_messages::msg::TimeStamp();
    stamp.nanoseconds_since_epoch = vertex_idx;
    vertex->insert(stream_name, *img_msg, stamp);
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
        vertex->retrieveData<sensor_msgs::msg::Image>(stream_name, 0);
    std::cout << "Showing vertex " << vertex_id << std::endl;
    auto cv_image = cv_bridge::toCvCopy(*message, "bgr8");
    cv::imshow("output", cv_image->image);
    cv::waitKey(0);
  }
  std::cout << "Finished: robot id: " << robot_id << " run id: " << run_id
            << std::endl;

  // Cleanup
  // fs::remove_all(working_dir);
}
