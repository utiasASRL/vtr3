#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_navigation/navigator.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;
using namespace vtr::pose_graph;
using namespace vtr::tactic;

using LocEvaluator = eval::Mask::Privileged<RCGraph>::Caching;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  const auto input_dir_str =
      node->declare_parameter<std::string>("input_dir", "/tmp");
  fs::path input_dir{utils::expand_user(utils::expand_env(input_dir_str))};
  const auto listen_to_ros_topic =
      node->declare_parameter<bool>("listen_to_ros_topic", false);

  auto data_dir_str = node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};
  auto clear_data_dir = node->declare_parameter<bool>("clear_data_dir", false);
  if (clear_data_dir) fs::remove_all(data_dir);

  auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  auto log_debug = node->declare_parameter<bool>("log_debug", false);
  auto log_enabled =
      node->declare_parameter<std::vector<std::string>>("log_enabled", {});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  auto stop_frame = node->declare_parameter<int>("stop_frame_idx", 1000000);

  // Navigator node that runs everything
  Navigator navigator{node};

  /// Localization specific

  // Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(navigator.graph()->numberOfVertices());
  CLOG(INFO, "navigator") << "Total number of vertices: "
                          << navigator.graph()->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  LocEvaluator::Ptr evaluator(new LocEvaluator());
  evaluator->setGraph(navigator.graph().get());
  auto privileged_path = navigator.graph()->getSubgraph(0ul, evaluator);
  std::stringstream ss;
  ss << "Repeat vertices: ";
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    ss << it->v()->id() << " ";
    sequence.push_back(it->v()->id());
  }
  CLOG(INFO, "navigator") << ss.str();

  navigator.tactic()->setPipeline(PipelineMode::Following);
  navigator.tactic()->addRun();
  navigator.tactic()->setPath(sequence);

  /// Load dataset directly or listen to topics
  if (!listen_to_ros_topic) {
    rosbag2_cpp::StorageOptions storage_options;
    storage_options.uri = input_dir.string();
    storage_options.storage_id = "sqlite3";
    storage_options.max_bagfile_size = 0;  // default
    storage_options.max_cache_size = 0;    // default
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    rosbag2_cpp::Reader reader(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, converter_options);

    int frame_idx = 0;
    while (rclcpp::ok() && reader.has_next()) {
      if (frame_idx == stop_frame) break;
      // load rosbag message
      auto bag_message = reader.read_next();
      if (bag_message->topic_name != "/points") continue;

      rclcpp::SerializedMessage msg(*bag_message->serialized_data);
      auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
      serialization.deserialize_message(&msg, points.get());

      CLOG(INFO, "navigator")
          << "Loaded message with time stamp: " << std::fixed
          << points->header.stamp.sec * 1e9 + points->header.stamp.nanosec;

      // Convert message to query_data format and store into query_data
      auto query_data = std::make_shared<lidar::LidarQueryCache>();

      /// \todo (yuchen) need to distinguish this with stamp
      query_data->rcl_stamp.fallback(points->header.stamp);

      // set time stamp
      navigation::TimeStampMsg stamp;
      stamp.nanoseconds_since_epoch =
          points->header.stamp.sec * 1e9 + points->header.stamp.nanosec;
      query_data->stamp.fallback(stamp);

      // put in the pointcloud msg pointer into query data
      query_data->pointcloud_msg = points;

      // fill in the vehicle to sensor transform and frame names
      query_data->robot_frame.fallback(navigator.robot_frame());
      query_data->lidar_frame.fallback(navigator.lidar_frame());
      query_data->T_s_r.fallback(navigator.T_lidar_robot());

      // execute the pipeline
      navigator.tactic()->runPipeline(query_data);

      // handle any transitions triggered by changes in localization status
      navigator.sm()->handleEvents();

      frame_idx++;
    }
  } else {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
}