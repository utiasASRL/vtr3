#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/storage_options.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_navigation/navigator.hpp>

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;
using namespace vtr::tactic;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  const auto input_dir_str =
      node->declare_parameter<std::string>("input_dir", "/tmp");
  fs::path input_dir{utils::expand_user(utils::expand_env(input_dir_str))};
  const auto listen_to_ros_topic =
      node->declare_parameter<bool>("listen_to_ros_topic", false);

  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};
  const auto clear_data_dir =
      node->declare_parameter<bool>("clear_data_dir", false);
  if (clear_data_dir) fs::remove_all(data_dir);

  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled =
      node->declare_parameter<std::vector<std::string>>("log_enabled", {});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  // Navigator node that runs everything
  Navigator navigator{node};

  /// Odometry specific

  navigator.tactic()->setPipeline(PipelineMode::Branching);
  navigator.tactic()->addRun();

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
    while (rclcpp::ok() && reader.has_next()) {
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
    }
  } else {
    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  /// Save odometry result
  CLOG(INFO, "navigator") << "Saving odometry results.";
  auto odometry_poses = navigator.tactic()->odometryPoses();
  std::ofstream outstream;
  outstream.open(data_dir / "odometry_result.txt");
  outstream << std::setprecision(6) << std::scientific;
  for (auto pose : odometry_poses) {
    const auto& tmp = pose.matrix();
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 4; j++) {
        outstream << tmp(i, j);
        if (i != 2 || j != 3) outstream << " ";
      }
    outstream << "\n";
  }
  outstream.close();
}