#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_navigation/navigator.hpp>

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
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      sensor_msgs::msg::PointCloud2 points;
      rclcpp::SerializedMessage msg(*bag_message->serialized_data);
      serialization.deserialize_message(&msg, &points);
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