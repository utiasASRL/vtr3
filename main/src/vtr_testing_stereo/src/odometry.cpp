#include "rclcpp/rclcpp.hpp"

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging_init.hpp>
#include <vtr_testing_stereo/odometry.hpp>

using namespace vtr;

using RigImagesMsg = vtr_messages::msg::RigImages;
using RigCalibrationMsg = vtr_messages::msg::RigCalibration;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  /// Log into a subfolder of the data directory (if requested to log)
  auto output_dir = node->declare_parameter<std::string>("output_dir", "/tmp");
  auto to_file = node->declare_parameter<bool>("log_to_file", false);
  std::string log_filename;
  if (to_file) {
    auto log_name = common::timing::toIsoFilename(common::timing::clock::now());
    log_filename = fs::path{common::utils::expand_user(
                       common::utils::expand_env(output_dir))} /
                   "logs" / (log_name + ".log");
  }
  logging::configureLogging(log_filename, true);
  LOG_IF(to_file, INFO) << "Logging to: " << log_filename;
  LOG_IF(!to_file, WARNING) << "NOT LOGGING TO A FILE.";

  /// Set odometry navigator
  LOG(INFO) << "Starting the Navigator node. Hello!";
  OdometryNavigator navigator{node, output_dir};

  /// Playback images
  auto input_dir = node->declare_parameter<std::string>("input_dir", "");
  storage::DataStreamReader<RigImagesMsg, RigCalibrationMsg> stereo_stream(
      common::utils::expand_user(common::utils::expand_env(input_dir)),
      "front_xb3");
  // fetch calibration
  auto calibration_msg =
      stereo_stream.fetchCalibration()->get<RigCalibrationMsg>();
  auto rig_calibration = vtr::messages::copyCalibration(calibration_msg);
  navigator.setCalibration(
      std::make_shared<vision::RigCalibration>(rig_calibration));
  // start playback images
  auto start_index = node->declare_parameter<int>("start_index", 1);
  auto stop_index = node->declare_parameter<int>("stop_index", 99999);

  bool seek_success =
      stereo_stream.seekByIndex(static_cast<int32_t>(start_index));
  if (!seek_success) {
    LOG(ERROR) << "Seek failed!";
    return 0;
  }

  int idx = 0;
  while (idx + start_index < stop_index && rclcpp::ok()) {
    auto storage_msg = stereo_stream.readNextFromSeek();
    if (!storage_msg) {
      LOG(ERROR) << "Storage msg is nullptr!";
      break;
    }
    auto rig_images = storage_msg->template get<RigImagesMsg>();
    /// \todo current datasets does not fill vtr_header so need this line
    rig_images.vtr_header.sensor_time_stamp.nanoseconds_since_epoch =
        rig_images.channels[0].cameras[0].stamp.nanoseconds_since_epoch;
    LOG(INFO) << "Processing image: " << idx;
    navigator.process(std::make_shared<RigImagesMsg>(rig_images));
    idx++;
  }

  LOG(INFO) << "Bye-bye!";
  rclcpp::shutdown();
}