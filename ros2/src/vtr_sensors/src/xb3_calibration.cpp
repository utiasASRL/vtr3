#include <vtr_sensors/xb3_calibration.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <thread>

namespace fs = std::filesystem;

Xb3Calibration::Xb3Calibration(const std::shared_ptr<rclcpp::Node> node,
                               const std::string &data_dir,
                               const std::string &stream_name)
    : node_(node), reader_(data_dir, stream_name) {
  // calibration service
  calibration_msg_ = reader_.fetchCalibration()->get<RigCalibration>();
  calibration_srv_ = node_->create_service<GetRigCalibration>(
      "xb3_calibration",
      std::bind(&Xb3Calibration::_calibrationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void Xb3Calibration::_calibrationCallback(
    const std::shared_ptr<GetRigCalibration::Request> request,
    std::shared_ptr<GetRigCalibration::Response> response) {
  response->calibration = calibration_msg_;
}

int main(int argc, char **argv) {
  // Default path
  fs::path data_dir{fs::current_path() / "xb3_data"};
  std::string stream_name = "front_xb3";

  // User specified path
  if (argc == 3) {
    data_dir = argv[1];
    stream_name = argv[2];
  } else if (argc != 1) {
    throw std::invalid_argument("Wrong number of arguments provided!");
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("xb3_calibration");

  Xb3Calibration xb3_calibration{node, data_dir.string(), stream_name};

  // Wait for shutdown
  rclcpp::spin(node);
  rclcpp::shutdown();
}
