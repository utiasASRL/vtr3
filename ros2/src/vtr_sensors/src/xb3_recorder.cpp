#include <vtr_sensors/xb3_recorder.hpp>
#include <filesystem>

namespace fs = std::filesystem;

Xb3Recorder::Xb3Recorder(const std::string &data_dir, const std::string &stream_name)
    : Node("xb3_recorder"), writer_(data_dir, stream_name), calib_writer_(data_dir, "xb3_calibration") {

  data_subscription_ = this->create_subscription<RigImages>("xb3_images",
                                                            10,
                                                            std::bind(&Xb3Recorder::imageCallback,
                                                                      this,
                                                                      std::placeholders::_1));
  calib_subscription_ = this->create_subscription<RigImages>("xb3_calibration",
                                                             10,
                                                             std::bind(&Xb3Recorder::calibCallback,
                                                                       this,
                                                                       std::placeholders::_1));
}

void Xb3Recorder::imageCallback(const RigImages::SharedPtr msg) {
  writer_.write(vtr::storage::VTRMessage(*msg));
}

void Xb3Recorder::calibCallback(const RigCalibration::SharedPtr msg) {
  calib_writer_.write(vtr::storage::VTRMessage(*msg));
}

/// @brief Record XB3 stereo images to a rosbag2
int main(int argc, char *argv[]) {
  //Default path
  fs::path data_dir{fs::current_path() / "xb3_data"};
  std::string stream_name = "front_xb3";

  //User specified path
  if (argc == 3) {
    data_dir = argv[1];
    stream_name = argv[2];
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Xb3Recorder>(data_dir.string(), stream_name));
  rclcpp::shutdown();
  return 0;
}