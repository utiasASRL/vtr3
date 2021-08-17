#include <filesystem>

#include <vtr_common/utils/filesystem.hpp>
#include <vtr_bumblebee_xb3/xb3_recorder.hpp>

namespace fs = std::filesystem;

Xb3Recorder::Xb3Recorder(const std::string &data_dir,
                         const std::string &stream_name)
    : Node("xb3_recorder"),
      writer_(data_dir, stream_name),
      calib_writer_(data_dir, "calibration") {
  data_subscription_ = this->create_subscription<RigImages>(
      "xb3_images", rclcpp::SensorDataQoS(),
      std::bind(&Xb3Recorder::_imageCallback, this, std::placeholders::_1));

  rig_calibration_client_ =
      this->create_client<GetRigCalibration>("xb3_calibration");
}

void Xb3Recorder::_imageCallback(const RigImages::SharedPtr msg) {
  if (!rig_calibration_) {
    _fetchCalibration();
  } else if (!calibration_recorded_) {
    calib_writer_.write(vtr::storage::VTRMessage(*rig_calibration_));
    calibration_recorded_ = true;
  }

  writer_.write(vtr::storage::VTRMessage(*msg));
  image_count_++;
  std::cout << "Image count: " << image_count_ << std::endl;
}

void Xb3Recorder::_fetchCalibration() {
  // wait for the service
  while (!rig_calibration_client_->wait_for_service()) {
    if (!rclcpp::ok()) {
      return;
    }
  }

  // send and wait for the result
  auto request = std::make_shared<GetRigCalibration::Request>();
  auto response_callback =
      [this](rclcpp::Client<GetRigCalibration>::SharedFuture future) {
        auto response = future.get();
        this->rig_calibration_ = std::make_shared<RigCalibration>(response->calibration);
      };
  auto response =
      rig_calibration_client_->async_send_request(request, response_callback);
}

/// @brief Record XB3 stereo images to a rosbag2
int main(int argc, char *argv[]) {
  // Default path
  fs::path data_dir{fs::current_path() / "xb3_data"};
  std::string stream_name = "front_xb3";

  // User specified path
  if (argc == 3) {
    data_dir = fs::path{vtr::common::utils::expand_user(vtr::common::utils::expand_env(argv[1]))};
    stream_name = argv[2];
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Xb3Recorder>(data_dir.string(), stream_name));
  rclcpp::shutdown();
  return 0;
}