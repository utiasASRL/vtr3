#include <electric_sheep/pc_recorder.hpp>
#include <filesystem>

namespace fs = std::filesystem;

PCRecorder::PCRecorder(const std::string &data_dir,
                       const std::string &stream_name)
    : Node("pc_recorder"), writer_(data_dir, stream_name) {
  data_subscription_ = this->create_subscription<PointCloudMsg>(
      "/ros/lidar_point_cloud", 100,
      std::bind(&PCRecorder::pointcloudCallback, this, std::placeholders::_1));
}

void PCRecorder::pointcloudCallback(const PointCloudMsg::SharedPtr msg) {
  auto stamp = msg->header.stamp;
  auto time_epoch = stamp.sec * 1e9 + stamp.nanosec;
  std::cout << "Received pointcloud message at time: " << time_epoch << "\n";
  writer_.write(vtr::storage::VTRMessage(*msg));
}

/// @brief Record XB3 stereo images to a rosbag2
int main(int argc, char *argv[]) {
  // Default path
  fs::path data_dir{fs::current_path() / "pointcloud_data"};
  std::string stream_name = "point_cloud";

  // User specified path
  if (argc == 3) {
    data_dir = argv[1];
    stream_name = argv[2];
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCRecorder>(data_dir.string(), stream_name));
  rclcpp::shutdown();
  return 0;
}