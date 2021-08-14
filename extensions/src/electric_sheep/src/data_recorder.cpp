#include <filesystem>

#include <electric_sheep/data_recorder.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using NavSatFixMsg = sensor_msgs::msg::NavSatFix;

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
  // Default path
  fs::path data_dir{fs::current_path() / "pointcloud_data"};
  std::string stream_name = "pointcloud";  // "gps"
  std::string topic_name = "/points";      // "/msa/gps_fix"

  // User specified path
  if (argc == 4) {
    data_dir = argv[1];
    stream_name = argv[2];
    topic_name = argv[3];
  }

  rclcpp::init(argc, argv);
  // PointCloudMsg, NavSatFixMsg
  rclcpp::spin(std::make_shared<Recorder<PointCloudMsg>>(
      data_dir.string(), stream_name, topic_name));
  rclcpp::shutdown();
  return 0;
}