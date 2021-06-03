#include <electric_sheep/pc_replay.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace fs = std::filesystem;

PCReplay::PCReplay(const std::string &data_dir, const std::string &stream_name,
                   const std::string &topic, const int qos)
    : Node("pc_replay"), reader_(data_dir, stream_name) {
  publisher_ = create_publisher<PointCloudMsg>(topic, qos);
  clock_publisher_ = create_publisher<ClockMsg>("clock", 10);
}

/// @brief Replay XB3 stereo images from a rosbag2
int main(int argc, char *argv[]) {
  // Default path
  fs::path data_dir{fs::current_path() / "pointcloud_data"};
  std::string stream_name = "pointcloud";
  bool manual_scrub = false;

  bool use_original_timestamps = true; // todo (Ben): add option as CLI arg
  double framerate = 16.0;

  // temp
  double delay_scale = 1.0; // make playblack slower
  double time_shift = 0;    // shift time stamp for repeat
  int start_index = 1;
  int stop_index = 9999999;

  int frame_skip = 1;

  // User specified path
  if (argc >= 4) {
    data_dir = argv[1];
    stream_name = argv[2];
    std::istringstream(argv[3]) >> std::boolalpha >> manual_scrub;
    if (manual_scrub) {
      std::cout << "Manual replay selected. Press/hold any key to advance "
                   "image stream."
                << std::endl;
    }
    if (argc >= 5)
      start_index = atof(argv[4]);
    if (argc >= 6)
      stop_index = atof(argv[5]);
    if (argc >= 7)
      delay_scale = atof(argv[6]);
    if (argc >= 8)
      time_shift = atof(argv[7]);
    if (argc >= 9)
      frame_skip = atof(argv[8]);
  } else if (argc != 1) {
    throw std::invalid_argument("Wrong number of arguments provided!");
  }

  rclcpp::init(argc, argv);
  auto replay = PCReplay(data_dir.string(), stream_name, "raw_points");

  replay.reader_.seekByIndex(start_index);

  std::string inputString;
  std::cout << "Enter to start!";
  std::cin.clear();
  std::getline(std::cin, inputString);

  int curr_index = start_index;

  uint64_t prev_stamp = 0;
  bool terminate = false;
  int frame_num = -1;
  while (true) {
    if (!rclcpp::ok())
      break;

    PointCloudMsg pointcloud_msg;

    for (int i = 0; i < 5; i++) {

      if (curr_index == stop_index) {
        terminate = true;
        break;
      }
      curr_index++;

      auto message = replay.reader_.readNextFromSeek();
      if (!message.get()) {
        terminate = true;
        break;
      }

      auto pcd = message->template get<PointCloudMsg>();

      std::cout << std::endl << "==== point cloud info === " << std::endl;
      std::cout << "header stamp - sec: " << pcd.header.stamp.sec
                << ", nsec: " << pcd.header.stamp.nanosec << std::endl;
      std::cout << "header frame_id: " << pcd.header.frame_id << std::endl;
      std::cout << "height: " << pcd.height << std::endl;
      std::cout << "width: " << pcd.width << std::endl;
      std::cout << "is_bigendian: " << pcd.is_bigendian << std::endl;
      std::cout << "point_step: " << pcd.point_step << std::endl;
      std::cout << "row_step: " << pcd.row_step << std::endl;
      std::cout << "data size: " << pcd.data.size() << std::endl;
      std::cout << "is dense: " << pcd.is_dense << std::endl;
      for (auto pf : pcd.fields) {
        std::cout << "field - name: " << pf.name << ", offset: " << pf.offset
                  << ", datatype: " << pf.datatype << ", count: " << pf.count
                  << std::endl;
      }

      if (i == 0) {
        pointcloud_msg.header = pcd.header;
        pointcloud_msg.header.stamp = replay.now();
        pointcloud_msg.header.frame_id = "velodyne";
        pointcloud_msg.height = pcd.height;
        pointcloud_msg.width = pcd.width;
        pointcloud_msg.is_bigendian = pcd.is_bigendian;
        pointcloud_msg.point_step = pcd.point_step;
        pointcloud_msg.row_step = pcd.row_step;
        pointcloud_msg.data = pcd.data;
        pointcloud_msg.is_dense = pcd.is_dense;
        pointcloud_msg.fields = pcd.fields;
      } else {
        pointcloud_msg.width += pcd.width;
        pointcloud_msg.row_step += pcd.row_step;
        pointcloud_msg.data.reserve(pointcloud_msg.data.size() +
                                    pcd.data.size());
        pointcloud_msg.data.insert(pointcloud_msg.data.end(), pcd.data.begin(),
                                   pcd.data.end());
      }
    }

    std::cout << std::endl << "==== MERGED POINT CLOUD INFO === " << std::endl;
    std::cout << "header stamp - sec: " << pointcloud_msg.header.stamp.sec
              << ", nsec: " << pointcloud_msg.header.stamp.nanosec << std::endl;
    std::cout << "header frame_id: " << pointcloud_msg.header.frame_id
              << std::endl;
    std::cout << "height: " << pointcloud_msg.height << std::endl;
    std::cout << "width: " << pointcloud_msg.width << std::endl;
    std::cout << "is_bigendian: " << pointcloud_msg.is_bigendian << std::endl;
    std::cout << "point_step: " << pointcloud_msg.point_step << std::endl;
    std::cout << "row_step: " << pointcloud_msg.row_step << std::endl;
    std::cout << "data size: " << pointcloud_msg.data.size() << std::endl;
    std::cout << "is dense: " << pointcloud_msg.is_dense << std::endl;
    for (auto pf : pointcloud_msg.fields) {
      std::cout << "field - name: " << pf.name << ", offset: " << pf.offset
                << ", datatype: " << pf.datatype << ", count: " << pf.count
                << std::endl;
    }

    if (terminate)
      break;

    frame_num++;
    std::cout << "===> frame_num is: " << frame_num << std::endl;
    if (frame_num % frame_skip != 0) {
      continue;
    }

    // // \todo yuchen Add necessary info for vtr to run, but they should not be
    // // here
    // image.name = "front_xb3";
    // image.vtr_header.sensor_time_stamp = image.channels[0].cameras[0].stamp;
    // image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch +=
    //     1e12 * time_shift;

    // ClockMsg clock_msg;
    // clock_msg.clock.sec =
    //     image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch / 1e9;
    // clock_msg.clock.nanosec =
    //     image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch %
    //     (long)1e9;
    // // Publish time to /clock for nodes using sim_time
    // replay.clock_publisher_->publish(clock_msg);

    // std::cout << "Publishing image with time stamp: "
    //           << image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch
    //           << " and index is " << curr_index << std::endl;

    // Publish message for use with offline tools
    std::cout << "===> curr_index is: " << curr_index << std::endl;
    replay.publisher_->publish(pointcloud_msg);

    // Replays images based on their timestamps. Converts nanoseconds to
    // milliseconds
    if (manual_scrub) {
      std::cin.clear();
      std::getline(std::cin, inputString);
    } else {
      // Add a delay so that the image publishes at roughly the true rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds((int)(delay_scale * 50)));

      // if (prev_stamp != 0) {
      //   double delay = use_original_timestamps
      //                      ? (left.stamp.nanoseconds_since_epoch -
      //                      prev_stamp) *
      //                            std::pow(10, -6)
      //                      : 1000.0 / framerate;
      //   delay *= delay_scale;
      //   cv::waitKey(delay);
      // }
    }
    // prev_stamp = left.stamp.nanoseconds_since_epoch;
  }
  rclcpp::shutdown();
}
