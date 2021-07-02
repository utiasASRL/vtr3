#include <chrono>
#include <filesystem>
#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/opencv.hpp>

#include <electric_sheep/pc_replay.hpp>

namespace fs = std::filesystem;

PCReplay::PCReplay(const std::string &data_dir, const std::string &stream_name,
                   const std::string &topic, int replay_mode, int start_index,
                   int stop_index, double delay_scale, double time_shift,
                   int frame_skip)
    : Node("pc_replay"),
      replay_mode_(replay_mode),
      start_index_(start_index),
      stop_index_(stop_index),
      delay_scale_(delay_scale),
      time_shift_(time_shift),
      frame_skip_(frame_skip),
      curr_index_(start_index),
      reader_(data_dir, stream_name) {
  /// create publishers
  publisher_ = create_publisher<PointCloudMsg>(topic, 10);
  clock_publisher_ = create_publisher<ClockMsg>("/clock", 1);

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  /// Publish the current frame localized against in world frame
  Eigen::Affine3d T(Eigen::Matrix4d::Identity());
  auto msg = tf2::eigenToTransform(T);
  msg.header.frame_id = "robot";
  msg.header.stamp = now();
  msg.child_frame_id = "velodyne";
  tf_broadcaster_->sendTransform(msg);
  msg.header.frame_id = "base_link";
  tf_broadcaster_->sendTransform(msg);
  // std::cout << "Sending transform!" << std::endl;

  std::cout << "Replay config: " << std::endl;
  std::cout << "replay_mode_: " << replay_mode_ << std::endl;
  std::cout << "start_index_: " << start_index_ << std::endl;
  std::cout << "stop_index_: " << stop_index_ << std::endl;
  std::cout << "delay_scale_: " << delay_scale_ << std::endl;
  std::cout << "time_shift_: " << time_shift_ << std::endl;
  std::cout << "frame_skip_: " << frame_skip_ << std::endl;
  std::cout << "curr_index_: " << curr_index_ << std::endl;

  ///
  reader_.seekByIndex(curr_index_);

  std::string inputString;
  std::cout << "Enter to start!";
  std::cin.clear();
  std::getline(std::cin, inputString);

  while (true) {
    if (!rclcpp::ok()) break;

    /// Publish the current frame localized against in world frame
    Eigen::Affine3d T(Eigen::Matrix4d::Identity());
    auto msg = tf2::eigenToTransform(T);
    msg.header.frame_id = "robot";
    msg.header.stamp = now();
    msg.child_frame_id = "velodyne";
    tf_broadcaster_->sendTransform(msg);
    msg.header.frame_id = "base_link";
    tf_broadcaster_->sendTransform(msg);
    // std::cout << "Sending transform!" << std::endl;

    switch (replay_mode_) {
      case 0:
        std::cin.clear();
        std::getline(std::cin, inputString);
        if (inputString == "q") return;
        break;
      case 1:
        // Add a delay so that the image publishes at roughly the true rate.
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(delay_scale * 50)));
        break;
      default:
        throw std::runtime_error{"Unknown replay mode."};
    }
    publish();
  }
}

void PCReplay::publish() {
  while (true) {
    int second = 0;
    int nanosec = 0;

    PointCloudMsg pointcloud_msg;
    pointcloud_msg.width = 0;
    pointcloud_msg.row_step = 0;
    pointcloud_msg.data.clear();
    pointcloud_msg.data.reserve(30000);

    for (int i = 0; i < 5; i++) {
      if (curr_index_ == stop_index_) {
        terminate_ = true;
        break;
      }
      curr_index_++;

      auto message = reader_.readNextFromSeek();
      if (!message.get()) {
        terminate_ = true;
        break;
      }

      auto pcd = message->template get<PointCloudMsg>();

      second = pcd.header.stamp.sec;
      nanosec = pcd.header.stamp.nanosec;
      if (start_second_ == -1) start_second_ = second;

      // std::cout << std::endl << "==== point cloud info === " << std::endl;
      // std::cout << "header stamp - sec: " << pcd.header.stamp.sec
      //           << ", nsec: " << pcd.header.stamp.nanosec << std::endl;
      // std::cout << "header frame_id: " << pcd.header.frame_id << std::endl;
      // std::cout << "height: " << pcd.height << std::endl;
      // std::cout << "width: " << pcd.width << std::endl;
      // std::cout << "is_bigendian: " << pcd.is_bigendian << std::endl;
      // std::cout << "point_step: " << pcd.point_step << std::endl;
      // std::cout << "row_step: " << pcd.row_step << std::endl;
      // std::cout << "data size: " << pcd.data.size() << std::endl;
      // std::cout << "is dense: " << pcd.is_dense << std::endl;
      // for (auto pf : pcd.fields) {
      //   std::cout << "field - name: " << pf.name << ", offset: " << pf.offset
      //             << ", datatype: " << pf.datatype << ", count: " << pf.count
      //             << std::endl;
      // }

      pointcloud_msg.width += pcd.width;
      pointcloud_msg.row_step += pcd.width * 20;  // 3 float + 1 double
      //
      double time_stamp = (double)second + (double)nanosec / 1e9;
      std::vector<char> ts_vec(sizeof(double));
      std::memcpy(&ts_vec[0], (char *)&time_stamp, sizeof(double));
      for (size_t j = 0; j < pcd.data.size(); j += 16) {
        // point xyz
        pointcloud_msg.data.insert(pointcloud_msg.data.end(),
                                   pcd.data.begin() + j,
                                   pcd.data.begin() + j + 12);
        // time stamp
        pointcloud_msg.data.insert(pointcloud_msg.data.end(), ts_vec.begin(),
                                   ts_vec.end());
      }

      if (i == 4) {
        pointcloud_msg.header = pcd.header;
        pointcloud_msg.header.frame_id = "velodyne";
        pointcloud_msg.height = pcd.height;
        // pointcloud_msg.width = pcd.width;  // accumulated
        pointcloud_msg.is_bigendian = pcd.is_bigendian;
        pointcloud_msg.point_step = 20;  // 3 float + 1 double
        // pointcloud_msg.row_step = pcd.row_step;  // accumulated
        // pointcloud_msg.data = pcd.data;  // accumulated
        pointcloud_msg.is_dense = pcd.is_dense;
        auto fields = pcd.fields;
        fields[3] = PointFieldMsg();
        fields[3].name = "t";
        fields[3].offset = 12;
        fields[3].datatype = 8;
        fields[3].count = 1;
        pointcloud_msg.fields = fields;
      }
    }

    if (terminate_) break;

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

    frame_num_++;
    if (frame_num_ % frame_skip_ != 0) continue;
    std::cout << "===> number of frame (complete rev.): " << frame_num_;
    std::cout << ", current data index: " << curr_index_;
    std::cout << ", time (sec): " << second - start_second_ << std::endl;

    ClockMsg clock_msg;
    clock_msg.clock = pointcloud_msg.header.stamp;
    clock_publisher_->publish(clock_msg);

    // Publish message for use with offline tools
    publisher_->publish(pointcloud_msg);
    break;
  }
}

int main(int argc, char *argv[]) {
  // Default path
  fs::path data_dir{fs::current_path()};
  std::string stream_name = "pointcloud";

  bool use_original_timestamps = true;  // todo (Ben): add option as CLI arg
  double framerate = 16.0;

  // temp
  int replay_mode = 0;
  int start_index = 1;
  int stop_index = 9999999;
  double delay_scale = 1.0;  // make playblack slower
  double time_shift = 0;     // shift time stamp for repeat
  int frame_skip = 1;

  // User specified path
  if (argc >= 3) {
    data_dir = argv[1];
    stream_name = argv[2];
    if (argc >= 4) replay_mode = atoi(argv[3]);
    if (argc >= 5) start_index = atof(argv[4]);
    if (argc >= 6) stop_index = atof(argv[5]);
    if (argc >= 7) delay_scale = atof(argv[6]);
    if (argc >= 8) time_shift = atof(argv[7]);
    if (argc >= 9) frame_skip = atof(argv[8]);
  } else if (argc != 1) {
    throw std::invalid_argument("Wrong number of arguments provided!");
  }

  rclcpp::init(argc, argv);
  auto replay =
      PCReplay(data_dir.string(), stream_name, "raw_points", replay_mode,
               start_index, stop_index, delay_scale, time_shift, frame_skip);
  rclcpp::shutdown();
}
