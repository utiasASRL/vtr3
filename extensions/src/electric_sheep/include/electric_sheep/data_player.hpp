#pragma once

#include <chrono>
#include <filesystem>
#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;

using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using ClockMsg = rosgraph_msgs::msg::Clock;

namespace fs = std::filesystem;

template <class MessageType>
class DataPlayer : public rclcpp::Node {
 public:
  DataPlayer(const std::string &data_dir, const std::string &stream_name,
             const std::string &topic, int replay_mode = 0, int start_index = 1,
             int stop_index = 9999999, double delay_scale = 1,
             double time_shift = 0, int frame_skip = 1);

  void publish();

 private:
  int replay_mode_ = 0;
  int start_index_ = 1;
  int stop_index_ = 9999999;
  double delay_scale_ = 1;
  double time_shift_ = 0;
  int frame_skip_ = 1;

  int curr_index_ = 0;
  bool terminate_ = false;
  int frame_num_ = -1;

  int start_second_ = -1;

  vtr::storage::DataStreamReader<MessageType> reader_;

  typename rclcpp::Publisher<MessageType>::SharedPtr publisher_;
  /* \brief Publisher for simulated time (image timestamps) */
  rclcpp::Publisher<ClockMsg>::SharedPtr clock_publisher_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

template <class MessageType>
DataPlayer<MessageType>::DataPlayer(const std::string &data_dir,
                                    const std::string &stream_name,
                                    const std::string &topic, int replay_mode,
                                    int start_index, int stop_index,
                                    double delay_scale, double time_shift,
                                    int frame_skip)
    : Node("data_player"),
      replay_mode_(replay_mode),
      start_index_(start_index),
      stop_index_(stop_index),
      delay_scale_(delay_scale),
      time_shift_(time_shift),
      frame_skip_(frame_skip),
      curr_index_(start_index),
      reader_(data_dir, stream_name) {
  /// create publishers
  publisher_ = create_publisher<MessageType>(topic, 10);
  clock_publisher_ = create_publisher<ClockMsg>("clock", 10);

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
    if (terminate_) break;

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

template <class MessageType>
void DataPlayer<MessageType>::publish() {
  if (curr_index_ == stop_index_) {
    terminate_ = true;
    return;
  }

  auto message = reader_.readNextFromSeek();
  if (!message.get()) {
    terminate_ = true;
    return;
  }
  auto msg = message->template get<MessageType>();
  publisher_->publish(msg);

  std::cout << "Current index: " << curr_index_ << std::endl;
  curr_index_++;
}
