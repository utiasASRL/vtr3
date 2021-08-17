#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointFieldMsg = sensor_msgs::msg::PointField;
using ClockMsg = rosgraph_msgs::msg::Clock;

class PCReplay : public rclcpp::Node {
 public:
  PCReplay(const std::string &data_dir, const std::string &stream_name,
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

  vtr::storage::DataStreamReader<PointCloudMsg> reader_;

  rclcpp::Publisher<PointCloudMsg>::SharedPtr publisher_;
  /* \brief Publisher for simulated time (image timestamps) */
  rclcpp::Publisher<ClockMsg>::SharedPtr clock_publisher_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};