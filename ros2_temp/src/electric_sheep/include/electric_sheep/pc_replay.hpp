#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using ClockMsg = rosgraph_msgs::msg::Clock;

class PCReplay : public rclcpp::Node {
public:
  PCReplay(const std::string &data_dir, const std::string &stream_name,
           const std::string &topic, int qos = 10);

  vtr::storage::DataStreamReader<PointCloudMsg> reader_;

  rclcpp::Publisher<PointCloudMsg>::SharedPtr publisher_;
  /* \brief Publisher for simulated time (image timestamps) */
  rclcpp::Publisher<ClockMsg>::SharedPtr clock_publisher_;
};