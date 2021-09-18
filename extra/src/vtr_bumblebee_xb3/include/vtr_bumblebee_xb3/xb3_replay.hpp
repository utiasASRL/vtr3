#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <vtr_messages/msg/rig_image_calib.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/rig_calibration.hpp>

#include <vtr_storage/stream/data_stream_reader.hpp>
#include <vtr_storage/stream/data_stream_writer.hpp>

using RigImageCalib = vtr_messages::msg::RigImageCalib;
using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;

class Xb3Replay : public rclcpp::Node {
 public:
  Xb3Replay(const std::string &data_dir,
            const std::string &stream_name, const std::string &topic, int qos = 10);

  vtr::storage::DataStreamReader<RigImages, RigCalibration> reader_;
  // vtr::storage::DataStreamReader<RigImages, RigCalibration> calib_reader_;

  RigCalibration calibration_msg_;
  rclcpp::Publisher<RigImageCalib>::SharedPtr publisher_;
  /* \brief Publisher for simulated time (image timestamps) */
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
};