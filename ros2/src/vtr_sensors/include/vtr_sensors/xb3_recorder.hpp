#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/rig_calibration.hpp>
#include <vtr_storage/data_stream_writer.hpp>

using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;

class Xb3Recorder : public rclcpp::Node {
 public:
  Xb3Recorder(const std::string &data_dir, const std::string &stream_name);

 private:
  void imageCallback(RigImages::SharedPtr msg);

  void calibCallback(RigCalibration::SharedPtr msg);

  rclcpp::Subscription<RigImages>::SharedPtr data_subscription_;

  rclcpp::Subscription<RigCalibration>::SharedPtr calib_subscription_;

  vtr::storage::DataStreamWriter<RigImages> writer_;

  vtr::storage::DataStreamWriter<RigCalibration> calib_writer_;
};