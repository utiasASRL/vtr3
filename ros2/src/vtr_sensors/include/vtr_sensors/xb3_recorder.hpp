#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_storage/data_stream_writer.hpp>

using RigImages = vtr_messages::msg::RigImages;

class Xb3Recorder : public rclcpp::Node {
 public:
  Xb3Recorder(const std::string &data_dir, const std::string &stream_name);

 private:
  void imageCallback(RigImages::SharedPtr msg);

  rclcpp::Subscription<RigImages>::SharedPtr subscription_;

  vtr::storage::DataStreamWriter<RigImages> writer_;
};