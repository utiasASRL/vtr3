#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vtr_messages/msg/rig_images.hpp>

using RigImages = vtr_messages::msg::RigImages;

class Xb3Recorder : public rclcpp::Node {
 public:
  Xb3Recorder(std::string data_dir, std::string stream_name);

 private:
  void imageCallback(RigImages::SharedPtr msg) const;

  rclcpp::Subscription<RigImages>::SharedPtr subscription_;

  std::string data_dir_;

  std::string stream_name_;

};