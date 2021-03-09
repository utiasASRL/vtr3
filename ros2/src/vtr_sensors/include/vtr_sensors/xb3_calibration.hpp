#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/srv/get_rig_calibration.hpp>
#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_storage/data_stream_writer.hpp>

using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;
using GetRigCalibration = vtr_messages::srv::GetRigCalibration;

class Xb3Calibration {
 public:
  Xb3Calibration(const std::shared_ptr<rclcpp::Node> node,
                 const std::string &data_dir, const std::string &stream_name);

 private:
  void _calibrationCallback(
      const std::shared_ptr<GetRigCalibration::Request> request,
      std::shared_ptr<GetRigCalibration::Response> response);

  vtr::storage::DataStreamReader<RigImages, RigCalibration> reader_;

  const std::shared_ptr<rclcpp::Node> node_;

  RigCalibration calibration_msg_;

  rclcpp::Service<GetRigCalibration>::SharedPtr calibration_srv_;
};
