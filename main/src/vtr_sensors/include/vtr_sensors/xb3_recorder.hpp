#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_messages/msg/rig_calibration.hpp>
#include <vtr_messages/srv/get_rig_calibration.hpp>
#include <vtr_storage/data_stream_writer.hpp>

using RigImages = vtr_messages::msg::RigImages;
using RigCalibration = vtr_messages::msg::RigCalibration;
using GetRigCalibration = vtr_messages::srv::GetRigCalibration;

/** \brief Subscribes to XB3 images and calibration and saves both to rosbag2s */
class Xb3Recorder : public rclcpp::Node {
 public:
  /** \brief Constructor */
  Xb3Recorder(const std::string &data_dir, const std::string &stream_name);

 private:
  /** \brief Callback for XB3 image msgs */
  void _imageCallback(RigImages::SharedPtr msg);

  /** \brief Fetch image calibration data from a service */
  void _fetchCalibration();

  /** \brief Subscriber for images */
  rclcpp::Subscription<RigImages>::SharedPtr data_subscription_;

  /** \brief Whether we've saved calibration already */
  bool calibration_recorded_ = false;

  /** \brief Image writer */
  vtr::storage::DataStreamWriter<RigImages> writer_;

  /** \brief Calibration writer */
  vtr::storage::DataStreamWriter<RigCalibration> calib_writer_;

  /** \brief Client to call calibration service */
  rclcpp::Client<GetRigCalibration>::SharedPtr rig_calibration_client_;

  /** \brief Calibration for the stereo rig */
  RigCalibration::SharedPtr rig_calibration_;
};