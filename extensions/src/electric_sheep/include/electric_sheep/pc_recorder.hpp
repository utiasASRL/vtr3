#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vtr_storage/data_stream_writer.hpp>

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

/** \brief Subscribes to XB3 images and calibration and saves both to rosbag2s
 */
class PCRecorder : public rclcpp::Node {
 public:
  /** \brief Constructor */
  PCRecorder(const std::string &data_dir, const std::string &stream_name);

 private:
  /** \brief Callback for XB3 image msgs */
  void pointcloudCallback(PointCloudMsg::SharedPtr msg);

  /** \brief Subscriber for images */
  rclcpp::Subscription<PointCloudMsg>::SharedPtr data_subscription_;

  /** \brief Image writer */
  vtr::storage::DataStreamWriter<PointCloudMsg> writer_;
};