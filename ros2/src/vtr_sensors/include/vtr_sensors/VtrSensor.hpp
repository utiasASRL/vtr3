#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <vtr_messages/msg/rig_images.hpp>

/// @brief Abstract class for sensors used in VTR
/// @param T  the type of ROS2 message the sensor publishes
template <class T>
class VtrSensor {
 public:
  explicit VtrSensor(std::shared_ptr<rclcpp::Node> node, std::string topic_name)
      : sensor_okay_(true), node_(std::move(node)) {
    sensor_pub_ = node_->create_publisher<T>(topic_name, 0);
  }

  ~VtrSensor() = default;

  int run() {
    while (sensor_okay_ && rclcpp::ok()) {
      T sensor_msg;
      sensor_msg = grabSensorFrameBlocking();

      visualizeData();

      publishData(sensor_msg);
    }
    // Something went wrong with the sensor
    return -1;
  }

 protected:
  virtual T grabSensorFrameBlocking() = 0;

  virtual void visualizeData() = 0;

  virtual void publishData(T image) = 0;

  bool sensor_okay_;

  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<rclcpp::Publisher<T>> sensor_pub_;
};
