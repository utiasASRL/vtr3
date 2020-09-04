#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <vtr_messages/msg/rig_images.hpp>

/// @brief Abstract class for sensors used in VTR
/// @param T  the type of ROS2 message the sensor publishes
template <class T>
class VtrSensor {
 public:

  /// @brief Constructor. Initializes a node and publisher
  explicit VtrSensor(std::shared_ptr<rclcpp::Node> node, std::string topic_name)
      : sensor_okay_(true), node_(std::move(node)) {
    sensor_pub_ = node_->create_publisher<T>(topic_name, 0);
  }

  /// @brief Default destructor
  ~VtrSensor() = default;

  /// @brief Gets sensor messages, preprocesses, visualizes, and publishes
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
  /// @brief Get sensor message and preprocess if necessary
  virtual T grabSensorFrameBlocking() = 0;

  /// @brief Visualize sensor data
  virtual void visualizeData() = 0;

  /// @brief Use publisher to publish sensor data as ROS2 message
  virtual void publishData(T image) = 0;

  /// @brief True unless something went wrong with the sensor
  bool sensor_okay_;

  /// @brief The ROS2 node
  std::shared_ptr<rclcpp::Node> node_;

  /// @brief The ROS2 publisher
  std::shared_ptr<rclcpp::Publisher<T>> sensor_pub_;
};
