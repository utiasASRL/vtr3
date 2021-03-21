#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

/// @brief Abstract class for sensors used in VTR
/// @param T1  the type of ROS2 message the sensor publishes
/// @param T2  the type of ROS2 message of the sensor calibration
template<class T1, class T2>
class VtrSensor {
 public:

  /// @brief Constructor. Initializes a node and publisher
  explicit VtrSensor(std::shared_ptr<rclcpp::Node> node, std::string sensor_topic_name)
      : sensor_okay_(true), node_(std::move(node)) {
    sensor_pub_ = node_->create_publisher<T1>(sensor_topic_name, 0);
  }

  /// @brief Default destructor
  ~VtrSensor() = default;

  /// @brief Gets sensor messages, preprocesses, visualizes, and publishes
  int run() {
    while (sensor_okay_ && rclcpp::ok()) {
      T1 sensor_msg;
      sensor_msg = grabSensorFrameBlocking();

      visualizeData();

      publishData(sensor_msg);

      rclcpp::spin_some(node_);
    }
    // Something went wrong with the sensor
    rclcpp::shutdown();
    return -1;
  }

 protected:
  /// @brief Get sensor message and preprocess if necessary
  virtual T1 grabSensorFrameBlocking() = 0;

  /// @brief Visualize sensor data
  virtual void visualizeData() = 0;

  /// @brief Use publisher to publish sensor data as ROS2 message
  virtual void publishData(T1 image) = 0;

  /// @brief True unless something went wrong with the sensor
  bool sensor_okay_;

  /// @brief The ROS2 node
  std::shared_ptr<rclcpp::Node> node_;

  /// @brief The ROS2 publisher
  std::shared_ptr<rclcpp::Publisher<T1>> sensor_pub_;
};
