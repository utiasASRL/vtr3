#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vtr_messages/msg/rig_images.hpp>

#include <memory>

class VtrSensor {
 public:

  VtrSensor();

  ~VtrSensor() = default;

  int run();


 protected:

  virtual vtr_messages::msg::RigImages grabSensorFrameBlocking() = 0;      // todo: make more general?

  virtual void publishData(vtr_messages::msg::RigImages image) = 0;

  bool sensor_okay_;

  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<rclcpp::Publisher<vtr_messages::msg::RigImages>> sensor_pub_;

};

