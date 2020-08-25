#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>

#include <memory>

class VtrSensor {
 public:

  VtrSensor();

  ~VtrSensor() = default;

  int run();


 protected:

  virtual sensor_msgs__msg__Image grabSensorFrameBlocking() = 0;      // todo: make more general?

  bool sensor_okay_;

};

