
#include <vtr_sensors/VtrSensor.hpp>

#include <iostream>


VtrSensor::VtrSensor() : sensor_okay_(true){

  node_ = rclcpp::Node::make_shared("BumblebeeXb3");
  sensor_pub_ = node_->create_publisher<vtr_messages::msg::RigImages>("imagesss", 0);
}

int VtrSensor::run() {


  while(sensor_okay_ && rclcpp::ok()) {
    vtr_messages::msg::RigImages sensor_msg;       // todo: make more general?

    sensor_msg = grabSensorFrameBlocking();

#if 0       //todo: add visualization method
    visualizeData(sensor_message);
#endif

    publishData(sensor_msg);

    std::cout << "[debug] bottom of sensor run" << std::endl;

  }

  // Something went wrong with the camera
#if 0
  LOG(ERROR) << "Sensor isn't okay!!";
#endif
  return -1;
}
