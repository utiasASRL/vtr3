
#include <utility>
#include <vtr_sensors/VtrSensor.hpp>

VtrSensor::VtrSensor(std::shared_ptr<rclcpp::Node> node)
    : sensor_okay_(true), node_(std::move(node)) {
  sensor_pub_ =
      node_->create_publisher<vtr_messages::msg::RigImages>("imagesss", 0);
}

int VtrSensor::run() {
  while (sensor_okay_ && rclcpp::ok()) {
    vtr_messages::msg::RigImages sensor_msg;  // todo: make more general?

    sensor_msg = grabSensorFrameBlocking();

    visualizeData();

    publishData(sensor_msg);
  }

  // Something went wrong with the camera
#if 0
  LOG(ERROR) << "Sensor isn't okay!!";
#endif
  return -1;
}
