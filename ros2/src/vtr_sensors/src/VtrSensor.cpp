
#include <vtr_sensors/VtrSensor.hpp>

#include <iostream>


VtrSensor::VtrSensor() : sensor_okay_(true){

}

int VtrSensor::run() {


  while(sensor_okay_) {
    sensor_msgs__msg__Image sensor_msg;       // todo: make more general?

    sensor_msg = grabSensorFrameBlocking();

#if 0
    visualizeData(sensor_message);
#endif

#if 0
    // publish the data
    auto &topic = config_.data_comms.topic;
    publishData(topic, sensor_message.baseMessage());
    publishData(topic + "_logger", sensor_message.baseMessage());
#endif      // todo: publish as ROS2 message



    std::cout << "[debug] bottom of sensor run" << std::endl;

  }

  // Something went wrong with the camera
#if 0
  LOG(ERROR) << "Sensor isn't okay!!";
#endif
  return -1;
}
