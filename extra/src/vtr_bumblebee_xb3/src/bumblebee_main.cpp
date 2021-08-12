
#include <vtr_bumblebee_xb3/bumblebee_xb3.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("BumblebeeXb3");

  // Get configuration from YAML file
  vtr::sensors::xb3::Xb3Configuration config;
  node->declare_parameter("camera_model", "XB3");
  node->declare_parameter("camera_name", "front_xb3");
  node->declare_parameter("rectified_image_size_height", 384);
  node->declare_parameter("rectified_image_size_width", 512);
  node->declare_parameter("packet_multiplier", 0);
  node->declare_parameter("output_gray", false);
  node->declare_parameter("show_rectified_images", false);
  node->declare_parameter("show_raw_images", false);

  node->get_parameter("camera_model", config.camera_model);
  node->get_parameter("camera_name", config.camera_name);
  node->get_parameter("rectified_image_size_height",
                      config.rectified_image_size.height);
  node->get_parameter("rectified_image_size_width",
                      config.rectified_image_size.width);
  node->get_parameter("packet_multiplier", config.packet_multiplier);
  node->get_parameter("output_gray", config.output_gray);
  node->get_parameter("show_rectified_images", config.show_rectified_images);
  node->get_parameter("show_raw_images", config.show_raw_images);

  try {
    vtr::sensors::xb3::BumblebeeXb3 cameraDriver(node, config);
    return cameraDriver.run();
  } catch (const std::exception &e) {
    return -3;
  }
}