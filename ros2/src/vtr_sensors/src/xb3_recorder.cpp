#include <utility>
#include <vtr_sensors/xb3_recorder.hpp>
#include <vtr_storage/data_stream_writer.hpp>

Xb3Recorder::Xb3Recorder(std::string data_dir, std::string stream_name)
    : Node("xb3_recorder"), data_dir_(std::move(data_dir)), stream_name_(std::move(stream_name)) {

  subscription_ =
      this->create_subscription<RigImages>("xb3_images",
                                           10,
                                           std::bind(&Xb3Recorder::imageCallback, this, std::placeholders::_1));
}

void Xb3Recorder::imageCallback(const RigImages::SharedPtr msg) const {

  vtr::storage::DataStreamWriter<RigImages> writer(data_dir_, stream_name_);

  writer.open();
  writer.write(vtr::storage::VTRMessage(*msg));
  writer.close();
}

/// @brief Record XB3 stereo images to a rosbag2
int main(int argc, char *argv[]) {
  //Default path
  std::string data_dir = "/home/ben/test/rosbag2_image_demo";
  std::string stream_name = "front_xb3";

  //User specified path
  if (argc == 3) {
    data_dir = argv[1];
    stream_name = argv[2];
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Xb3Recorder>(data_dir, stream_name));
  rclcpp::shutdown();
  return 0;
}