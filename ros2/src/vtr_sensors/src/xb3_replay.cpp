#include <vtr_storage/data_stream_reader.hpp>
#include <vtr_messages/msg/rig_images.hpp>

#include <opencv2/opencv.hpp>

/// @brief Replay XB3 stereo images from a rosbag2
int main(int argc, char *argv[]) {
  using RigImages = vtr_messages::msg::RigImages;

  //Default path
  std::string data_dir = "/home/ben/test/rosbag2_image_demo";
  std::string stream_name = "front_xb3";

  //User specified path
  if (argc == 3) {
    data_dir = argv[1];
    stream_name = argv[2];
  }

  vtr::storage::DataStreamReader<RigImages> reader(
      data_dir,
      stream_name);

  auto image_bag = reader.readAtIndexRange(1, 9999);    // todo: figure out if better way to read whole bag
  uint64_t prev_stamp = 0;
  for (const auto &message : *image_bag) {

    auto image = message->template get<RigImages>();
    auto left = image.channels[0].cameras[0];
    auto right = image.channels[0].cameras[1];

    // Replays images based on their timestamps. Converts nanoseconds to milliseconds
    if (prev_stamp != 0) {
      cv::waitKey((left.stamp.nanoseconds_since_epoch - prev_stamp) * std::pow(10, -6));
    }
    prev_stamp = left.stamp.nanoseconds_since_epoch;

    // Get image parameters from left camera and assume right is the same
    int outputmode = -1;
    int datasize = 0;
    if (left.encoding == "bgr8") {
      datasize = left.height * left.width * 3;
      outputmode = CV_8UC3;
    } else if (left.encoding == "mono8") {
      datasize = left.height * left.width;
      outputmode = CV_8UC1;
    }

    // Create OpenCV images to be shown
    left.data.resize(datasize);
    cv::Mat cv_left = cv::Mat(left.height, left.width, outputmode, (void *) left.data.data());
    cv::imshow(image.channels[0].name + "/left", cv_left);
    right.data.resize(datasize);
    cv::Mat cv_right = cv::Mat(right.height, right.width, outputmode, (void *) right.data.data());
    cv::imshow(image.channels[0].name + "/right", cv_right);
  }

  return 0;
}
