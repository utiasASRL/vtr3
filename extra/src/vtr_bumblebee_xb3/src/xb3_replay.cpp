#include <chrono>
#include <filesystem>
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/opencv.hpp>

#include <vtr_bumblebee_xb3/xb3_replay.hpp>

namespace fs = std::filesystem;

Xb3Replay::Xb3Replay(const std::string &data_dir,
                     const std::string &stream_name, const std::string &topic,
                     const int qos)
    : Node("xb3_recorder"), reader_(data_dir, stream_name) {
  publisher_ = create_publisher<RigImages>(topic, qos);
  clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
}

/// @brief Replay XB3 stereo images from a rosbag2
int main(int argc, char *argv[]) {
  // Default path
  fs::path data_dir{fs::current_path() / "xb3_data"};
  std::string stream_name = "front_xb3";
  bool manual_scrub = false;

  bool use_original_timestamps = true;  // todo (Ben): add option as CLI arg
  double framerate = 16.0;

  // temp
  double delay_scale = 1.0;  // make playblack slower
  double time_shift = 0;     // shift time stamp for repeat
  int start_index = 1;
  int stop_index = 9999999;

  // User specified path
  if (argc >= 4) {
    data_dir = argv[1];
    stream_name = argv[2];
    std::istringstream(argv[3]) >> std::boolalpha >> manual_scrub;
    if (manual_scrub) {
      std::cout << "Manual replay selected. Press/hold any key to advance "
                   "image stream."
                << std::endl;
    }
    if (argc >= 5) start_index = atof(argv[4]);
    if (argc >= 6) stop_index = atof(argv[5]);
    if (argc >= 7) delay_scale = atof(argv[6]);
    if (argc >= 8) time_shift = atof(argv[7]);
  } else if (argc != 1) {
    throw std::invalid_argument("Wrong number of arguments provided!");
  }

  rclcpp::init(argc, argv);
  auto replay = Xb3Replay(data_dir.string(), stream_name, "xb3_images");

  replay.reader_.seekByIndex(start_index);

  std::string inputString;
  std::cout << "Enter to start!";
  std::cin.clear();
  std::getline(std::cin, inputString);

  int curr_index = start_index;

  uint64_t prev_stamp = 0;
  while (true) {
    if (!rclcpp::ok()) break;

    if (curr_index == stop_index) break;
    curr_index++;

    auto message = replay.reader_.readNextFromSeek();
    if (!message.get()) break;

    auto image = message->template get<RigImages>();

    if (!use_original_timestamps) {
      auto current_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
      for (auto &chan : image.channels) {
        for (auto &cam : chan.cameras) {
          cam.stamp.nanoseconds_since_epoch = current_ns;
        }
      }
    }

    // \todo yuchen Add necessary info for vtr to run, but they should not be
    // here
    image.name = "front_xb3";
    image.vtr_header.sensor_time_stamp = image.channels[0].cameras[0].stamp;
    image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch +=
        1e12 * time_shift;

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock.sec =
        image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch / 1e9;
    clock_msg.clock.nanosec =
        image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch % (long)1e9;
    // Publish time to /clock for nodes using sim_time
    replay.clock_publisher_->publish(clock_msg);

    std::cout << "Publishing image with time stamp: "
              << image.vtr_header.sensor_time_stamp.nanoseconds_since_epoch
              << " and index is " << curr_index << std::endl;

    // Publish message for use with offline tools
    replay.publisher_->publish(image);
    // Add a delay so that the image publishes at roughly the true rate.
    // std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Visualization
    auto left = image.channels[0].cameras[0];
    auto right = image.channels[0].cameras[1];

    // Replays images based on their timestamps. Converts nanoseconds to
    // milliseconds
    if (manual_scrub) {
      // cv::waitKey(0);
      std::cin.clear();
      std::getline(std::cin, inputString);
    } else {
      if (prev_stamp != 0) {
        double delay = use_original_timestamps
                           ? (left.stamp.nanoseconds_since_epoch - prev_stamp) *
                                 std::pow(10, -6)
                           : 1000.0 / framerate;
        delay *= delay_scale;
        cv::waitKey(delay);
      }
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
    cv::Mat cv_left =
        cv::Mat(left.height, left.width, outputmode, (void *)left.data.data());
    cv::imshow(image.channels[0].name + "/left", cv_left);
    right.data.resize(datasize);
    cv::Mat cv_right = cv::Mat(right.height, right.width, outputmode,
                               (void *)right.data.data());
    cv::imshow(image.channels[0].name + "/right", cv_right);
  }
  rclcpp::shutdown();
}
