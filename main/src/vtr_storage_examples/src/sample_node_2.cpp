#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "vtr_logging/logging_init.hpp"
#include "vtr_storage/data_stream_reader.hpp"
#include "vtr_storage/data_stream_writer.hpp"
#include "vtr_storage/message.hpp"

#include "test_msgs/msg/basic_types.hpp"

// sample code showing how to use the data streams
int main() {
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  // create
  vtr::storage::DataStreamWriter<TestMsgT> writer(
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag",
      "test_stream");
  writer.open();
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    int32_t index_return = writer.write(vtr::storage::VTRMessage(test_msg));
    std::cout << index_return << std::endl;
  }
  writer.close();

  // append
  vtr::storage::DataStreamWriter<TestMsgT> writer2(
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag",
      "test_stream", true);
  writer2.open();
  for (int i = 11; i <= 20; i++) {
    test_msg.float64_value = i * 10;
    int32_t index_return = writer2.write(vtr::storage::VTRMessage(test_msg));
    std::cout << index_return << std::endl;
  }
  // writer2.close();

  // read
  vtr::storage::DataStreamReader<TestMsgT> reader(
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag",
      "test_stream");
  test_msg.float64_value =
      reader.readAtIndex(5)->template get<TestMsgT>().float64_value;
  std::cout << test_msg.float64_value << std::endl;

  std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  auto bag_message_vector = reader.readAtIndexRange(1, 9);
  for (auto message : *bag_message_vector) {
    std::cout << message->template get<TestMsgT>().float64_value << std::endl;
  }

  writer2.close();
  // append and read
  writer2.open();
  for (int i = 21; i <= 30; i++) {
    test_msg.float64_value = i * 10;
    int32_t index_return = writer2.write(vtr::storage::VTRMessage(test_msg));
    auto anytype_msg = reader.readAtIndex(i);
    test_msg.float64_value =
        anytype_msg->template get<TestMsgT>().float64_value;
    auto index = anytype_msg->get_index();
    std::cout << "Written index: " << index_return
              << ", Read: " << test_msg.float64_value
              << ", Read index: " << index << std::endl;
  }
  // writer auto closes when it goes out of scope
  // writer2.close(); // when writer closes, it writes the metadata.yaml
}