#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "vtr_logging/logging_init.hpp"
#include "vtr_storage/data_bubble.hpp"
#include "vtr_storage/data_stream_reader.hpp"
#include "vtr_storage/data_stream_writer.hpp"
#include "vtr_storage/message.hpp"

#include "test_msgs/msg/basic_types.hpp"

// sample code showing how to use the data streams
int main() {
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  vtr::storage::DataStreamWriter<TestMsgT> writer(
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/"
      "test_rosbag2_writer_api_bag");

  writer.open();
  int64_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i * 2000;
    auto message = vtr::storage::VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    writer.write(message);
  }
  writer.close();

  auto reader = std::make_shared<vtr::storage::DataStreamReader<TestMsgT>>(
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/"
      "test_rosbag2_writer_api_bag");
  test_msg.float64_value =
      reader->readAtTimestamp(8000)->template get<TestMsgT>().float64_value;
  std::cout << test_msg.float64_value
            << std::endl;  // should output 40 (vertex 4)

  std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  auto bag_message_vector = reader->readAtTimestampRange(5000, 14500);
  // should output 30 to 70 (vertex 3 to 7)
  for (auto message : *bag_message_vector) {
    std::cout << message->template get<TestMsgT>().float64_value << std::endl;
  }

  vtr::storage::DataBubble bubble;
  bubble.initialize(
      std::static_pointer_cast<vtr::storage::DataStreamReaderBase>(reader));
  bubble.loadTime(8000);
  auto message = bubble.retrieveTime(8000);
  std::cout << "Message: " << message.template get<TestMsgT>().float64_value
            << ", index: " << message.get_index()
            << ", timestamp: " << message.get_timestamp() << std::endl;

  std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  bubble.unload();
  bubble.setTimeIndices(5000, 14500);
  bubble.load();
  // bubble.loadTime(5000, 14500);
  for (int time = 6000; time <= 14000; time += 2000) {
    auto message = bubble.retrieveTime(time);
    std::cout << "Message: " << message.template get<TestMsgT>().float64_value
              << ", index: " << message.get_index()
              << ", timestamp: " << message.get_timestamp() << std::endl;
  }

  // bubble.setIndices(2, 8);
  // bubble.load();
  // auto message = bubble.retrieve(3);  // 3 is local index of this bubble,
  // which
  //                                     // translates to a global index of
  //                                     2+3=5
  // std::cout << message.template get<TestMsgT>().float64_value << std::endl;

  // test_msg.float64_value = reader.readAtIndex(5)->float64_value;
  // std::cout << test_msg.float64_value << std::endl;

  // std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  // auto bag_message_vector = reader.readAtIndexRange(3, 8);
  // for (auto message : *bag_message_vector) {
  //   std::cout << message->float64_value << std::endl;
  // }
}