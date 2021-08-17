#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include <vtr_messages/msg/rig_calibration.hpp>
#include "vtr_logging/logging_init.hpp"
#include "vtr_storage/data_bubble.hpp"
#include "vtr_storage/data_stream_reader.hpp"
#include "vtr_storage/data_stream_writer.hpp"
#include "vtr_storage/message.hpp"

#include "test_msgs/msg/basic_types.hpp"

// sample code showing how to write/fetch calibration
int main() {
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  std::string base_url =
      "/home/daniel/test/ROS2BagFileParsing/dev_ws/test_rosbag2_writer_api_bag";
  std::string stream_name = "test_stream";

  // write data
  vtr::storage::DataStreamWriter<TestMsgT> writer(base_url, stream_name);
  int32_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i * 2000;
    auto message = vtr::storage::VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    writer.write(message);
  }

  // read entire data with seek
  vtr::storage::DataStreamReader<TestMsgT> reader(base_url, stream_name);
  auto message = reader.readNextFromSeek();
  while (message.get()) {
    std::cout << message->template get<TestMsgT>().float64_value << std::endl;
    message = reader.readNextFromSeek();
  }

  // read starting from index 5
  std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
  reader.seekByIndex(5);
  message = reader.readNextFromSeek();
  while (message.get()) {
    std::cout << message->template get<TestMsgT>().float64_value << std::endl;
    message = reader.readNextFromSeek();
  }

  // read starting from timestamp 10000
  std::cout << "~~~~~~~~~~~~~~~~~~~" << std::endl;
  reader.seekByTimestamp(20001);
  message = reader.readNextFromSeek();
  while (message.get()) {
    std::cout << message->template get<TestMsgT>().float64_value << std::endl;
    message = reader.readNextFromSeek();
  }
  std::cout << "done" << std::endl;
}