#include <gtest/gtest.h>
#include <filesystem>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "vtr_storage/data_stream_reader.hpp"
#include "vtr_storage/data_stream_writer.hpp"
#include "vtr_storage/message.hpp"

#include "test_msgs/msg/basic_types.hpp"

namespace fs = std::filesystem;

// sample code showing how to use the data streams
TEST(VTRStorage, readSeek) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.
  std::string stream_name = "test_stream";
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  // write data
  vtr::storage::DataStreamWriter<TestMsgT> writer(
      working_dir,
      stream_name);
  int64_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i*2000;
    auto message = vtr::storage::VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    writer.write(message);
  }

  vtr::storage::DataStreamReader<TestMsgT> reader(working_dir, stream_name);
  // testing for message contents and number of messages

  // read entire data with seek
  auto message = reader.readNextFromSeek();
  int count = 1;
  while (message.get()) {
    EXPECT_EQ(count*10, message->template get<TestMsgT>().float64_value);
    message = reader.readNextFromSeek();
    ++count;
  }
  EXPECT_EQ(count, 10+1);

  // read starting from index 5
  std::cout << "~~~~~~~~~~~~~~~~~~~"  << std::endl;
  reader.seekByIndex(5);
  message = reader.readNextFromSeek();
  count = 5;
  while (message.get()) {
    EXPECT_EQ(count*10, message->template get<TestMsgT>().float64_value);
    message = reader.readNextFromSeek();
    ++count;
  }
  EXPECT_EQ(count, 10+1);

  // read starting from timestamp 10000
  std::cout << "~~~~~~~~~~~~~~~~~~~"  << std::endl;
  reader.seekByTimestamp(16000);
  message = reader.readNextFromSeek();
  count = 8;
  while (message.get()) {
    EXPECT_EQ(count*10, message->template get<TestMsgT>().float64_value);
    message = reader.readNextFromSeek();
    ++count;
  }
  EXPECT_EQ(count, 10+1);

  reader.seekByIndex(11);
  EXPECT_TRUE(reader.readNextFromSeek().get() == nullptr);

  reader.seekByTimestamp(20001);
  EXPECT_TRUE(reader.readNextFromSeek().get() == nullptr);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
