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
TEST(VTRStorage, readWriteAppend) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  // also test that creating new bags deletes old data
  vtr::storage::DataStreamWriter<TestMsgT> writer(
      working_dir,
      "test_stream");
  writer.open();
  test_msg.float64_value = 123;
  writer.write(vtr::storage::VTRMessage(test_msg));
  writer.close();
  writer.open();  // should delete the 123 message
  int64_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i*2000;
    auto message = vtr::storage::VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    int32_t index_return = writer.write(message);
    EXPECT_EQ(i, index_return);
    // std::cout << index_return << std::endl;
  }
  writer.close();

  // append
  vtr::storage::DataStreamWriter<TestMsgT> writer2(
      working_dir,
      "test_stream", true);
  writer2.open();
  for (int i = 11; i <= 20; i++) {
    test_msg.float64_value = i * 10;
    int32_t index_return = writer2.write(vtr::storage::VTRMessage(test_msg));
    EXPECT_EQ(i, index_return);
    // std::cout << index_return << std::endl;
  }
  // writer2.close();

  // read
  vtr::storage::DataStreamReader<TestMsgT> reader(
      working_dir,
      "test_stream");
  test_msg.float64_value =
      reader.readAtIndex(5)->template get<TestMsgT>().float64_value;
  EXPECT_EQ(test_msg.float64_value, 50);
  // std::cout << test_msg.float64_value << std::endl;

  // std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  auto bag_message_vector = reader.readAtIndexRange(1, 9);
  int count = 1;
  for (auto message : *bag_message_vector) {
    test_msg.float64_value = message->template get<TestMsgT>().float64_value;
    EXPECT_EQ(test_msg.float64_value, count*10);
    count++;
    // std::cout << message->template get<TestMsgT>().float64_value << std::endl;
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
    EXPECT_EQ(index_return, index);
    EXPECT_EQ(test_msg.float64_value, index*10);
    // std::cout << "Written index: " << index_return
    //           << ", Read: " << test_msg.float64_value
    //           << ", Read index: " << index << std::endl;
  }
  // writer auto closes when it goes out of scope
  // writer2.close(); // when writer closes, it writes the metadata.yaml

  // ~~~~~~~~~~~~~
  // test reading by timestamp
  test_msg.float64_value = reader.readAtTimestamp(8000)->template get<TestMsgT>().float64_value;
  EXPECT_EQ(test_msg.float64_value, 40);
  // std::cout << test_msg.float64_value << std::endl; // should output 40 (vertex 4)

  // std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  bag_message_vector = reader.readAtTimestampRange(5000, 14500);
  // should output 30 to 70 (vertex 3 to 7)
  count = 3;
  for (auto message : *bag_message_vector) {
    test_msg.float64_value = message->template get<TestMsgT>().float64_value;
    EXPECT_EQ(test_msg.float64_value, count*10);
    count++;
    // std::cout << message->template get<TestMsgT>().float64_value << std::endl;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
