#include <gtest/gtest.h>
#include <filesystem>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "vtr_storage/data_bubble.hpp"
#include "vtr_storage/data_stream_writer.hpp"
#include "vtr_storage/message.hpp"

#include "test_msgs/msg/basic_types.hpp"

namespace fs = std::filesystem;

// sample code showing how to use the data streams
TEST(VTRStorage, dataBubble) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.

  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;
  vtr::storage::VTRMessage message;

  // write
  vtr::storage::DataStreamWriter<TestMsgT> writer(working_dir);
  writer.open();
  int64_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i * 2000;
    message = vtr::storage::VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    writer.write(message);
  }
  writer.close();

  auto reader =
      std::make_shared<vtr::storage::DataStreamReader<TestMsgT>>(working_dir);
  vtr::storage::DataBubble bubble;

  // test retrieving by indices
  bubble.initialize(
      std::static_pointer_cast<vtr::storage::DataStreamReaderBase>(reader));
  bubble.setIndices(2, 8);
  bubble.load();
  message = bubble.retrieve(3);  // 3 is local index of this bubble, which
                                 // translates to a global index of 2+3=5
  EXPECT_EQ(message.template get<TestMsgT>().float64_value, 50);
  // std::cout << message.template get<TestMsgT>().float64_value << std::endl;
  for (int global_index = 2; global_index <= 8; ++global_index) {
    auto message = bubble.retrieve(global_index - 2);
    EXPECT_EQ(message.template get<TestMsgT>().float64_value,
              global_index * 10);
  }

  // ~~~~~~~~~~~~~
  // test retrieving by timestamps
  bubble.reset();  // ToDo: still fixing the (probably is a) bug where the
                   // writer isn't closing properly
  bubble.initialize(
      std::static_pointer_cast<vtr::storage::DataStreamReaderBase>(reader));
  bubble.loadTime(8000);
  message = bubble.retrieveTime(8000);
  EXPECT_EQ(message.template get<TestMsgT>().float64_value, 40);
  EXPECT_EQ(message.get_index(), 4);
  EXPECT_EQ(message.get_timestamp(), 8000);
  // std::cout << "Message: " << message.template get<TestMsgT>().float64_value
  //           << ", index: " << message.get_index()
  //           << ", timestamp: " << message.get_timestamp() << std::endl;

  // std::cout << "~~~~~~~~~~~~~~~~~~~~" << std::endl;
  bubble.setTimeIndices(5000, 14500);
  bubble.load();
  for (int time = 6000; time <= 14000; time += 2000) {
    message = bubble.retrieveTime(time);
    EXPECT_EQ(message.template get<TestMsgT>().float64_value, time / 200);
    EXPECT_EQ(message.get_index(), time / 2000);
    EXPECT_EQ(message.get_timestamp(), time);
    // std::cout << "Message: " << message.template
    // get<TestMsgT>().float64_value
    //           << ", index: " << message.get_index()
    //           << ", timestamp: " << message.get_timestamp() << std::endl;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
