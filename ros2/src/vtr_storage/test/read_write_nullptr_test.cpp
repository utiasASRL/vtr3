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
TEST(VTRStorage, readWriteNullptr) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.
  std::string stream_name = "test_stream";
  using TestMsgT = test_msgs::msg::BasicTypes;
  TestMsgT test_msg;

  // write data
  vtr::storage::DataStreamWriter<TestMsgT> writer(working_dir, stream_name);
  int64_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i * 2000;
    auto message = vtr::storage::VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    writer.write(message);
  }

  vtr::storage::DataStreamReader<TestMsgT> reader(working_dir, stream_name);

  // reading singular index/timestamp
  EXPECT_TRUE(reader.readAtIndex(0).get() == nullptr);
  EXPECT_TRUE(reader.readAtIndex(1).get() != nullptr);
  EXPECT_TRUE(reader.readAtIndex(10).get() != nullptr);
  EXPECT_TRUE(reader.readAtIndex(11).get() == nullptr);

  EXPECT_TRUE(reader.readAtTimestamp(1999).get() == nullptr);
  EXPECT_TRUE(reader.readAtTimestamp(2000).get() != nullptr);
  EXPECT_TRUE(reader.readAtTimestamp(10000).get() != nullptr);
  EXPECT_TRUE(reader.readAtTimestamp(10001).get() == nullptr);

  // reading ranges
  EXPECT_EQ(reader.readAtIndexRange(1, 10)->size(), (unsigned)10);
  EXPECT_EQ(reader.readAtIndexRange(0, 10)->size(), (unsigned)10);
  EXPECT_EQ(reader.readAtIndexRange(9, 11)->size(), (unsigned)2);
  EXPECT_EQ(reader.readAtIndexRange(10, 12)->size(), (unsigned)1);
  EXPECT_EQ(reader.readAtIndexRange(11, 12)->size(), (unsigned)0);

  EXPECT_EQ(reader.readAtTimestampRange(2000, 20000)->size(), (unsigned)10);
  EXPECT_EQ(reader.readAtTimestampRange(1000, 20000)->size(), (unsigned)10);
  EXPECT_EQ(reader.readAtTimestampRange(18000, 22000)->size(), (unsigned)2);
  EXPECT_EQ(reader.readAtTimestampRange(20000, 22000)->size(), (unsigned)1);
  EXPECT_EQ(reader.readAtTimestampRange(20001, 22000)->size(), (unsigned)0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
