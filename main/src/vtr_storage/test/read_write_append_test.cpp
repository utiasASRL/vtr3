// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file read_write_append_test.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>
#include <filesystem>

#include <vtr_storage/stream/data_stream_reader.hpp>
#include <vtr_storage/stream/data_stream_writer.hpp>
#include <vtr_storage/stream/message.hpp>

#include "test_msgs/msg/basic_types.hpp"
using TestMsg = test_msgs::msg::BasicTypes;

namespace fs = std::filesystem;

using namespace vtr::storage;

// sample code showing how to use the data streams
TEST(VTRStorage, readWriteAppend) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.

  TestMsg test_msg;

  // also test that creating new bags deletes old data
  DataStreamWriter<TestMsg> writer(working_dir, "test_stream");
  writer.open();
  test_msg.float64_value = 123;
  writer.write(VTRMessage(test_msg));
  writer.close();

  writer.open();  // should delete the 123 message
  int64_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i * 2000;
    auto message = VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    int32_t index_return = writer.write(message);
    EXPECT_EQ(i, index_return);
  }
  writer.close();

  // append
  DataStreamWriter<TestMsg> writer2(working_dir, "test_stream", true);
  writer2.open();
  for (int i = 11; i <= 20; i++) {
    test_msg.float64_value = i * 10;
    int32_t index_return = writer2.write(VTRMessage(test_msg));
    EXPECT_EQ(i, index_return);
  }
  // writer2.close();

  // read while writer2 is still open
  DataStreamReader<TestMsg> reader(working_dir, "test_stream");
  test_msg.float64_value =
      reader.readAtIndex(5)->template get<TestMsg>().float64_value;
  EXPECT_EQ(test_msg.float64_value, 50);

  auto bag_message_vector = reader.readAtIndexRange(1, 9);
  int count = 1;
  for (auto message : *bag_message_vector) {
    test_msg.float64_value = message->template get<TestMsg>().float64_value;
    EXPECT_EQ(test_msg.float64_value, count * 10);
    count++;
  }

  writer2.close();  // when writer closes, it writes the metadata.yaml

  // append and read
  writer2.open();
  for (int i = 21; i <= 30; i++) {
    test_msg.float64_value = i * 10;
    int32_t index_return = writer2.write(VTRMessage(test_msg));
    auto anytype_msg = reader.readAtIndex(i);
    test_msg.float64_value = anytype_msg->template get<TestMsg>().float64_value;
    auto index = anytype_msg->get_index();
    EXPECT_EQ(index_return, index);
    EXPECT_EQ(test_msg.float64_value, index * 10);
  }
  // writer auto closes when it goes out of scope

  // test reading by timestamp
  test_msg.float64_value =
      reader.readAtTimestamp(8000)->template get<TestMsg>().float64_value;
  EXPECT_EQ(test_msg.float64_value, 40);

  bag_message_vector = reader.readAtTimestampRange(5000, 14500);
  count = 3;
  for (auto message : *bag_message_vector) {
    test_msg.float64_value = message->template get<TestMsg>().float64_value;
    EXPECT_EQ(test_msg.float64_value, count * 10);
    count++;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
