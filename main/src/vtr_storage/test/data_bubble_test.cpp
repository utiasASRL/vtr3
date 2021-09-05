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
 * \file data_bubble_test.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>
#include <filesystem>

#include <vtr_storage/stream/data_bubble.hpp>
#include <vtr_storage/stream/data_stream_writer.hpp>
#include <vtr_storage/stream/message.hpp>

#include "test_msgs/msg/basic_types.hpp"
using TestMsg = test_msgs::msg::BasicTypes;

namespace fs = std::filesystem;

using namespace vtr::storage;

// sample code showing how to use the data streams
TEST(VTRStorage, dataBubble) {
  fs::path working_dir{fs::temp_directory_path() / "vtr_storage_test"};
  fs::remove_all(working_dir);  // make sure the directory is empty.

  TestMsg test_msg;
  VTRMessage message;

  // write
  DataStreamWriter<TestMsg> writer(working_dir);
  writer.open();
  int64_t timestamp;
  for (int i = 1; i <= 10; i++) {
    test_msg.float64_value = i * 10;
    timestamp = i * 2000;
    message = VTRMessage(test_msg);
    message.set_timestamp(timestamp);
    writer.write(message);
  }
  writer.close();

  auto reader = std::make_shared<DataStreamReader<TestMsg>>(working_dir);
  DataBubble bubble;

  // test retrieving by indices
  bubble.initialize(std::static_pointer_cast<DataStreamReaderBase>(reader));
  bubble.setIndices(2, 8);
  bubble.load();
  message = bubble.retrieve(3);  // 3 is local index of this bubble, which
                                 // translates to a global index of 2+3=5
  EXPECT_EQ(message.template get<TestMsg>().float64_value, 50);

  for (int global_index = 2; global_index <= 8; ++global_index) {
    auto message = bubble.retrieve(global_index - 2);
    EXPECT_EQ(message.template get<TestMsg>().float64_value, global_index * 10);
  }

  bubble.reset();

  // test retrieving by timestamps
  bubble.initialize(std::static_pointer_cast<DataStreamReaderBase>(reader));
  bubble.loadTime(8000);
  message = bubble.retrieveTime(8000);
  EXPECT_EQ(message.template get<TestMsg>().float64_value, 40);
  EXPECT_EQ(message.get_index(), 4);
  EXPECT_EQ(message.get_timestamp(), 8000);

  bubble.setTimeIndices(5000, 14500);
  bubble.load();
  for (int time = 6000; time <= 14000; time += 2000) {
    message = bubble.retrieveTime(time);
    EXPECT_EQ(message.template get<TestMsg>().float64_value, time / 200);
    EXPECT_EQ(message.get_index(), time / 2000);
    EXPECT_EQ(message.get_timestamp(), time);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
