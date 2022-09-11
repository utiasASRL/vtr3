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
 * \file test_storage_accessor.hpp
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gmock/gmock.h>

#include <chrono>
#include <filesystem>
#include <thread>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/logging_macros.h"
#include "rcutils/snprintf.h"

#include "vtr_storage/stream/data_stream_accessor.hpp"

#include "std_msgs/msg/string.hpp"

using namespace ::testing;  // NOLINT
using namespace std::chrono_literals;
using namespace vtr::storage;

using StringMsg = std_msgs::msg::String;

class TemporaryDirectoryFixture : public Test {
 public:
  TemporaryDirectoryFixture() {
    temp_dir_ = rcpputils::fs::create_temp_directory("tmp_test_dir_").string();
    // temp_dir_ = "<set to some directory to check the resulting database>";
    // (void)rcpputils::fs::create_directories(temp_dir_);
  }

  ~TemporaryDirectoryFixture() override {
    rcpputils::fs::remove_all(rcpputils::fs::path(temp_dir_));
  }

  std::string temp_dir_;
};

TEST_F(TemporaryDirectoryFixture, constructor_and_destructor) {
  /// \note check the resulting database directory
  // constructor creates database and topic
  DataStreamAccessor<StringMsg> accessor(temp_dir_, "test_string");
  // destructor stores the metadata
}

TEST_F(TemporaryDirectoryFixture,
       constructor_and_destructor_with_message_saved) {
  /// \note check the resulting database directory
  DataStreamAccessor<StringMsg> accessor(temp_dir_, "test_string");

  const auto data = std::make_shared<StringMsg>();
  data->data = "data to be saved.";

  const auto lockable_message = std::make_shared<LockableMessage<StringMsg>>(data);
  auto& message = lockable_message->locked().get();

  // message information (no index given implies not saved)
  EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
  EXPECT_EQ(message.getIndex(), NO_INDEX_VALUE);
  EXPECT_EQ(message.getSaved(), false);

  accessor.write(lockable_message);

  // message information after saving
  EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
  EXPECT_EQ(message.getIndex(), 1);
  EXPECT_EQ(message.getSaved(), true);

  // read the message from database
  {
    const auto lockable_message = accessor.readAtIndex(1);
    const auto& message = lockable_message->unlocked().get();
    EXPECT_EQ(message.getData().data, data->data);
    EXPECT_EQ(message.getTimestamp(), NO_TIMESTAMP_VALUE);
    EXPECT_EQ(message.getIndex(), 1);
    EXPECT_EQ(message.getSaved(), true);
  }

  // make some updates to message and write
  message.setTimestamp(0);
  EXPECT_EQ(message.getSaved(), false);

  accessor.write(lockable_message);

  // message information after saving
  EXPECT_EQ(message.getTimestamp(), 0);
  EXPECT_EQ(message.getIndex(), 1);
  EXPECT_EQ(message.getSaved(), true);

  // read the message from database
  {
    const auto lockable_message = accessor.readAtTimestamp(0);
    const auto& message = lockable_message->unlocked().get();
    EXPECT_EQ(message.getData().data, data->data);
    EXPECT_EQ(message.getTimestamp(), 0);
    EXPECT_EQ(message.getIndex(), 1);
    EXPECT_EQ(message.getSaved(), true);
  }

  // manually set saved flag
  message.setTimestamp(100);
  message.setSaved(true);

  accessor.write(lockable_message);

  // message information after saving
  EXPECT_EQ(message.getTimestamp(), 100);
  EXPECT_EQ(message.getIndex(), 1);
  EXPECT_EQ(message.getSaved(), true);

  // read the message from database
  {
    const auto lockable_message = accessor.readAtTimestamp(0);
    const auto& message = lockable_message->unlocked().get();
    EXPECT_EQ(message.getData().data, data->data);
    EXPECT_EQ(message.getTimestamp(), 0);  // not updated since saved is false
    EXPECT_EQ(message.getIndex(), 1);
    EXPECT_EQ(message.getSaved(), true);
  }
}

TEST_F(TemporaryDirectoryFixture, read_from_existing_and_update) {
  // store some data first
  {
    DataStreamAccessor<StringMsg> accessor(temp_dir_, "test_string");

    const auto data = std::make_shared<StringMsg>();
    data->data = "data1";

    const auto lockable_message = std::make_shared<LockableMessage<StringMsg>>(data, 0);

    accessor.write(lockable_message);

    // destructor closes the bag and writes metadata
  }

  // open existing
  {
    DataStreamAccessor<StringMsg> accessor(temp_dir_, "test_string");

    // read the message from database
    {
      const auto lockable_message = accessor.readAtTimestamp(0);
      const auto& message = lockable_message->unlocked().get();
      EXPECT_EQ(message.getData().data, "data1");
      EXPECT_EQ(message.getTimestamp(), 0);
      EXPECT_EQ(message.getIndex(), 1);
      EXPECT_EQ(message.getSaved(), true);
    }

    const auto data = std::make_shared<StringMsg>();
    data->data = "data2";

    const auto lockable_message = std::make_shared<LockableMessage<StringMsg>>(data, 1);
    auto& message = lockable_message->locked().get();

    accessor.write(lockable_message);

    // message information after saving
    EXPECT_EQ(message.getTimestamp(), 1);
    EXPECT_EQ(message.getIndex(), 2);  // now we have 2 messages
    EXPECT_EQ(message.getSaved(), true);

    // read the message from database
    {
      const auto lockable_message = accessor.readAtIndex(2);
      const auto& message = lockable_message->unlocked().get();
      EXPECT_EQ(message.getData().data, "data2");
      EXPECT_EQ(message.getTimestamp(), 1);
      EXPECT_EQ(message.getIndex(), 2);
      EXPECT_EQ(message.getSaved(), true);
    }
  }
}

TEST_F(TemporaryDirectoryFixture, random_access) {
  DataStreamAccessor<StringMsg> accessor(temp_dir_, "test_string");

  StringMsg data;
  data.data = "data";

  // generate 10 messages with different time stamps
  std::vector<std::shared_ptr<LockableMessage<StringMsg>>> messages{
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 0),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 2),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 4),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 6),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 8),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 1),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 3),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 5),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 7),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 9),
  };

  // store the first group
  std::vector<std::shared_ptr<LockableMessage<StringMsg>>> first_group(
      messages.begin(), messages.begin() + 5);
  accessor.write(first_group);

  // read the first stored group
  {
    const auto lockable_messages = accessor.readAtTimestampRange(0, 8);
    ASSERT_EQ(lockable_messages.size(), (size_t)5);
    for (int i = 0; i < 5; i++) {
      const auto& message = lockable_messages[i]->unlocked().get();
      EXPECT_EQ(message.getData().data, "data");
      EXPECT_EQ(message.getTimestamp(), 2 * i);
      EXPECT_EQ(message.getIndex(), i + 1);
      EXPECT_EQ(message.getSaved(), true);
    }
  }

  // store all
  accessor.write(messages);

  // read based on time stamp
  {
    const auto lockable_messages = accessor.readAtTimestampRange(0, 9);
    ASSERT_EQ(lockable_messages.size(), (size_t)10);
    for (int i = 0; i < 10; i++) {
      const auto& message = lockable_messages[i]->unlocked().get();
      EXPECT_EQ(message.getData().data, "data");
      EXPECT_EQ(message.getTimestamp(), i);
      EXPECT_EQ(message.getIndex(), (i % 2 == 0 ? i / 2 + 1 : (i + 1) / 2 + 5));
      EXPECT_EQ(message.getSaved(), true);
    }
  }

  // read based on index
  {
    const auto lockable_messages = accessor.readAtIndexRange(1, 10);
    ASSERT_EQ(lockable_messages.size(), (size_t)10);
    for (int i = 0; i < 5; i++) {
      const auto& message = lockable_messages[i]->unlocked().get();
      EXPECT_EQ(message.getData().data, "data");
      EXPECT_EQ(message.getTimestamp(), 2 * i);
      EXPECT_EQ(message.getIndex(), i + 1);
      EXPECT_EQ(message.getSaved(), true);
    }
    for (int i = 5; i < 10; i++) {
      const auto& message = lockable_messages[i]->unlocked().get();
      EXPECT_EQ(message.getData().data, "data");
      EXPECT_EQ(message.getTimestamp(), 1 + 2 * (i - 5));
      EXPECT_EQ(message.getIndex(), i + 1);
      EXPECT_EQ(message.getSaved(), true);
    }
  }
}

TEST_F(TemporaryDirectoryFixture, data_stream_writer_locks_message_correctly) {
  DataStreamAccessor<StringMsg> accessor(temp_dir_, "test_string");

  StringMsg data;
  data.data = "data";

  // generate 10 messages with different time stamps
  std::vector<std::shared_ptr<LockableMessage<StringMsg>>> messages{
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 0),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 2),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 4),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 6),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 8),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 1),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 3),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 5),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 7),
      std::make_shared<LockableMessage<StringMsg>>(
          std::make_shared<StringMsg>(data), 9),
  };

  std::thread th([&messages]() {
    std::cout << "lock message 0" << std::endl;
    messages[0]->lock();
    std::this_thread::sleep_for(1s);
    std::cout << "lock message 1" << std::endl;
    messages[1]->lock();
    messages[0]->unlock();
    std::this_thread::sleep_for(1s);
    std::cout << "lock message 2" << std::endl;
    messages[2]->lock();
    messages[1]->unlock();
    std::this_thread::sleep_for(1s);
    std::cout << "lock message 3" << std::endl;
    messages[3]->lock();
    messages[2]->unlock();

    // update message 3, which has timestamp 6 to 20
    messages[3]->unlocked().get().setTimestamp(10);

    std::cout << "releasing the last lock, writer can start now" << std::endl;
    messages[3]->unlock();
  });

  // not a very good way to wait for thread to start
  std::this_thread::sleep_for(1s);

  // store all
  accessor.write(messages);
  std::cout << "writer finished" << std::endl;

  // read message
  {
    auto lockable_message = accessor.readAtTimestamp(6);
    ASSERT_EQ(lockable_message, nullptr);  // we change the timestamp to 10.
    lockable_message = accessor.readAtTimestamp(10);
    ASSERT_NE(lockable_message, nullptr);
    const auto& message = lockable_message->unlocked().get();
    EXPECT_EQ(message.getTimestamp(), 10);
    EXPECT_EQ(message.getIndex(), 4);
    EXPECT_EQ(message.getSaved(), true);
  }

  th.join();
}