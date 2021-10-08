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

/** \brief Defines a storable type being converted to StringMsg when saving */
class Storable1 : public StringMsg {
 public:
  /// IMPORTANT: the following two functions are required and must be defined in
  /// public scope
  /** \brief Returns the ROS2 message to be stored */
  StringMsg toStorable() const { return *this; }
  /** \brief Static function that constructs this class from ROS2 message */
  static std::shared_ptr<Storable1> fromStorable(const StringMsg& storable) {
    auto data = std::make_shared<Storable1>();
    data->data = storable.data;  // can use a move here, but don't bother.
    return data;
  }
};

/** \brief Defines a storable type being converted to StringMsg when saving */
class Storable2 {
 public:
  /// IMPORTANT: the following two functions are required and must be defined in
  /// public scope
  /** \brief Returns the ROS2 message to be stored */
  StringMsg toStorable() const {
    StringMsg msg;
    msg.data = data;
    return msg;
  }
  /** \brief Static function that constructs this class from ROS2 message */
  static std::shared_ptr<Storable2> fromStorable(const StringMsg& storable) {
    auto data = std::make_shared<Storable2>();
    data->data = storable.data;  // can use a move here, but don't bother.
    return data;
  }

  std::string data;
};

TEST_F(TemporaryDirectoryFixture, ros2_type) {
  /// \note check the resulting database directory
  DataStreamAccessor<StringMsg> accessor(temp_dir_, "test_string");

  const auto data = std::make_shared<StringMsg>();
  data->data = "data to be saved.";

  const auto lockable_message =
      std::make_shared<LockableMessage<StringMsg>>(data);
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
}

TEST_F(TemporaryDirectoryFixture, storable_type_v1) {
  /// \note check the resulting database directory
  DataStreamAccessor<Storable1> accessor(temp_dir_, "test_string");

  const auto data = std::make_shared<Storable1>();
  data->data = "data to be saved.";

  const auto lockable_message =
      std::make_shared<LockableMessage<Storable1>>(data);
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
}

TEST_F(TemporaryDirectoryFixture, storable_type_v2) {
  /// \note check the resulting database directory
  DataStreamAccessor<Storable2> accessor(temp_dir_, "test_string");

  const auto data = std::make_shared<Storable2>();
  data->data = "data to be saved.";

  const auto lockable_message =
      std::make_shared<LockableMessage<Storable2>>(data);
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
}