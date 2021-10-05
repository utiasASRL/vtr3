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

#include <filesystem>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/logging_macros.h"
#include "rcutils/snprintf.h"

#include "vtr_storage/accessor/storage_accessor.hpp"

// clang-format off

using namespace ::testing;  // NOLINT
using namespace vtr::storage;

class TemporaryDirectoryFixture : public Test
{
public:
  TemporaryDirectoryFixture()
  {
    temporary_dir_path_ = rcpputils::fs::create_temp_directory("tmp_test_dir_").string();
  }

  ~TemporaryDirectoryFixture() override
  {
    rcpputils::fs::remove_all(rcpputils::fs::path(temporary_dir_path_));
  }

  std::string temporary_dir_path_;
};

class AccessorTestFixture : public TemporaryDirectoryFixture
{
public:
  AccessorTestFixture()
  {
    allocator_ = rcutils_get_default_allocator();
  }

  std::shared_ptr<rcutils_uint8_array_t> make_serialized_message(std::string message)
  {
    int message_size = get_buffer_capacity(message);
    message_size++;  // need to account for terminating null character
    assert(message_size > 0);

    auto msg = new rcutils_uint8_array_t;
    *msg = rcutils_get_zero_initialized_uint8_array();
    auto ret = rcutils_uint8_array_init(msg, message_size, &allocator_);
    if (ret != RCUTILS_RET_OK) {
      throw std::runtime_error("Error allocating resources " + std::to_string(ret));
    }

    auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      msg,
      [](rcutils_uint8_array_t * msg) {
        int error = rcutils_uint8_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_storage_default_plugins", "Leaking memory %i", error);
        }
      });

    serialized_data->buffer_length = message_size;
    int written_size = write_data_to_serialized_string_message(
      serialized_data->buffer, serialized_data->buffer_capacity, message);

    assert(written_size == message_size - 1);  // terminated null character not counted
    (void) written_size;
    return serialized_data;
  }

  std::string deserialize_message(std::shared_ptr<rcutils_uint8_array_t> serialized_message)
  {
    uint8_t * copied = new uint8_t[serialized_message->buffer_length];
    auto string_length = serialized_message->buffer_length - 8;
    memcpy(copied, &serialized_message->buffer[8], string_length);
    std::string message_content(reinterpret_cast<char *>(copied));
    // cppcheck-suppress mismatchAllocDealloc ; complains about "copied" but used new[] and delete[]
    delete[] copied;
    return message_content;
  }

protected:
  int get_buffer_capacity(const std::string & message)
  {
    return write_data_to_serialized_string_message(nullptr, 0, message);
  }

  int write_data_to_serialized_string_message(
    uint8_t * buffer, size_t buffer_capacity, const std::string & message)
  {
    // This function also writes the final null charachter, which is absent in the CDR format.
    // Here this behaviour is ok, because we only test test writing and reading from/to sqlite.
    return rcutils_snprintf(
      reinterpret_cast<char *>(buffer),
      buffer_capacity,
      "%c%c%c%c%c%c%c%c%s",
      0x00, 0x01, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,
      message.c_str());
  }

  rcutils_allocator_t allocator_;
  MetadataIo metadata_io_;

  const std::string plugin_id_ = "sqlite3";
};

TEST_F(AccessorTestFixture, insert_read_update_read_messages_with_one_accessor) {

  std::unique_ptr<StorageAccessor> storage_accessor = std::make_unique<StorageAccessor>();
  auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag").string();
  storage_accessor->open(db_file);

  std::vector<std::tuple<std::string, int64_t, std::string, std::string, std::string>> messages =
  {
    std::make_tuple("1st message", 1, "topic1", "type1", "rmw1"),
    std::make_tuple("2nd message", 2, "topic2", "type2", "rmw2"),
    std::make_tuple("3th message", 3, "topic3", "type3", "rmw3"),
    std::make_tuple("4th message", 4, "topic4", "type4", "rmw4"),
    std::make_tuple("5th message", 5, "topic5", "type5", "rmw5"),
    std::make_tuple("6th message", 6, "topic6", "type6", "rmw6"),
    std::make_tuple("7th message", 7, "topic7", "type7", "rmw7"),
    std::make_tuple("8th message", 8, "topic8", "type8", "rmw8"),
    std::make_tuple("9th message", 9, "topic9", "type9", "rmw9"),
    std::make_tuple("10th message", 10, "topic10", "type10", "rmw10"),
    std::make_tuple("11th message", 11, "topic11", "type11", "rmw11"),
    std::make_tuple("12th message", 12, "topic12", "type12", "rmw12")
  };

  // insert messages one by one
  size_t num_one_by_one = 6;
  for (size_t i = 0; i < num_one_by_one; i++) {
    const auto &msg = messages[i];
    const auto topic_name = std::get<2>(msg);
    const auto type_name = std::get<3>(msg);
    const auto rmw_format = std::get<4>(msg);
    storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
    bag_message->time_stamp = std::get<1>(msg);
    bag_message->topic_name = topic_name;
    storage_accessor->write(bag_message);
    EXPECT_THAT(bag_message->index, Eq((int)(i+1)));
  }

  // insert message as a vector
  std::vector<std::shared_ptr<SerializedBagMessage>> bag_messages;
  for (size_t i = num_one_by_one; i < messages.size(); i++) {
    const auto &msg = messages[i];
    std::string topic_name = std::get<2>(msg);
    std::string type_name = std::get<3>(msg);
    std::string rmw_format = std::get<4>(msg);
    storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
    bag_message->time_stamp = std::get<1>(msg);
    bag_message->topic_name = topic_name;
    bag_messages.push_back(bag_message);
  }
  storage_accessor->write(bag_messages); // message index is updated
  for (size_t i = num_one_by_one; i < messages.size(); i++) {
    EXPECT_THAT(bag_messages[i - num_one_by_one]->index, Eq((int)(i+1)));
  }

  // read messages
  const auto read_messages = storage_accessor->read_at_timestamp_range(1, messages.size());
  ASSERT_THAT(read_messages, SizeIs(messages.size()));
  for (size_t i = 0; i < messages.size(); i++) {
    EXPECT_THAT(deserialize_message(read_messages[i]->serialized_data), StrEq(std::get<0>(messages[i])));
    EXPECT_THAT(read_messages[i]->time_stamp, Eq(std::get<1>(messages[i])));
    EXPECT_THAT(read_messages[i]->topic_name, Eq(std::get<2>(messages[i])));
  }

  // update messages without creating topic this time
  std::vector<std::shared_ptr<SerializedBagMessage>> bag_messages_updated;
  for (size_t i = 0; i < read_messages.size(); i++) {
    const auto& msg = messages[i];
    const auto topic_name = std::get<2>(msg);
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg) + " updated");
    bag_message->time_stamp = std::get<1>(msg) + 20;
    bag_message->topic_name = topic_name;
    bag_message->index = read_messages[i]->index;
    bag_messages_updated.push_back(bag_message);
  }
  // also insert all messages again (note: without index, this will be INSERTION)
  for (size_t i = 0; i < messages.size(); i++) {
    const auto &msg = messages[i];
    std::string topic_name = std::get<2>(msg);
    std::string type_name = std::get<3>(msg);
    std::string rmw_format = std::get<4>(msg);
    storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
    bag_message->time_stamp = std::get<1>(msg);
    bag_message->topic_name = topic_name;
    bag_messages_updated.push_back(bag_message);
  }
  storage_accessor->write(bag_messages_updated); // message index is updated

  // read messages
  const auto read_messages_updated = storage_accessor->read_at_index_range(1, read_messages.size()+messages.size());
  ASSERT_THAT(read_messages_updated, SizeIs(read_messages.size()+messages.size()));
  for (size_t i = 0; i < read_messages.size(); i++) {
    EXPECT_THAT(deserialize_message(read_messages_updated[i]->serialized_data), StrEq(std::get<0>(messages[i]) + " updated"));
    EXPECT_THAT(read_messages_updated[i]->time_stamp, Eq(std::get<1>(messages[i]) + 20));
    EXPECT_THAT(read_messages_updated[i]->topic_name, Eq(std::get<2>(messages[i])));
  }
  for (size_t i = 0; i < messages.size(); i++) {
    EXPECT_THAT(deserialize_message(read_messages_updated[i+read_messages.size()]->serialized_data), StrEq(std::get<0>(messages[i])));
    EXPECT_THAT(read_messages_updated[i+read_messages.size()]->time_stamp, Eq(std::get<1>(messages[i])));
    EXPECT_THAT(read_messages_updated[i+read_messages.size()]->topic_name, Eq(std::get<2>(messages[i])));
  }
}

TEST_F(AccessorTestFixture, insert_read_update_read_messages_with_two_accessor) {

  std::unique_ptr<StorageAccessor> storage_accessor = std::make_unique<StorageAccessor>();
  auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag").string();
  storage_accessor->open(db_file);

  std::vector<std::tuple<std::string, int64_t, std::string, std::string, std::string>> messages =
  {
    std::make_tuple("1st message", 1, "topic1", "type1", "rmw1"),
    std::make_tuple("2nd message", 2, "topic2", "type2", "rmw2"),
    std::make_tuple("3rd message", 3, "topic3", "type3", "rmw3")
  };

  // insert messages
  for (auto msg : messages) {
    const auto topic_name = std::get<2>(msg);
    const auto type_name = std::get<3>(msg);
    const auto rmw_format = std::get<4>(msg);
    storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
    bag_message->time_stamp = std::get<1>(msg);
    bag_message->topic_name = topic_name;
    storage_accessor->write(bag_message);
  }

  // read messages
  const auto read_messages = storage_accessor->read_at_timestamp_range(1, 3);
  ASSERT_THAT(read_messages, SizeIs(3));
  for (size_t i = 0; i < 3; i++) {
    EXPECT_THAT(deserialize_message(read_messages[i]->serialized_data), StrEq(std::get<0>(messages[i])));
    EXPECT_THAT(read_messages[i]->time_stamp, Eq(std::get<1>(messages[i])));
    EXPECT_THAT(read_messages[i]->topic_name, Eq(std::get<2>(messages[i])));
  }

  storage_accessor->close();

  storage_accessor->open(db_file);

  // update messages without creating topic this time
  for (size_t i = 0; i < read_messages.size(); i++) {
    const auto& msg = messages[i];
    const auto topic_name = std::get<2>(msg);
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg) + " updated");
    bag_message->time_stamp = std::get<1>(msg) + 20;
    bag_message->topic_name = topic_name;
    bag_message->index = read_messages[i]->index;
    storage_accessor->write(bag_message);
  }

  // read messages
  const auto read_messages_updated = storage_accessor->read_at_index_range(1, 3);
  ASSERT_THAT(read_messages_updated, SizeIs(3));
  for (size_t i = 0; i < 3; i++) {
    EXPECT_THAT(deserialize_message(read_messages_updated[i]->serialized_data), StrEq(std::get<0>(messages[i]) + " updated"));
    EXPECT_THAT(read_messages_updated[i]->time_stamp, Eq(std::get<1>(messages[i]) + 20));
    EXPECT_THAT(read_messages_updated[i]->topic_name, Eq(std::get<2>(messages[i])));
  }
}