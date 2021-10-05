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
 * \file test_sqlite_storage2.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/snprintf.h"

#include "vtr_storage/storage/storage_filter.hpp"

#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::storage;

// clang-format off

TEST_F(StorageTestFixture, insert_read_update_read_messages_with_one_accessor) {

  std::unique_ptr<ReadWriteInterface> storage_accessor = std::make_unique<sqlite::SqliteStorage>();
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
    std::string topic_name = std::get<2>(msg);
    std::string type_name = std::get<3>(msg);
    std::string rmw_format = std::get<4>(msg);
    storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
    bag_message->time_stamp = std::get<1>(msg);
    bag_message->topic_name = topic_name;
    storage_accessor->write(bag_message); // message index is updated
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
  std::vector<std::shared_ptr<SerializedBagMessage>> read_messages;
  while (storage_accessor->has_next())
    read_messages.push_back(storage_accessor->read_next());

  ASSERT_THAT(read_messages, SizeIs(messages.size()));
  for (size_t i = 0; i < read_messages.size(); i++) {
    EXPECT_THAT(deserialize_message(read_messages[i]->serialized_data), StrEq(std::get<0>(messages[i])));
    EXPECT_THAT(read_messages[i]->time_stamp, Eq(std::get<1>(messages[i])));
    EXPECT_THAT(read_messages[i]->topic_name, Eq(std::get<2>(messages[i])));
  }

  // update messages
  for (size_t i = 0; i < read_messages.size(); i++) {
    const auto& msg = messages[i];
    std::string topic_name = std::get<2>(msg);
    std::string type_name = std::get<3>(msg);
    std::string rmw_format = std::get<4>(msg);
    storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg) + " updated");
    bag_message->time_stamp = std::get<1>(msg);
    bag_message->topic_name = topic_name;
    bag_message->index = read_messages[i]->index;
    storage_accessor->write(bag_message);
  }

  // read messages
  storage_accessor->seek(0);
  std::vector<std::shared_ptr<SerializedBagMessage>> read_messages_updated;
  while (storage_accessor->has_next())
    read_messages_updated.push_back(storage_accessor->read_next());

  ASSERT_THAT(read_messages_updated, SizeIs(messages.size()));
  for (size_t i = 0; i < messages.size(); i++) {
    EXPECT_THAT(deserialize_message(read_messages_updated[i]->serialized_data), StrEq(std::get<0>(messages[i]) + " updated"));
    EXPECT_THAT(read_messages_updated[i]->time_stamp, Eq(std::get<1>(messages[i])));
    EXPECT_THAT(read_messages_updated[i]->topic_name, Eq(std::get<2>(messages[i])));
  }
}

TEST_F(StorageTestFixture, insert_read_update_read_messages_with_two_accessor) {

  std::vector<std::tuple<std::string, int64_t, std::string, std::string, std::string>> messages =
  {
    std::make_tuple("1st message", 1, "topic1", "type1", "rmw1"),
    std::make_tuple("2nd message", 2, "topic2", "type2", "rmw2"),
    std::make_tuple("3rd message", 3, "topic3", "type3", "rmw3")
  };

  std::vector<std::shared_ptr<SerializedBagMessage>> read_messages;
  {
    std::unique_ptr<ReadWriteInterface> storage_accessor = std::make_unique<sqlite::SqliteStorage>();
    auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag").string();
    storage_accessor->open(db_file);


    // insert messages
    for (auto msg : messages) {
      std::string topic_name = std::get<2>(msg);
      std::string type_name = std::get<3>(msg);
      std::string rmw_format = std::get<4>(msg);
      storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
      auto bag_message = std::make_shared<SerializedBagMessage>();
      bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
      bag_message->time_stamp = std::get<1>(msg);
      bag_message->topic_name = topic_name;
      storage_accessor->write(bag_message);
    }

    // read messages
    while (storage_accessor->has_next())
      read_messages.push_back(storage_accessor->read_next());

    ASSERT_THAT(read_messages, SizeIs(3));
    for (size_t i = 0; i < 3; i++) {
      EXPECT_THAT(deserialize_message(read_messages[i]->serialized_data), StrEq(std::get<0>(messages[i])));
      EXPECT_THAT(read_messages[i]->time_stamp, Eq(std::get<1>(messages[i])));
      EXPECT_THAT(read_messages[i]->topic_name, Eq(std::get<2>(messages[i])));
    }
  }

  {
    std::unique_ptr<ReadWriteInterface> storage_accessor = std::make_unique<sqlite::SqliteStorage>();
    auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag").string();
    storage_accessor->open(db_file);

    // update messages
    for (size_t i = 0; i < read_messages.size(); i++) {
      const auto& msg = messages[i];
      std::string topic_name = std::get<2>(msg);
      std::string type_name = std::get<3>(msg);
      std::string rmw_format = std::get<4>(msg);
      storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
      auto bag_message = std::make_shared<SerializedBagMessage>();
      bag_message->serialized_data = make_serialized_message(std::get<0>(msg) + " updated");
      bag_message->time_stamp = std::get<1>(msg);
      bag_message->topic_name = topic_name;
      bag_message->index = read_messages[i]->index;
      storage_accessor->write(bag_message);
    }

    // read messages
    std::vector<std::shared_ptr<SerializedBagMessage>> read_messages_updated;
    while (storage_accessor->has_next())
      read_messages_updated.push_back(storage_accessor->read_next());

    ASSERT_THAT(read_messages_updated, SizeIs(3));
    for (size_t i = 0; i < 3; i++) {
      EXPECT_THAT(deserialize_message(read_messages_updated[i]->serialized_data), StrEq(std::get<0>(messages[i]) + " updated"));
      EXPECT_THAT(read_messages_updated[i]->time_stamp, Eq(std::get<1>(messages[i])));
      EXPECT_THAT(read_messages_updated[i]->topic_name, Eq(std::get<2>(messages[i])));
    }
  }
}

TEST_F(StorageTestFixture, randomly_reading_messages) {
  std::unique_ptr<ReadWriteInterface> storage_accessor = std::make_unique<sqlite::SqliteStorage>();
  auto db_file = (rcpputils::fs::path(temporary_dir_path_) / "rosbag").string();
  storage_accessor->open(db_file);

  std::vector<std::tuple<std::string, int64_t, std::string, std::string, std::string>> messages =
  {
    std::make_tuple("message0", 0, "topic0", "type0", "rmw"),
    std::make_tuple("message1", 1, "topic0", "type0", "rmw"),
    std::make_tuple("message2", 2, "topic0", "type0", "rmw"),
    std::make_tuple("message3", 3, "topic0", "type0", "rmw"),
    std::make_tuple("message4", 4, "topic0", "type0", "rmw"),
    std::make_tuple("message5", 0, "topic1", "type1", "rmw"),
    std::make_tuple("message6", 1, "topic1", "type1", "rmw"),
    std::make_tuple("message7", 2, "topic1", "type1", "rmw"),
    std::make_tuple("message8", 3, "topic1", "type1", "rmw"),
    std::make_tuple("message9", 4, "topic1", "type1", "rmw")
  };

  // insert messages
  for (auto msg : messages) {
    std::string topic_name = std::get<2>(msg);
    std::string type_name = std::get<3>(msg);
    std::string rmw_format = std::get<4>(msg);
    storage_accessor->create_topic({topic_name, type_name, rmw_format, ""});
    auto bag_message = std::make_shared<SerializedBagMessage>();
    bag_message->serialized_data = make_serialized_message(std::get<0>(msg));
    bag_message->time_stamp = std::get<1>(msg);
    bag_message->topic_name = topic_name;
    storage_accessor->write(bag_message);
  }
  // read at a random timestamp
  {
    const auto read_message = storage_accessor->read_at_timestamp(3);
    // message stored first is returned
    EXPECT_THAT(deserialize_message(read_message->serialized_data), StrEq("message3"));
    EXPECT_THAT(read_message->time_stamp, Eq((unsigned)3));
    EXPECT_THAT(read_message->topic_name, Eq("topic0"));
  }

  // read at a timestamp range
  {
    const auto read_messages = storage_accessor->read_at_timestamp_range(3, 3);
    ASSERT_THAT(read_messages.size(), Eq((unsigned)2));
    // messages are orded by timestamp then insertion order
    // first
    EXPECT_THAT(deserialize_message(read_messages[0]->serialized_data), StrEq("message3"));
    EXPECT_THAT(read_messages[0]->time_stamp, Eq((unsigned)3));
    EXPECT_THAT(read_messages[0]->topic_name, Eq("topic0"));
    // second
    EXPECT_THAT(deserialize_message(read_messages[1]->serialized_data), StrEq("message8"));
    EXPECT_THAT(read_messages[1]->time_stamp, Eq((unsigned)3));
    EXPECT_THAT(read_messages[1]->topic_name, Eq("topic1"));
  }

  // read at a random index
  {
    const auto read_message = storage_accessor->read_at_index(8);
    // message stored is returned (this must be unique since it is by id)
    EXPECT_THAT(deserialize_message(read_message->serialized_data), StrEq("message7"));
    EXPECT_THAT(read_message->time_stamp, Eq((unsigned)2));
    EXPECT_THAT(read_message->topic_name, Eq("topic1"));
  }

  // read at a index range
  {
    const auto read_messages = storage_accessor->read_at_index_range(5, 6);
    ASSERT_THAT(read_messages.size(), Eq((unsigned)2));
    // messages are orded by insertion order then timestamp
    // first
    EXPECT_THAT(deserialize_message(read_messages[0]->serialized_data), StrEq("message4"));
    EXPECT_THAT(read_messages[0]->time_stamp, Eq((unsigned)4));
    EXPECT_THAT(read_messages[0]->topic_name, Eq("topic0"));
    // second
    EXPECT_THAT(deserialize_message(read_messages[1]->serialized_data), StrEq("message5"));
    EXPECT_THAT(read_messages[1]->time_stamp, Eq((unsigned)0));
    EXPECT_THAT(read_messages[1]->topic_name, Eq("topic1"));
  }

}