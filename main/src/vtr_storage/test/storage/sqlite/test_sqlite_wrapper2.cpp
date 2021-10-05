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
 * \file test_sqlite_wrapper2.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/types.h"

#include "vtr_storage/storage/sqlite/sqlite_wrapper.hpp"

#include "storage_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace vtr::storage;

// clang-format off

class SqliteWrapperTestFixture : public StorageTestFixture
{
public:
  SqliteWrapperTestFixture()
  : StorageTestFixture(),
    db_((rcpputils::fs::path(temporary_dir_path_) / "test.db3").string(), IOFlag::READ_WRITE)
  {
    // data base initialization - desired ros2 bag format (SqliteStorage.initialize)
    std::string create_stmt = "CREATE TABLE topics(" \
      "id INTEGER PRIMARY KEY," \
      "name TEXT NOT NULL," \
      "type TEXT NOT NULL," \
      "serialization_format TEXT NOT NULL," \
      "offered_qos_profiles TEXT NOT NULL);";
    db_.prepare_statement(create_stmt)->execute_and_reset();
    create_stmt = "CREATE TABLE messages(" \
      "id INTEGER PRIMARY KEY," \
      "topic_id INTEGER NOT NULL," \
      "timestamp INTEGER NOT NULL, " \
      "data BLOB NOT NULL);";
    db_.prepare_statement(create_stmt)->execute_and_reset();
    create_stmt = "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
    db_.prepare_statement(create_stmt)->execute_and_reset();
  }

  sqlite::SqliteWrapper db_;
};

TEST_F(SqliteWrapperTestFixture, creating_querying_removing_topics) {

  std::vector topics{"topic0", "topic1", "topic2"};

  // insert topics
  /// \note serialization_format -> we always use 'cdr'
  /// \note offered_qos_profiles -> empty string means default
  auto insert_topic_stmt = db_.prepare_statement("INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) VALUES (?, ?, ?, ?)");
  insert_topic_stmt->bind(topics[0], "std_msgs/msg/Bool", "cdr", "")->execute_and_reset();
  insert_topic_stmt->bind(topics[1], "std_msgs/msg/Bool", "cdr", "")->execute_and_reset();
  insert_topic_stmt->bind(topics[2], "std_msgs/msg/Bool", "cdr", "")->execute_and_reset();

  // query topics
  auto query_stmt = db_.prepare_statement("SELECT name, type, serialization_format FROM topics ORDER BY id;");
  auto query_results = query_stmt->execute_query<std::string, std::string, std::string>();
  int i = 0;
  for (auto result : query_results) {
    ASSERT_THAT(std::get<0>(result), StrEq(topics[i]));
    i++;
  }

  // remove topics
  auto delete_topic_stmt = db_.prepare_statement("DELETE FROM topics where name = ? and type = ? and serialization_format = ?");
  delete_topic_stmt->bind(topics[0], "std_msgs/msg/Bool", "cdr")->execute_and_reset();
  delete_topic_stmt->bind(topics[1], "std_msgs/msg/Bool", "cdr")->execute_and_reset();
  delete_topic_stmt->bind(topics[2], "std_msgs/msg/Bool", "cdr")->execute_and_reset();

  // query topics
  query_stmt->reset();
  query_results = query_stmt->execute_query<std::string, std::string, std::string>();
  ASSERT_TRUE(query_results.begin() == query_results.end());
}

TEST_F(SqliteWrapperTestFixture, writing_updating_messages) {

  std::vector topics{"topic0", "topic1"};

  // insert topics
  /// \note serialization_format -> we always use 'cdr'
  /// \note offered_qos_profiles -> empty string means default
  auto insert_topic_stmt = db_.prepare_statement("INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) VALUES (?, ?, ?, ?)");
  insert_topic_stmt->bind(topics[0], "std_msgs/msg/Bool", "cdr", "")->execute_and_reset();
  insert_topic_stmt->bind(topics[1], "std_msgs/msg/Bool", "cdr", "")->execute_and_reset();

  // write message
  int topic_id = 1; // topic2id mapping is {"topic0": 1, "topic1": 2}
  rcutils_time_point_value_t timestamp = 1099511627783;
  std::shared_ptr<rcutils_uint8_array_t> message = make_serialized_message("message1");
  auto write_stmt = db_.prepare_statement("INSERT INTO messages (timestamp, topic_id, data) VALUES (?, ?, ?);");
  write_stmt->bind(timestamp, topic_id, message)->execute_and_reset();

  // read inserted
  auto read_stmt = db_.prepare_statement("SELECT data, timestamp, topics.name, messages.id "
                                         "FROM messages JOIN topics ON messages.topic_id = topics.id "
                                         "WHERE (timestamp = ?) "
                                         "ORDER BY messages.timestamp;");
  auto result_iter = read_stmt->bind(timestamp)->execute_query<std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  auto result = result_iter.get_single_line();

  ASSERT_THAT(deserialize_message(std::get<0>(result)), StrEq("message1"));
  ASSERT_THAT(std::get<1>(result), Eq(1099511627783));
  ASSERT_THAT(std::get<2>(result), StrEq("topic0"));
  ASSERT_THAT(std::get<3>(result), Eq(1));

  // update message
  topic_id = 2;
  timestamp = 1099511627784;
  message = make_serialized_message("message2");
  auto update_stmt = db_.prepare_statement("UPDATE messages SET timestamp = ?, topic_id = ?, data = ? WHERE (id = ?);");
  update_stmt->bind(timestamp, topic_id, message, std::get<3>(result))->execute_and_reset();

  // read updated
  read_stmt = db_.prepare_statement("SELECT data, timestamp, topics.name, messages.id "
                                    "FROM messages JOIN topics ON messages.topic_id = topics.id "
                                    "WHERE (timestamp = ?) "
                                    "ORDER BY messages.timestamp;");
  result_iter = read_stmt->bind(timestamp)->execute_query<std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  result = result_iter.get_single_line();

  ASSERT_THAT(deserialize_message(std::get<0>(result)), StrEq("message2"));
  ASSERT_THAT(std::get<1>(result), Eq(1099511627784));
  ASSERT_THAT(std::get<2>(result), StrEq("topic1"));
  ASSERT_THAT(std::get<3>(result), Eq(1));
}

TEST_F(SqliteWrapperTestFixture, randomly_reading_messages) {
  // insert topics
  {
    /// \note serialization_format -> we always use 'cdr'
    /// \note offered_qos_profiles -> empty string means default
    std::vector topics{"topic0", "topic1"};
    auto insert_topic_stmt = db_.prepare_statement("INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) VALUES (?, ?, ?, ?)");
    insert_topic_stmt->bind(topics[0], "std_msgs/msg/Bool", "cdr", "")->execute_and_reset();
    insert_topic_stmt->bind(topics[1], "std_msgs/msg/Bool", "cdr", "")->execute_and_reset();
  }

  // write some messages
  {
    auto write_stmt = db_.prepare_statement("INSERT INTO messages (timestamp, topic_id, data) VALUES (?, ?, ?);");
    // write some messages to topic 0
    int topic_id = 1; // topic2id mapping is {"topic0": 1, "topic1": 2}
    rcutils_time_point_value_t timestamp = 0;
    for (int i = 0; i < 10; i++) {
      const auto message = make_serialized_message("message" + std::to_string(timestamp));
      write_stmt->bind(timestamp, topic_id, message)->execute_and_reset();
      timestamp += 2;
    }
    // write some messages to topic 1
    topic_id = 2; // topic2id mapping is {"topic0": 1, "topic1": 2}
    timestamp = 1;
    for (int i = 0; i < 10; i++) {
      const auto message = make_serialized_message("message" + std::to_string(timestamp));
      write_stmt->bind(timestamp, topic_id, message)->execute_and_reset();
      timestamp += 2;
    }
  }

  // read at a random timestamp
  {
    rcutils_time_point_value_t timestamp = 5;
    std::string read_stmt_str = "SELECT data, timestamp, topics.name, messages.id "
                                "FROM messages JOIN topics ON messages.topic_id = topics.id "
                                "WHERE (topics.name IN ('topic0','topic1')) AND "
                                "(messages.timestamp = ?) "
                                "ORDER BY messages.timestamp, messages.id;";
    auto read_stmt = db_.prepare_statement(read_stmt_str);
    auto result_iter = read_stmt->bind(timestamp)->execute_query<std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
    auto result = result_iter.get_single_line();
    ASSERT_THAT(deserialize_message(std::get<0>(result)), StrEq("message" + std::to_string(timestamp)));
  }

  // read at a timestamp range
  {
    rcutils_time_point_value_t start_timestamp = 3;
    rcutils_time_point_value_t end_timestamp = 15;
    std::string read_stmt_str = "SELECT data, timestamp, topics.name, messages.id "
                                "FROM messages JOIN topics ON messages.topic_id = topics.id "
                                "WHERE (topics.name IN ('topic0')) AND "
                                "(messages.timestamp BETWEEN ? AND ?) "
                                "ORDER BY messages.timestamp, messages.id;";
    auto read_stmt = db_.prepare_statement(read_stmt_str);
    auto result_iter = read_stmt->bind(start_timestamp, end_timestamp)->execute_query<std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();

    rcutils_time_point_value_t timestamp = 4;  // since we only read even number from topic0
    for (auto result : result_iter) {
      ASSERT_THAT(std::get<1>(result), Eq(timestamp));
      ASSERT_THAT(deserialize_message(std::get<0>(result)), StrEq("message" + std::to_string(timestamp)));
      ASSERT_THAT(std::get<2>(result), StrEq("topic0"));
      timestamp += 2;
    }
  }

  // read at a random index
  {
    int index = 5;
    std::string read_stmt_str = "SELECT data, timestamp, topics.name, messages.id "
                                "FROM messages JOIN topics ON messages.topic_id = topics.id "
                                "WHERE (topics.name IN ('topic0','topic1')) AND "
                                "(messages.id = ?) "
                                "ORDER BY messages.id, messages.timestamp;";
    auto read_stmt = db_.prepare_statement(read_stmt_str);
    auto result_iter = read_stmt->bind(index)->execute_query<std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
    auto result = result_iter.get_single_line();
    ASSERT_THAT(std::get<3>(result), Eq(index));
  }

  // read at a timestamp range
  {
    int start_index = 1;
    int end_index = 20;
    std::string read_stmt_str = "SELECT data, timestamp, topics.name, messages.id "
                                "FROM messages JOIN topics ON messages.topic_id = topics.id "
                                "WHERE (topics.name IN ('topic1')) AND "
                                "(messages.id BETWEEN ? AND ?) "
                                "ORDER BY messages.id, messages.timestamp;";
    auto read_stmt = db_.prepare_statement(read_stmt_str);
    auto result_iter = read_stmt->bind(start_index, end_index)->execute_query<std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();

    int index = 11;  // since we only read even number from topic0
    for (auto result : result_iter) {
      ASSERT_THAT(std::get<3>(result), Eq(index));
      index++;
    }
  }

}