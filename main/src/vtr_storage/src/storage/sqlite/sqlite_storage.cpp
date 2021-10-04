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
 * \file sqlite_storage.cpp
 * \brief SqliteStorage class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_storage/storage/sqlite/sqlite_storage.hpp"

#include <sys/stat.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcpputils/filesystem_helper.hpp"

#include "vtr_storage/storage/metadata_io.hpp"
#include "vtr_storage/storage/serialized_bag_message.hpp"
#include "vtr_storage/storage/sqlite/sqlite_exception.hpp"
#include "vtr_storage/storage/sqlite/sqlite_statement_wrapper.hpp"

namespace {
/// \note the following code is adapted from rosbag2 galactic

// Copyright 2018, Bosch Software Innovations GmbH.
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

// clang-format off

using namespace vtr::storage;

bool ends_with(const std::string& full_string, const std::string& ending) {
  if (full_string.length() >= ending.length())
    return (0 == full_string.compare(full_string.length() - ending.length(),
                                     ending.length(), ending));
  else
    return false;
}
#if false
std::string to_string(IOFlag io_flag)
{
  switch (io_flag) {
    case IOFlag::APPEND:
      return "APPEND";
    case IOFlag::READ_ONLY:
      return "READ_ONLY";
    case IOFlag::READ_WRITE:
      return "READ_WRITE";
    default:
      return "UNKNOWN";
  }
}
#endif
bool is_read_write(const IOFlag io_flag)
{
  return io_flag == IOFlag::READ_WRITE;
}
#if false
bool is_read_only(const IOFlag io_flag)
{
  return io_flag == IOFlag::READ_ONLY;
}
#endif
constexpr const auto FILE_EXTENSION = ".db3";

// Minimum size of a sqlite3 database file in bytes (84 kiB).
constexpr const uint64_t MIN_SPLIT_FILE_SIZE = 86016;

// clang-format on
}  // namespace

namespace vtr {
namespace storage {
namespace sqlite {
/// \note the following code is adapted from rosbag2 galactic

// Copyright 2018, Bosch Software Innovations GmbH.
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

// clang-format off

SqliteStorage::~SqliteStorage()
{
  if (active_transaction_) {
    commit_transaction();
  }
}

void SqliteStorage::open(
  const std::string & uri, IOFlag io_flag)
{
  /// \note rosbag2 SqliteStorage generates pragmas here, which can include some
  /// settings to improve resilience.

  relative_path_ = ends_with(uri, FILE_EXTENSION) ? uri : uri + FILE_EXTENSION;

  bool database_exists = rcpputils::fs::path(relative_path_).exists();
  // APPEND and READ_ONLY require the DB to exist
  if ((!is_read_write(io_flag)) && !database_exists) {
    throw std::runtime_error(
            "Failed to read from bag: File '" + relative_path_ + "' does not exist!");
  }

  try {
    database_ = std::make_unique<SqliteWrapper>(relative_path_, io_flag);
  } catch (const SqliteException & e) {
    throw std::runtime_error("Failed to setup storage. Error: " + std::string(e.what()));
  }

  // initialize only for READ_WRITE since the DB is already initialized if in APPEND/READ_ONLY.
  if (is_read_write(io_flag) && !database_exists) {
    initialize();
  } else {
    fill_topics_map(); // get existing topics for modification
  }

  /// \todo need to fill topics

  // Reset the read and write statements in case the database changed.
  // These will be reinitialized lazily on the first read or write.
  read_statement_ = nullptr;
  insert_statement_ = nullptr;
  update_statement_ = nullptr;
#if false
  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_INFO_STREAM(
    "Opened database '" << relative_path_ << "' for " << to_string(io_flag) << ".");
#endif
}

void SqliteStorage::activate_transaction()
{
  if (active_transaction_) {
    return;
  }

#if false
  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_DEBUG_STREAM("begin transaction");
#endif
  database_->prepare_statement("BEGIN TRANSACTION;")->execute_and_reset();

  active_transaction_ = true;
}

void SqliteStorage::commit_transaction()
{
  if (!active_transaction_) {
    return;
  }

#if false
  ROSBAG2_STORAGE_DEFAULT_PLUGINS_LOG_DEBUG_STREAM("commit transaction");
#endif
  database_->prepare_statement("COMMIT;")->execute_and_reset();

  active_transaction_ = false;
}

void SqliteStorage::write(const std::shared_ptr<SerializedBagMessage> & message)
{
  std::lock_guard<std::mutex> db_lock(database_write_mutex_);
  write_locked(message);
}

void SqliteStorage::write(const std::vector<std::shared_ptr<SerializedBagMessage>> & messages)
{
  std::lock_guard<std::mutex> db_lock(database_write_mutex_);
  if (!insert_statement_ || !update_statement_) {
    prepare_for_writing();
  }
  /// \note cannot use transaction here because we need the last insertion id,
  /// which may not be correct if we use transaction.
#if false
  activate_transaction();
#endif
  for (const auto & message : messages) {
    write_locked(message);
  }
#if false
  commit_transaction();
#endif
}

void SqliteStorage::write_locked(const std::shared_ptr<SerializedBagMessage> & message)
{
  if (!insert_statement_ || !update_statement_) {
    prepare_for_writing();
  }
  auto topic_entry = topics_.find(message->topic_name);
  if (topic_entry == end(topics_)) {
    throw SqliteException(
            "Topic '" + message->topic_name +
            "' has not been created yet! Call 'create_topic' first.");
  }

  if (message->index == 0) {
    insert_statement_->bind(message->time_stamp, topic_entry->second, message->serialized_data);
    insert_statement_->execute_and_reset();
    message->index = static_cast<int>(database_->get_last_insert_id());
  } else {
    update_statement_->bind(message->time_stamp, topic_entry->second, message->serialized_data, message->index);
    update_statement_->execute_and_reset();
  }
}

bool SqliteStorage::has_next()
{
  if (!read_statement_) {
    prepare_for_reading();
  }

  return current_message_row_ != message_result_.end();
}

std::shared_ptr<SerializedBagMessage> SqliteStorage::read_next()
{
  if (!read_statement_) {
    prepare_for_reading();
  }

  auto bag_message = std::make_shared<SerializedBagMessage>();
  bag_message->serialized_data = std::get<0>(*current_message_row_);
  bag_message->time_stamp = std::get<1>(*current_message_row_);
  bag_message->topic_name = std::get<2>(*current_message_row_);
  bag_message->index = std::get<3>(*current_message_row_);

  // set start time to current time
  // and set seek_row_id to the new row id up
  seek_time_ = bag_message->time_stamp;
  seek_row_id_ = std::get<3>(*current_message_row_) + 1;

  ++current_message_row_;
  return bag_message;
}

// bool SqliteStorage::seek_by_index(int32_t index)
// {
//   auto read_statement = database_->prepare_statement(
//     "SELECT data, timestamp, topics.name, messages.id "
//     "FROM messages JOIN topics ON messages.topic_id = topics.id "
//     "WHERE messages.id >= " + std::to_string(index) + " "
//     "ORDER BY messages.timestamp;");
//   modified_message_result_ = read_statement->execute_query<
//     std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int32_t>();
//   modified_current_message_row_ = modified_message_result_.begin();
//   return modified_current_message_row_ != modified_message_result_.end();
// }

// bool SqliteStorage::seek_by_timestamp(rcutils_time_point_value_t timestamp)
// {
//   auto read_statement = database_->prepare_statement(
//     "SELECT data, timestamp, topics.name, messages.id "
//     "FROM messages JOIN topics ON messages.topic_id = topics.id "
//     "WHERE messages.timestamp >= " + std::to_string(timestamp) + " "
//     "ORDER BY messages.timestamp;");
//   modified_message_result_ = read_statement->execute_query<
//     std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int32_t>();
//   modified_current_message_row_ = modified_message_result_.begin();
//   return modified_current_message_row_ != modified_message_result_.end();
// }

std::shared_ptr<SerializedBagMessage>
SqliteStorage::read_at_timestamp(rcutils_time_point_value_t timestamp)
{
  /// prepare for reading
  std::string statement_str = "SELECT data, timestamp, topics.name, messages.id "
                              "FROM messages JOIN topics ON messages.topic_id = topics.id WHERE ";

  // add topic filter
  if (!storage_filter_.topics.empty()) {
    // Construct string for selected topics
    std::string topic_list{""};
    for (auto & topic : storage_filter_.topics) {
      topic_list += "'" + topic + "'";
      if (&topic != &storage_filter_.topics.back()) {
        topic_list += ",";
      }
    }
    statement_str += "(topics.name IN (" + topic_list + ")) AND ";
  }
  // add time filter
  statement_str += "(messages.timestamp = " + std::to_string(timestamp) + ") ";

  // add order by time then id
  statement_str += "ORDER BY messages.timestamp, messages.id;";

  // query data
  read_statement_ = database_->prepare_statement(statement_str);
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  current_message_row_ = message_result_.begin();

  // reset read statement
  read_statement_ = nullptr;

  // early return if no message found
  if (current_message_row_ == message_result_.end()) return nullptr;

  // return the first message in queue
  const auto bag_message = std::make_shared<SerializedBagMessage>();
  bag_message->serialized_data = std::get<0>(*current_message_row_);
  bag_message->time_stamp = std::get<1>(*current_message_row_);
  bag_message->topic_name = std::get<2>(*current_message_row_);
  bag_message->index = std::get<3>(*current_message_row_);
  return bag_message;
}

std::vector<std::shared_ptr<SerializedBagMessage>>
SqliteStorage::read_at_timestamp_range(
  rcutils_time_point_value_t timestamp_begin,
  rcutils_time_point_value_t timestamp_end)
{
  /// prepare for reading
  std::string statement_str = "SELECT data, timestamp, topics.name, messages.id "
                              "FROM messages JOIN topics ON messages.topic_id = topics.id WHERE ";

  // add topic filter
  if (!storage_filter_.topics.empty()) {
    // Construct string for selected topics
    std::string topic_list{""};
    for (auto & topic : storage_filter_.topics) {
      topic_list += "'" + topic + "'";
      if (&topic != &storage_filter_.topics.back()) {
        topic_list += ",";
      }
    }
    statement_str += "(topics.name IN (" + topic_list + ")) AND ";
  }
  // add time filter
  statement_str += "(messages.timestamp BETWEEN " +
                   std::to_string(timestamp_begin) + " AND " +
                   std::to_string(timestamp_end) + ") ";

  // add order by time then id
  statement_str += "ORDER BY messages.timestamp, messages.id;";

  // query data
  read_statement_ = database_->prepare_statement(statement_str);
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  current_message_row_ = message_result_.begin();

  // reset read statement
  read_statement_ = nullptr;

  // return query messages
  std::vector<std::shared_ptr<SerializedBagMessage>> bag_messages;
  for (; current_message_row_ != message_result_.end(); ++current_message_row_) {
    bag_messages.push_back(std::make_shared<SerializedBagMessage>());
    bag_messages.back()->serialized_data = std::get<0>(*current_message_row_);
    bag_messages.back()->time_stamp = std::get<1>(*current_message_row_);
    bag_messages.back()->topic_name = std::get<2>(*current_message_row_);
    bag_messages.back()->index = std::get<3>(*current_message_row_);
  }
  return bag_messages;
}

std::shared_ptr<SerializedBagMessage>
SqliteStorage::read_at_index(int32_t index)
{
  /// prepare for reading
  std::string statement_str = "SELECT data, timestamp, topics.name, messages.id "
                              "FROM messages JOIN topics ON messages.topic_id = topics.id WHERE ";

  // add topic filter
  if (!storage_filter_.topics.empty()) {
    // Construct string for selected topics
    std::string topic_list{""};
    for (auto & topic : storage_filter_.topics) {
      topic_list += "'" + topic + "'";
      if (&topic != &storage_filter_.topics.back()) {
        topic_list += ",";
      }
    }
    statement_str += "(topics.name IN (" + topic_list + ")) AND ";
  }
  // add time filter
  statement_str += "(messages.id = " + std::to_string(index) + ") ";

  // add order by time then id
  statement_str += "ORDER BY messages.id, messages.timestamp;";

  // query data
  read_statement_ = database_->prepare_statement(statement_str);
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  current_message_row_ = message_result_.begin();

  // reset read statement
  read_statement_ = nullptr;

  // early return if no message found
  if (current_message_row_ == message_result_.end()) return nullptr;

  // return the first message in queue
  const auto bag_message = std::make_shared<SerializedBagMessage>();
  bag_message->serialized_data = std::get<0>(*current_message_row_);
  bag_message->time_stamp = std::get<1>(*current_message_row_);
  bag_message->topic_name = std::get<2>(*current_message_row_);
  bag_message->index = std::get<3>(*current_message_row_);
  return bag_message;
}

std::vector<std::shared_ptr<SerializedBagMessage>>
SqliteStorage::read_at_index_range(
  int32_t index_begin,
  int32_t index_end)
{
  /// prepare for reading
  std::string statement_str = "SELECT data, timestamp, topics.name, messages.id "
                              "FROM messages JOIN topics ON messages.topic_id = topics.id WHERE ";

  // add topic filter
  if (!storage_filter_.topics.empty()) {
    // Construct string for selected topics
    std::string topic_list{""};
    for (auto & topic : storage_filter_.topics) {
      topic_list += "'" + topic + "'";
      if (&topic != &storage_filter_.topics.back()) {
        topic_list += ",";
      }
    }
    statement_str += "(topics.name IN (" + topic_list + ")) AND ";
  }
  // add time filter
  statement_str += "(messages.id BETWEEN " +
                   std::to_string(index_begin) + " AND " +
                   std::to_string(index_end) + ") ";

  // add order by time then id
  statement_str += "ORDER BY messages.id, messages.timestamp;";

  // query data
  read_statement_ = database_->prepare_statement(statement_str);
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  current_message_row_ = message_result_.begin();

  // reset read statement
  read_statement_ = nullptr;

  // return query messages
  std::vector<std::shared_ptr<SerializedBagMessage>> bag_messages;
  for (; current_message_row_ != message_result_.end(); ++current_message_row_) {
    bag_messages.push_back(std::make_shared<SerializedBagMessage>());
    bag_messages.back()->serialized_data = std::get<0>(*current_message_row_);
    bag_messages.back()->time_stamp = std::get<1>(*current_message_row_);
    bag_messages.back()->topic_name = std::get<2>(*current_message_row_);
    bag_messages.back()->index = std::get<3>(*current_message_row_);
  }
  return bag_messages;
}

std::vector<TopicMetadata> SqliteStorage::get_all_topics_and_types()
{
  if (all_topics_and_types_.empty()) {
    fill_topics_and_types();
  }

  return all_topics_and_types_;
}

uint64_t SqliteStorage::get_bagfile_size() const
{
  const auto bag_path = rcpputils::fs::path{get_relative_file_path()};

  return bag_path.exists() ? bag_path.file_size() : 0u;
}

void SqliteStorage::initialize()
{
  std::string create_stmt = "CREATE TABLE topics(" \
    "id INTEGER PRIMARY KEY," \
    "name TEXT NOT NULL," \
    "type TEXT NOT NULL," \
    "serialization_format TEXT NOT NULL," \
    "offered_qos_profiles TEXT NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();
  create_stmt = "CREATE TABLE messages(" \
    "id INTEGER PRIMARY KEY," \
    "topic_id INTEGER NOT NULL," \
    "timestamp INTEGER NOT NULL, " \
    "data BLOB NOT NULL);";
  database_->prepare_statement(create_stmt)->execute_and_reset();
  create_stmt = "CREATE INDEX timestamp_idx ON messages (timestamp ASC);";
  database_->prepare_statement(create_stmt)->execute_and_reset();
}

void SqliteStorage::create_topic(const TopicMetadata & topic)
{
  std::lock_guard<std::mutex> db_lock(database_write_mutex_);
  if (topics_.find(topic.name) == std::end(topics_)) {
    auto insert_topic =
      database_->prepare_statement(
      "INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) "
      "VALUES (?, ?, ?, ?)");
    insert_topic->bind(
      topic.name, topic.type, topic.serialization_format, topic.offered_qos_profiles);
    insert_topic->execute_and_reset();
    topics_.emplace(topic.name, static_cast<int>(database_->get_last_insert_id()));
  }
}

void SqliteStorage::remove_topic(const TopicMetadata & topic)
{
  std::lock_guard<std::mutex> db_lock(database_write_mutex_);
  if (topics_.find(topic.name) != std::end(topics_)) {
    auto delete_topic =
      database_->prepare_statement(
      "DELETE FROM topics where name = ? and type = ? and serialization_format = ?");
    delete_topic->bind(topic.name, topic.type, topic.serialization_format);
    delete_topic->execute_and_reset();
    topics_.erase(topic.name);
  }
}

void SqliteStorage::fill_topics_map()
{
  std::lock_guard<std::mutex> db_lock(database_write_mutex_);
  auto query_stmt = database_->prepare_statement("SELECT name, id FROM topics ORDER BY id;");
  auto query_results = query_stmt->execute_query<std::string, int>();
  for (auto result : query_results)
    topics_.emplace(std::get<0>(result), std::get<1>(result));
}

void SqliteStorage::prepare_for_writing()
{
  insert_statement_ = database_->prepare_statement(
    "INSERT INTO messages (timestamp, topic_id, data) VALUES (?, ?, ?);");
  update_statement_ = database_->prepare_statement(
    "UPDATE messages SET timestamp = ?, topic_id = ?, data = ? WHERE (id = ?);");
}

void SqliteStorage::prepare_for_reading()
{
  std::string statement_str = "SELECT data, timestamp, topics.name, messages.id "
    "FROM messages JOIN topics ON messages.topic_id = topics.id WHERE ";

  // add topic filter
  if (!storage_filter_.topics.empty()) {
    // Construct string for selected topics
    std::string topic_list{""};
    for (auto & topic : storage_filter_.topics) {
      topic_list += "'" + topic + "'";
      if (&topic != &storage_filter_.topics.back()) {
        topic_list += ",";
      }
    }
    statement_str += "(topics.name IN (" + topic_list + ")) AND ";
  }
  // add start time filter
  statement_str += "(((timestamp = " + std::to_string(seek_time_) + ") "
    "AND (messages.id >= " + std::to_string(seek_row_id_) + ")) "
    "OR (timestamp > " + std::to_string(seek_time_) + ")) ";

  // add order by time then id
  statement_str += "ORDER BY messages.timestamp, messages.id;";

  read_statement_ = database_->prepare_statement(statement_str);
  message_result_ = read_statement_->execute_query<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int>();
  current_message_row_ = message_result_.begin();
}

void SqliteStorage::fill_topics_and_types()
{
  auto statement = database_->prepare_statement(
    "SELECT name, type, serialization_format FROM topics ORDER BY id;");
  auto query_results = statement->execute_query<std::string, std::string, std::string>();

  for (auto result : query_results) {
    all_topics_and_types_.push_back(
      {std::get<0>(result), std::get<1>(result), std::get<2>(result), ""});
  }
}

std::string SqliteStorage::get_storage_identifier() const
{
  return "sqlite3";
}

std::string SqliteStorage::get_relative_file_path() const
{
  return relative_path_;
}

uint64_t SqliteStorage::get_minimum_split_file_size() const
{
  return MIN_SPLIT_FILE_SIZE;
}

BagMetadata SqliteStorage::get_metadata()
{
  BagMetadata metadata;
  metadata.storage_identifier = get_storage_identifier();
  metadata.relative_file_paths = {get_relative_file_path()};

  metadata.message_count = 0;
  metadata.topics_with_message_count = {};

  // get all topics - note: we need this because some topics may not have message associated
  using RowType = std::tuple<std::string, std::string, std::string, std::string>;
  std::unordered_map<std::string, RowType> topic_name2metadata;
  {
    auto statement = database_->prepare_statement(
      "SELECT name, type, serialization_format, offered_qos_profiles FROM topics ORDER BY id;");
    auto query_results = statement->execute_query<std::string, std::string, std::string, std::string>();
    for (auto result : query_results)
      topic_name2metadata.emplace(std::get<0>(result), result);
  }

  auto statement = database_->prepare_statement(
    "SELECT name, type, serialization_format, COUNT(messages.id), MIN(messages.timestamp), "
    "MAX(messages.timestamp), offered_qos_profiles "
    "FROM messages JOIN topics on topics.id = messages.topic_id "
    "GROUP BY topics.name;");
  auto query_results = statement->execute_query<
    std::string, std::string, std::string, int, rcutils_time_point_value_t,
    rcutils_time_point_value_t, std::string>();

  rcutils_time_point_value_t min_time = INT64_MAX;
  rcutils_time_point_value_t max_time = 0;
  for (auto result : query_results) {
    metadata.topics_with_message_count.push_back(
      {
        {std::get<0>(result), std::get<1>(result), std::get<2>(result), std::get<6>(result)},
        static_cast<size_t>(std::get<3>(result))
      });

    metadata.message_count += std::get<3>(result);
    min_time = std::get<4>(result) < min_time ? std::get<4>(result) : min_time;
    max_time = std::get<5>(result) > max_time ? std::get<5>(result) : max_time;

    topic_name2metadata.erase(std::get<0>(result));
  }

  // add topics that have no messages
  for (const auto &empty_entry: topic_name2metadata) {
    const auto &info = empty_entry.second;
    metadata.topics_with_message_count.push_back(
      {
        {std::get<0>(info), std::get<1>(info), std::get<2>(info), std::get<3>(info)},
        static_cast<size_t>(0)
      });
  }

  if (metadata.message_count == 0) {
    min_time = 0;
    max_time = 0;
  }

  metadata.starting_time =
    std::chrono::time_point<std::chrono::high_resolution_clock>(std::chrono::nanoseconds(min_time));
  metadata.duration = std::chrono::nanoseconds(max_time) - std::chrono::nanoseconds(min_time);
  metadata.bag_size = get_bagfile_size();

  return metadata;
}

void SqliteStorage::set_filter(
  const StorageFilter & storage_filter)
{
  // keep current start time and start row_id
  // set topic filter and reset read statement for re-read
  storage_filter_ = storage_filter;
  read_statement_ = nullptr;
}

void SqliteStorage::reset_filter()
{
  set_filter(StorageFilter());
}

void SqliteStorage::seek(const rcutils_time_point_value_t & timestamp)
{
  // reset row id to 0 and set start time to input
  // keep topic filter and reset read statement for re-read
  seek_row_id_ = 0;
  seek_time_ = timestamp;
  read_statement_ = nullptr;
}

// clang-format on

}  // namespace sqlite
}  // namespace storage
}  // namespace vtr