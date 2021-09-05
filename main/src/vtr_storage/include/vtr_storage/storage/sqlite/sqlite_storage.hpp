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
 * \file sqlite_storage.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rcutils/types.h"

#include "vtr_storage/serialized_bag_message.hpp"
#include "vtr_storage/storage/read_write_interface.hpp"
#include "vtr_storage/storage/sqlite/sqlite_wrapper.hpp"
#include "vtr_storage/storage/storage_filter.hpp"
#include "vtr_storage/storage/topic_metadata.hpp"

namespace vtr {
namespace storage {
namespace sqlite_storage {

/// \note the following code is adapted from rosbag2 foxy

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

class SqliteStorage
  : public storage_interfaces::ReadWriteInterface
{
public:
  SqliteStorage() = default;

  ~SqliteStorage() override;

  void open(
    const std::string & uri,
    storage_interfaces::IOFlag io_flag =
    storage_interfaces::IOFlag::READ_WRITE) override;

  void remove_topic(const TopicMetadata & topic) override;

  void create_topic(const TopicMetadata & topic) override;

  void write(std::shared_ptr<const SerializedBagMessage> message) override;

  void write(
    const std::vector<std::shared_ptr<const SerializedBagMessage>> & messages)
  override;

  bool has_next() override;

  std::shared_ptr<SerializedBagMessage> read_next() override;

  std::shared_ptr<SerializedBagMessage>
  read_at_timestamp(rcutils_time_point_value_t timestamp) override;

  std::shared_ptr<SerializedBagMessage> read_at_index(int32_t index) override;

  std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
  read_at_timestamp_range(
    rcutils_time_point_value_t timestamp_begin,
    rcutils_time_point_value_t timestamp_end) override;

  std::shared_ptr<std::vector<std::shared_ptr<SerializedBagMessage>>>
  read_at_index_range(int32_t index_begin, int32_t index_end) override;

  int32_t get_last_inserted_id() override;

  std::vector<TopicMetadata> get_all_topics_and_types() override;

  BagMetadata get_metadata() override;

  std::string get_relative_file_path() const override;

  uint64_t get_bagfile_size() const override;

  std::string get_storage_identifier() const override;

  uint64_t get_minimum_split_file_size() const override;

  void set_filter(const StorageFilter & storage_filter) override;

  void reset_filter() override;

  bool seek_by_index(int32_t index) override;

  bool seek_by_timestamp(rcutils_time_point_value_t timestamp) override;

  std::shared_ptr<SerializedBagMessage> modified_read_next() override;

private:
  void initialize();
  void prepare_for_writing();
  void prepare_for_reading();
  void fill_topics_and_types();
  void activate_transaction();
  void commit_transaction();

  using ReadQueryResult = SqliteStatementWrapper::QueryResult<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string>;
  using ModifiedReadQueryResult = SqliteStatementWrapper::QueryResult<
    std::shared_ptr<rcutils_uint8_array_t>, rcutils_time_point_value_t, std::string, int32_t>;

  std::shared_ptr<SqliteWrapper> database_;
  SqliteStatement write_statement_ {};
  SqliteStatement read_statement_ {};
  ReadQueryResult message_result_ {nullptr};
  ModifiedReadQueryResult modified_message_result_ {nullptr};
  ReadQueryResult::Iterator current_message_row_ {nullptr, SqliteStatementWrapper::QueryResult<>::Iterator::POSITION_END};
  ModifiedReadQueryResult::Iterator modified_current_message_row_ {nullptr, SqliteStatementWrapper::QueryResult<>::Iterator::POSITION_END};
  std::unordered_map<std::string, int> topics_;
  std::vector<TopicMetadata> all_topics_and_types_;
  std::string relative_path_;
  std::atomic_bool active_transaction_ {false};
  StorageFilter storage_filter_ {};
};

// clang-format on

}  // namespace sqlite_storage
}  // namespace storage
}  // namespace vtr
