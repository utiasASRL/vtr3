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
 * \file data_stream_accessor.cpp
 * \brief DataStreamAccessor(Base) class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_storage/stream/data_stream_accessor.hpp>

namespace vtr {
namespace storage {

DataStreamAccessorBase::DataStreamAccessorBase(
    const std::string &base_directory, const std::string &stream_name,
    const std::string &stream_type)
    : base_directory_(std::filesystem::path(base_directory)),
      data_directory_(std::filesystem::path(base_directory) / stream_name) {
  storage_accessor_->open(data_directory_.string());

  tm_.name = stream_name;
  tm_.type = stream_type;
  tm_.serialization_format = "cdr";
  storage_accessor_->create_topic(tm_);
}

std::shared_ptr<LockableMessage> DataStreamAccessorBase::readAtIndex(
    Index index) {
  const auto serialized_message = storage_accessor_->read_at_index(index);
  return deserializeMessage(serialized_message);
}

std::shared_ptr<LockableMessage> DataStreamAccessorBase::readAtTimestamp(
    Timestamp timestamp) {
  const auto serialized_message =
      storage_accessor_->read_at_timestamp(timestamp);
  return deserializeMessage(serialized_message);
}

std::vector<std::shared_ptr<LockableMessage>>
DataStreamAccessorBase::readAtIndexRange(Index index_begin, Index index_end) {
  const auto serialized_messages =
      storage_accessor_->read_at_index_range(index_begin, index_end);

  std::vector<std::shared_ptr<LockableMessage>> messages;
  messages.reserve(serialized_messages.size());
  for (const auto &serialized_message : serialized_messages)
    messages.push_back(deserializeMessage(serialized_message));

  return messages;
}

std::vector<std::shared_ptr<LockableMessage>>
DataStreamAccessorBase::readAtTimestampRange(Timestamp timestamp_begin,
                                             Timestamp timestamp_end) {
  const auto serialized_messages = storage_accessor_->read_at_timestamp_range(
      timestamp_begin, timestamp_end);

  std::vector<std::shared_ptr<LockableMessage>> messages;
  messages.reserve(serialized_messages.size());
  for (const auto &serialized_message : serialized_messages)
    messages.push_back(deserializeMessage(serialized_message));

  return messages;
}

}  // namespace storage
}  // namespace vtr