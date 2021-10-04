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
 * \file data_stream_accessor.hpp
 * \brief DataStreamAccessor(Base) class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <filesystem>

#include <boost/thread.hpp>  // std::lock that takes iterator input

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "vtr_storage/accessor/storage_accessor.hpp"
#include "vtr_storage/stream/message.hpp"

namespace vtr {
namespace storage {

class DataStreamAccessorBase {
 public:
  using Index = int;
  using Timestamp = rcutils_time_point_value_t;

  DataStreamAccessorBase(const std::string &base_directory,
                         const std::string &stream_name,
                         const std::string &stream_type);
  virtual ~DataStreamAccessorBase() = default;

  // returns a nullptr if no data exist at the specified index/timestamp
  std::shared_ptr<LockableMessage> readAtIndex(Index index);
  std::shared_ptr<LockableMessage> readAtTimestamp(Timestamp time);
  // returns an empty vector if no data exist at the specified range
  std::vector<std::shared_ptr<LockableMessage>> readAtIndexRange(
      Index index_begin, Index index_end);
  std::vector<std::shared_ptr<LockableMessage>> readAtTimestampRange(
      Timestamp timestamp_begin, Timestamp timestamp_end);

  virtual void write(const std::shared_ptr<LockableMessage> &message) = 0;
  virtual void write(
      const std::vector<std::shared_ptr<LockableMessage>> &messages) = 0;

 private:
  virtual std::shared_ptr<LockableMessage> deserializeMessage(
      const std::shared_ptr<SerializedBagMessage> &message) = 0;

 protected:
  std::unique_ptr<StorageAccessor> storage_accessor_ =
      std::make_unique<StorageAccessor>();
  TopicMetadata tm_;

 private:
  std::filesystem::path base_directory_;
  std::filesystem::path data_directory_;
};

template <typename DataType>
class DataStreamAccessor : public DataStreamAccessorBase {
 public:
  DataStreamAccessor(const std::string &base_directory,
                     const std::string &stream_name = "",
                     const std::string &stream_type = "UnknownType")
      : DataStreamAccessorBase(base_directory, stream_name, stream_type) {}

  void write(const std::shared_ptr<LockableMessage> &message) override;
  void write(
      const std::vector<std::shared_ptr<LockableMessage>> &messages) override;

 private:
  std::shared_ptr<LockableMessage> deserializeMessage(
      const std::shared_ptr<SerializedBagMessage> &serialized) override;

  rclcpp::Serialization<DataType> serialization_;
};

template <typename DataType>
void DataStreamAccessor<DataType>::write(
    const std::shared_ptr<LockableMessage> &message) {
  const auto message_locked = message->locked();
  auto &message_ref = message_locked.get();

  if (message_ref.getSaved() == true) return;

  const auto serialized = std::make_shared<SerializedBagMessage>();
  serialized->time_stamp = message_ref.getTimestamp();
  serialized->index = message_ref.getIndex();
  serialized->topic_name = tm_.name;

  const auto data_ptr = message_ref.getDataPtr<DataType>();
  rclcpp::SerializedMessage serialized_data;
  serialization_.serialize_message(data_ptr.get(), &serialized_data);
  // temporary store the payload in a shared_ptr.
  // add custom no-op deleter to avoid deep copying data.
  serialized->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      const_cast<rcutils_uint8_array_t *>(
          &serialized_data.get_rcl_serialized_message()),
      [](rcutils_uint8_array_t * /* data */) {});

  storage_accessor_->write(serialized);

  // the index should be set after insertion
  message_ref.setIndex(serialized->index);
}

template <typename DataType>
void DataStreamAccessor<DataType>::write(
    const std::vector<std::shared_ptr<LockableMessage>> &messages) {
  // do not make use of the vector write for simplicity
  for (const auto &message : messages) write(message);
#if false
  // lock all messages - note: this block is prepared for writing a vector of
  // messages with potentially better performance
  using LockType = std::shared_lock<std::shared_mutex>;
  std::vector<LockType> locks;
  locks.reserve(messages.size());
  for (const auto &message : messages)
    locks.emplace_back(message->mutex(), std::defer_lock);
  boost::lock(locks.begin(), locks.end());
#endif
}

template <typename DataType>
std::shared_ptr<LockableMessage>
DataStreamAccessor<DataType>::deserializeMessage(
    const std::shared_ptr<SerializedBagMessage> &serialized) {
  if (!serialized) return nullptr;

  auto data = std::make_shared<DataType>();
  rclcpp::SerializedMessage extracted_data(*serialized->serialized_data);
  serialization_.deserialize_message(&extracted_data, data.get());

  auto deserialized = std::make_shared<LockableMessage>(
      data, serialized->time_stamp, serialized->index);

  return deserialized;
}

}  // namespace storage
}  // namespace vtr
