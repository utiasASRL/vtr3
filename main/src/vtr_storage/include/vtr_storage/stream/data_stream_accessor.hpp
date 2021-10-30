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
#include "vtr_storage/stream/type_traits.hpp"

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

  // returns a nullptr if no data exist at the specified index/timestamp
  std::shared_ptr<LockableMessage<DataType>> readAtIndex(Index index);
  std::shared_ptr<LockableMessage<DataType>> readAtTimestamp(Timestamp time);
  // returns an empty vector if no data exist at the specified range
  std::vector<std::shared_ptr<LockableMessage<DataType>>> readAtIndexRange(
      Index index_begin, Index index_end);
  std::vector<std::shared_ptr<LockableMessage<DataType>>> readAtTimestampRange(
      Timestamp timestamp_begin, Timestamp timestamp_end);

  template <typename T = DataType>
  typename std::enable_if<!is_storable<T>::value, void>::type write(
      const std::shared_ptr<LockableMessage<DataType>> &message);

  template <typename T = DataType>
  typename std::enable_if<is_storable<T>::value, void>::type write(
      const std::shared_ptr<LockableMessage<DataType>> &message);

  void write(
      const std::vector<std::shared_ptr<LockableMessage<DataType>>> &messages);

 private:
  template <typename T = DataType>
  typename std::enable_if<!is_storable<T>::value,
                          std::shared_ptr<LockableMessage<DataType>>>::type
  deserializeMessage(const std::shared_ptr<SerializedBagMessage> &serialized);

  template <typename T = DataType>
  typename std::enable_if<is_storable<T>::value,
                          std::shared_ptr<LockableMessage<DataType>>>::type
  deserializeMessage(const std::shared_ptr<SerializedBagMessage> &serialized);

  rclcpp::Serialization<typename has_to_storable<DataType>::type>
      serialization_;
};

template <typename DataType>
std::shared_ptr<LockableMessage<DataType>>
DataStreamAccessor<DataType>::readAtIndex(Index index) {
  const auto serialized_message = storage_accessor_->read_at_index(index);
  return deserializeMessage(serialized_message);
}

template <typename DataType>
std::shared_ptr<LockableMessage<DataType>>
DataStreamAccessor<DataType>::readAtTimestamp(Timestamp timestamp) {
  const auto serialized_message =
      storage_accessor_->read_at_timestamp(timestamp);
  return deserializeMessage(serialized_message);
}

template <typename DataType>
std::vector<std::shared_ptr<LockableMessage<DataType>>>
DataStreamAccessor<DataType>::readAtIndexRange(Index index_begin,
                                               Index index_end) {
  const auto serialized_messages =
      storage_accessor_->read_at_index_range(index_begin, index_end);

  std::vector<std::shared_ptr<LockableMessage<DataType>>> messages;
  messages.reserve(serialized_messages.size());
  for (const auto &serialized_message : serialized_messages)
    messages.push_back(deserializeMessage(serialized_message));

  return messages;
}

template <typename DataType>
std::vector<std::shared_ptr<LockableMessage<DataType>>>
DataStreamAccessor<DataType>::readAtTimestampRange(Timestamp timestamp_begin,
                                                   Timestamp timestamp_end) {
  const auto serialized_messages = storage_accessor_->read_at_timestamp_range(
      timestamp_begin, timestamp_end);

  std::vector<std::shared_ptr<LockableMessage<DataType>>> messages;
  messages.reserve(serialized_messages.size());
  for (const auto &serialized_message : serialized_messages)
    messages.push_back(deserializeMessage(serialized_message));

  return messages;
}

template <typename DataType>
template <typename T>
typename std::enable_if<!is_storable<T>::value, void>::type
DataStreamAccessor<DataType>::write(
    const std::shared_ptr<LockableMessage<DataType>> &message) {
  const auto message_locked = message->locked();
  auto &message_ref = message_locked.get();

  if (message_ref.getSaved() == true) return;

  const auto serialized = std::make_shared<SerializedBagMessage>();
  serialized->time_stamp = message_ref.getTimestamp();
  serialized->index = message_ref.getIndex();
  serialized->topic_name = tm_.name;

  const auto &data = message_ref.getData();
  rclcpp::SerializedMessage serialized_data;
  serialization_.serialize_message(&data, &serialized_data);
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
template <typename T>
typename std::enable_if<is_storable<T>::value, void>::type
DataStreamAccessor<DataType>::write(
    const std::shared_ptr<LockableMessage<DataType>> &message) {
  const auto message_locked = message->locked();
  auto &message_ref = message_locked.get();

  if (message_ref.getSaved() == true) return;

  const auto serialized = std::make_shared<SerializedBagMessage>();
  serialized->time_stamp = message_ref.getTimestamp();
  serialized->index = message_ref.getIndex();
  serialized->topic_name = tm_.name;

  const auto &data = message_ref.getData();
  const auto storable = data.toStorable();
  rclcpp::SerializedMessage serialized_data;
  serialization_.serialize_message(&storable, &serialized_data);
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
    const std::vector<std::shared_ptr<LockableMessage<DataType>>> &messages) {
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
template <typename T>
typename std::enable_if<!is_storable<T>::value,
                        std::shared_ptr<LockableMessage<DataType>>>::type
DataStreamAccessor<DataType>::deserializeMessage(
    const std::shared_ptr<SerializedBagMessage> &serialized) {
  if (!serialized) return nullptr;

  auto data = std::make_shared<DataType>();
  rclcpp::SerializedMessage extracted_data(*serialized->serialized_data);
  serialization_.deserialize_message(&extracted_data, data.get());

  auto deserialized = std::make_shared<LockableMessage<DataType>>(
      data, serialized->time_stamp, serialized->index);

  return deserialized;
}

template <typename DataType>
template <typename T>
typename std::enable_if<is_storable<T>::value,
                        std::shared_ptr<LockableMessage<DataType>>>::type
DataStreamAccessor<DataType>::deserializeMessage(
    const std::shared_ptr<SerializedBagMessage> &serialized) {
  if (!serialized) return nullptr;

  typename has_to_storable<DataType>::type data;
  rclcpp::SerializedMessage extracted_data(*serialized->serialized_data);
  serialization_.deserialize_message(&extracted_data, &data);

  auto deserialized = std::make_shared<LockableMessage<DataType>>(
      DataType::fromStorable(data), serialized->time_stamp, serialized->index);

  return deserialized;
}

}  // namespace storage
}  // namespace vtr
