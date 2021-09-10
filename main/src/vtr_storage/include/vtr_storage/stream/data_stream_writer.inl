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
 * \file data_stream_writer.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_storage/stream/data_stream_writer.hpp>

namespace vtr {
namespace storage {

template <typename MessageType>
DataStreamWriter<MessageType>::DataStreamWriter(
    const std::string &base_directory, const std::string &stream_name,
    bool append)
    : DataStreamWriterBase(base_directory, stream_name, append) {
  tm_ = createTopicMetadata();
}

template <typename MessageType>
DataStreamWriter<MessageType>::~DataStreamWriter() {
  close();
}

template <typename MessageType>
void DataStreamWriter<MessageType>::open() {
  if (opened_) return;

  writer_ = std::make_shared<accessor::SequentialAppendWriter>(append_);
  writer_->open(data_directory_.string());
  if (!append_) writer_->create_topic(tm_);
  opened_ = true;
}

template <typename MessageType>
void DataStreamWriter<MessageType>::close() {
  writer_.reset();
  opened_ = false;
}

template <typename MessageType>
TopicMetadata DataStreamWriter<MessageType>::createTopicMetadata() {
  /// \todo create topic based on topic name
  TopicMetadata tm;
  tm.name = "/my/test/topic";
  tm.type = "test_msgs/msg/BasicTypes";
  tm.serialization_format = "cdr";
  return tm;
}

template <typename MessageType>
int32_t DataStreamWriter<MessageType>::write(const VTRMessage &vtr_message) {
  if (vtr_message.has_index()) {
    // index assigned for the message should be sequential
    throw std::runtime_error("Written message has an index assigned!");
  }
  if (!opened_) open();
  auto message = vtr_message.template get<MessageType>();
  auto bag_message = std::make_shared<SerializedBagMessage>();
  if (vtr_message.has_timestamp()) {
    bag_message->time_stamp = vtr_message.get_timestamp();
  } else {
    bag_message->time_stamp = NO_TIMESTAMP_VALUE;
  }
  rclcpp::SerializedMessage serialized_msg;
  serialization_.serialize_message(&message, &serialized_msg);

  bag_message->topic_name = tm_.name;
  bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      &serialized_msg.get_rcl_serialized_message(),
      [](rcutils_uint8_array_t * /* data */) {});

  writer_->write(bag_message);
  return writer_->get_last_inserted_id();
}

}  // namespace storage
}  // namespace vtr