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
 * \file data_stream_reader.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_storage/stream/data_stream_reader.hpp>

namespace vtr {
namespace storage {

template <typename MessageType>
DataStreamReaderMoreSpecificBase<MessageType>::DataStreamReaderMoreSpecificBase(
    const std::string &base_directory, const std::string &stream_name)
    : DataStreamReaderBase(base_directory, stream_name) {}

template <typename MessageType>
std::shared_ptr<VTRMessage>
DataStreamReaderMoreSpecificBase<MessageType>::convertBagMessage(
    std::shared_ptr<SerializedBagMessage> bag_message) {
  if (!bag_message) return nullptr;

  auto extracted_msg = std::make_shared<MessageType>();
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::SerializedMessage extracted_serialized_msg(
      *bag_message->serialized_data);
  serialization_.deserialize_message(&extracted_serialized_msg,
                                     extracted_msg.get());

  auto anytype_msg = std::make_shared<VTRMessage>(*extracted_msg);
  anytype_msg->set_index(bag_message->database_index);
  if (bag_message->time_stamp != NO_TIMESTAMP_VALUE)
    anytype_msg->set_timestamp(bag_message->time_stamp);

  return anytype_msg;
}

template <typename MessageType, typename CalibrationType>
std::shared_ptr<VTRMessage>
DataStreamReader<MessageType, CalibrationType>::fetchCalibration() {
  if (calibration_msg_) return calibration_msg_;

  if (!(this->base_directory_ / CALIBRATION_FOLDER).exists())
    throw NoBagExistsException(this->base_directory_ / CALIBRATION_FOLDER);

  calibration_reader_ =
      std::make_shared<accessor::RandomAccessReader>(CALIBRATION_FOLDER);

  calibration_reader_->open(
      (this->base_directory_ / CALIBRATION_FOLDER).string());
  rclcpp::Serialization<CalibrationType> calibration_serialization;

  auto bag_message = calibration_reader_->read_at_index(1);

  if (!bag_message)
    throw std::runtime_error("calibration database has no messages!");

  auto extracted_msg = std::make_shared<CalibrationType>();
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::SerializedMessage extracted_serialized_msg(
      *bag_message->serialized_data);
  calibration_serialization.deserialize_message(&extracted_serialized_msg,
                                                extracted_msg.get());

  calibration_msg_ = std::make_shared<VTRMessage>(*extracted_msg);
  calibration_msg_->set_index(bag_message->database_index);
  if (bag_message->time_stamp != NO_TIMESTAMP_VALUE)
    calibration_msg_->set_timestamp(bag_message->time_stamp);

  return calibration_msg_;
}

}  // namespace storage
}  // namespace vtr