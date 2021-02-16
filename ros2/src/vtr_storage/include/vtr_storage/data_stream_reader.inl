#pragma once

#include "vtr_storage/data_stream_reader.hpp"

namespace vtr {
namespace storage {

template <typename MessageType>
DataStreamReaderMoreSpecificBase<MessageType>::DataStreamReaderMoreSpecificBase(
    const std::string &data_directory, const std::string &stream_name)
    : DataStreamReaderBase(data_directory, stream_name) {}

template <typename MessageType>
std::shared_ptr<VTRMessage>
DataStreamReaderMoreSpecificBase<MessageType>::convertBagMessage(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message) {
  std::shared_ptr<VTRMessage> anytype_msg;
  if (bag_message) {
    auto extracted_msg = std::make_shared<MessageType>();
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::SerializedMessage extracted_serialized_msg(
        *bag_message->serialized_data);
    this->serialization_.deserialize_message(&extracted_serialized_msg,
                                             extracted_msg.get());

    anytype_msg = std::make_shared<VTRMessage>(*extracted_msg);
    anytype_msg->set_index(bag_message->database_index);
    if (bag_message->time_stamp != NO_TIMESTAMP_VALUE) {
      anytype_msg->set_timestamp(bag_message->time_stamp);
    }
  }
  return anytype_msg;
}

template <typename MessageType, typename CalibrationType>
std::shared_ptr<VTRMessage>
DataStreamReader<MessageType, CalibrationType>::fetchCalibration() {
  if (!calibration_fetched_) {
    if (!(this->base_directory_ / CALIBRATION_FOLDER).exists()) {
      throw NoBagExistsException(this->base_directory_ / CALIBRATION_FOLDER);
    }
    calibration_reader_ =
        std::make_shared<RandomAccessReader>(CALIBRATION_FOLDER);

    rosbag2_cpp::StorageOptions calibration_storage_options =
        this->storage_options_;
    calibration_storage_options.uri =
        (this->base_directory_ / CALIBRATION_FOLDER).string();
    calibration_reader_->open(calibration_storage_options,
                              this->converter_options_);
    rclcpp::Serialization<CalibrationType> calibration_serialization;

    auto bag_message = calibration_reader_->read_at_index(1);

    if (bag_message) {
      auto extracted_msg = std::make_shared<CalibrationType>();
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::SerializedMessage extracted_serialized_msg(
          *bag_message->serialized_data);
      calibration_serialization.deserialize_message(&extracted_serialized_msg,
                                                    extracted_msg.get());
      calibration_fetched_ = true;
      auto anytype_msg = std::make_shared<VTRMessage>(*extracted_msg);
      anytype_msg->set_index(bag_message->database_index);
      if (bag_message->time_stamp != NO_TIMESTAMP_VALUE) {
        anytype_msg->set_timestamp(bag_message->time_stamp);
      }
      calibration_msg_ = anytype_msg;
    } else {
      throw std::runtime_error("calibration database has no messages!");
    }
  }
  return calibration_msg_;
}

}  // namespace storage
}  // namespace vtr