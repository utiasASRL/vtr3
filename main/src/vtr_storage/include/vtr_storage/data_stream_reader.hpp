#pragma once

#include <utility>
#include <vtr_messages/msg/rig_calibration.hpp>
#include "vtr_storage/data_stream_base.hpp"
#include "vtr_storage/message.hpp"
#include "vtr_storage/random_access_reader.hpp"

namespace vtr {
namespace storage {

class DataStreamReaderBase : public DataStreamBase {
 public:
  DataStreamReaderBase(const std::string &data_directory,
                       const std::string &stream_name = "")
      : DataStreamBase(data_directory, stream_name) {}
  virtual ~DataStreamReaderBase();

  void openAndGetMessageType();
  void close();

  virtual std::shared_ptr<VTRMessage> fetchCalibration() = 0;

  // returns a nullptr if no data exist at the specified index/timestamp
  std::shared_ptr<VTRMessage> readAtIndex(int32_t index);
  std::shared_ptr<VTRMessage> readAtTimestamp(rcutils_time_point_value_t time);
  // returns an empty vector if no data exist at the specified range
  std::shared_ptr<std::vector<std::shared_ptr<VTRMessage>>> readAtIndexRange(
      int32_t index_begin, int32_t index_end);
  std::shared_ptr<std::vector<std::shared_ptr<VTRMessage>>>
  readAtTimestampRange(rcutils_time_point_value_t time_begin,
                       rcutils_time_point_value_t time_end);

  bool seekByIndex(int32_t index);
  bool seekByTimestamp(rcutils_time_point_value_t timestamp);
  std::shared_ptr<VTRMessage> readNextFromSeek();

 protected:
  virtual std::shared_ptr<VTRMessage> convertBagMessage(
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message) = 0;
  std::shared_ptr<RandomAccessReader> reader_;
  bool seeked_ = false;
};

template <typename MessageType>
class DataStreamReaderMoreSpecificBase : public DataStreamReaderBase {
 public:
  DataStreamReaderMoreSpecificBase(const std::string &data_directory,
                                   const std::string &stream_name = "");

 protected:
  std::shared_ptr<VTRMessage> convertBagMessage(
      std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message)
      override;

  rclcpp::Serialization<MessageType> serialization_;
};

template <typename MessageType, typename CalibrationType = NoCalibration>
class DataStreamReader : public DataStreamReaderMoreSpecificBase<MessageType> {
 public:
  DataStreamReader(const std::string &data_directory,
                   const std::string &stream_name = "")
      : DataStreamReaderMoreSpecificBase<MessageType>(data_directory,
                                                      stream_name) {}

  std::shared_ptr<VTRMessage> fetchCalibration() override;

 protected:
  rclcpp::Serialization<CalibrationType> calibration_serialization_;
  std::shared_ptr<RandomAccessReader> calibration_reader_;
  bool calibration_fetched_ = false;
  std::shared_ptr<VTRMessage> calibration_msg_;
};

template <typename MessageType>
class DataStreamReader<MessageType, NoCalibration>
    : public DataStreamReaderMoreSpecificBase<MessageType> {
 public:
  DataStreamReader(const std::string &data_directory,
                   const std::string &stream_name = "")
      : DataStreamReaderMoreSpecificBase<MessageType>(data_directory,
                                                      stream_name) {}

  std::shared_ptr<VTRMessage> fetchCalibration() override {
    assert(
        false &&
        "Attempted to fetchCalibration() calibration for NoCalibration type!");
    return nullptr;  // for suppressing warnings
  }
};
}  // namespace storage
}  // namespace vtr

#include "vtr_storage/data_stream_reader.inl"
