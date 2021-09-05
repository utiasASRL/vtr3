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
 * \file data_stream_reader.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <utility>

#include "rclcpp/serialization.hpp"

#include <vtr_storage/accessor/random_access_reader.hpp>
#include <vtr_storage/stream/data_stream_base.hpp>
#include <vtr_storage/stream/message.hpp>
#include <vtr_storage/stream/utils.hpp>

#include <vtr_messages/msg/rig_calibration.hpp>

namespace vtr {
namespace storage {

class DataStreamReaderBase : public DataStreamBase {
 public:
  DataStreamReaderBase(const std::string &base_directory,
                       const std::string &stream_name = "")
      : DataStreamBase(base_directory, stream_name) {}
  virtual ~DataStreamReaderBase();

  void open();
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
      std::shared_ptr<SerializedBagMessage> bag_message) = 0;
  std::shared_ptr<accessor::RandomAccessReader> reader_;
  bool seeked_ = false;
};

template <typename MessageType>
class DataStreamReaderMoreSpecificBase : public DataStreamReaderBase {
 public:
  DataStreamReaderMoreSpecificBase(const std::string &base_directory,
                                   const std::string &stream_name = "");

 protected:
  std::shared_ptr<VTRMessage> convertBagMessage(
      std::shared_ptr<SerializedBagMessage> bag_message) override;

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
  std::shared_ptr<accessor::RandomAccessReader> calibration_reader_ = nullptr;
  std::shared_ptr<VTRMessage> calibration_msg_ = nullptr;
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

#include "vtr_storage/stream/data_stream_reader.inl"
