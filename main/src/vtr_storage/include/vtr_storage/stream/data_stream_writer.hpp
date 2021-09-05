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
 * \file data_stream_writer.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <any>
#include <utility>

#include "rclcpp/serialization.hpp"

#include <vtr_storage/accessor/sequential_append_writer.hpp>
#include <vtr_storage/stream/data_stream_base.hpp>
#include <vtr_storage/stream/message.hpp>
#include <vtr_storage/stream/utils.hpp>

#include <vtr_messages/msg/rig_calibration.hpp>

namespace vtr {
namespace storage {

class DataStreamWriterBase : public DataStreamBase {
 public:
  DataStreamWriterBase(const std::string &base_directory,
                       const std::string &stream_name = "", bool append = false)
      : DataStreamBase(base_directory, stream_name), append_(append) {}
  virtual ~DataStreamWriterBase(){};

  virtual void open() = 0;
  virtual void close() = 0;
  virtual int32_t write(const VTRMessage &anytype_message) = 0;

 protected:
  virtual TopicMetadata createTopicMetadata() = 0;

  bool append_;
};

template <typename MessageType>
class DataStreamWriter : public DataStreamWriterBase {
 public:
  DataStreamWriter(const std::string &base_directory,
                   const std::string &stream_name = "", bool append = false);
  ~DataStreamWriter();

  void open() override;
  void close() override;

  // returns the id of the inserted message
  int32_t write(const VTRMessage &vtr_message) override;

 protected:
  TopicMetadata createTopicMetadata() override;

  rclcpp::Serialization<MessageType> serialization_;
  TopicMetadata tm_;
  std::shared_ptr<accessor::SequentialAppendWriter> writer_;
};

class DataStreamWriterCalibration
    : public DataStreamWriter<vtr_messages::msg::RigCalibration> {
 public:
  DataStreamWriterCalibration(const std::string &base_directory)
      : DataStreamWriter(base_directory, CALIBRATION_FOLDER, false) {}
};

}  // namespace storage
}  // namespace vtr

#include "vtr_storage/stream/data_stream_writer.inl"
