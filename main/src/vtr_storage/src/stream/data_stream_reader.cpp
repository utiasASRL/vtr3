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
 * \file data_stream_reader.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_storage/stream/data_stream_reader.hpp>

namespace vtr {
namespace storage {

DataStreamReaderBase::~DataStreamReaderBase() { close(); }

void DataStreamReaderBase::close() {
  reader_.reset();
  opened_ = false;
  seeked_ = false;
}

void DataStreamReaderBase::open() {
  if (opened_) return;

  if (!data_directory_.exists()) throw NoBagExistsException(data_directory_);

  reader_ = std::make_shared<accessor::RandomAccessReader>(stream_name_);
  reader_->open(data_directory_.string());
  opened_ = true;
}

std::shared_ptr<VTRMessage> DataStreamReaderBase::readAtIndex(int32_t index) {
  open();
  auto bag_message = reader_->read_at_index(index);
  return convertBagMessage(bag_message);
}

std::shared_ptr<VTRMessage> DataStreamReaderBase::readAtTimestamp(
    rcutils_time_point_value_t time) {
  open();
  auto bag_message = reader_->read_at_timestamp(time);
  return convertBagMessage(bag_message);
}

std::shared_ptr<std::vector<std::shared_ptr<VTRMessage>>>
DataStreamReaderBase::readAtIndexRange(int32_t index_begin, int32_t index_end) {
  open();
  auto bag_message_vector =
      reader_->read_at_index_range(index_begin, index_end);
  auto deserialized_bag_message_vector =
      std::make_shared<std::vector<std::shared_ptr<VTRMessage>>>();
  for (auto bag_message : *bag_message_vector) {
    auto anytype_msg = convertBagMessage(bag_message);
    deserialized_bag_message_vector->push_back(anytype_msg);
  }
  return deserialized_bag_message_vector;
}

std::shared_ptr<std::vector<std::shared_ptr<VTRMessage>>>
DataStreamReaderBase::readAtTimestampRange(
    rcutils_time_point_value_t time_begin,
    rcutils_time_point_value_t time_end) {
  open();
  auto bag_message_vector =
      reader_->read_at_timestamp_range(time_begin, time_end);
  auto deserialized_bag_message_vector =
      std::make_shared<std::vector<std::shared_ptr<VTRMessage>>>();
  for (auto bag_message : *bag_message_vector) {
    auto anytype_msg = convertBagMessage(bag_message);
    deserialized_bag_message_vector->push_back(anytype_msg);
  }
  return deserialized_bag_message_vector;
}

bool DataStreamReaderBase::seekByIndex(int32_t index) {
  open();
  seeked_ = true;
  return reader_->seek_by_index(index);
}

bool DataStreamReaderBase::seekByTimestamp(rcutils_time_point_value_t time) {
  open();
  seeked_ = true;
  return reader_->seek_by_timestamp(time);
}

std::shared_ptr<VTRMessage> DataStreamReaderBase::readNextFromSeek() {
  if (!seeked_) seekByIndex(1);  // read the whole database

  auto bag_message = reader_->read_next_from_seek();
  return convertBagMessage(bag_message);
}

}  // namespace storage
}  // namespace vtr