#include "vtr_storage/data_stream_reader.hpp"

namespace vtr {
namespace storage {

DataStreamReaderBase::~DataStreamReaderBase() { close(); }

void DataStreamReaderBase::close() {
  reader_.reset();
  this->opened_ = false;
  seeked_ = false;
}

void DataStreamReaderBase::openAndGetMessageType() {
  if (this->opened_ == false) {
    if (!data_directory_.exists()) {
      throw NoBagExistsException(data_directory_);
    }
    reader_ = std::make_shared<RandomAccessReader>(this->stream_name_);
    reader_->open(this->storage_options_, this->converter_options_);
    this->opened_ = true;
  }
  // ToDo: get message type, specialize this->serialization_ based on message
  // type?
}

std::shared_ptr<VTRMessage> DataStreamReaderBase::readAtIndex(int32_t index) {
  openAndGetMessageType();
  auto bag_message = reader_->read_at_index(index);
  return convertBagMessage(bag_message);
}

std::shared_ptr<VTRMessage> DataStreamReaderBase::readAtTimestamp(
    rcutils_time_point_value_t time) {
  openAndGetMessageType();
  auto bag_message = reader_->read_at_timestamp(time);
  return convertBagMessage(bag_message);
}

std::shared_ptr<std::vector<std::shared_ptr<VTRMessage>>>
DataStreamReaderBase::readAtIndexRange(int32_t index_begin, int32_t index_end) {
  openAndGetMessageType();
  auto bag_message_vector =
      reader_->read_at_index_range(index_begin, index_end);
  auto deserialized_bag_message_vector =
      std::make_shared<std::vector<std::shared_ptr<VTRMessage>>>();
  for (auto bag_message : *bag_message_vector) {
    auto anytype_msg = convertBagMessage(bag_message);
    deserialized_bag_message_vector->push_back(
        anytype_msg);  // ToDo: reserve the vector instead of pushing
                       // back
  }
  return deserialized_bag_message_vector;
}

std::shared_ptr<std::vector<std::shared_ptr<VTRMessage>>>
DataStreamReaderBase::readAtTimestampRange(
    rcutils_time_point_value_t time_begin,
    rcutils_time_point_value_t time_end) {
  openAndGetMessageType();
  auto bag_message_vector =
      reader_->read_at_timestamp_range(time_begin, time_end);
  auto deserialized_bag_message_vector =
      std::make_shared<std::vector<std::shared_ptr<VTRMessage>>>();
  for (auto bag_message : *bag_message_vector) {
    auto anytype_msg = convertBagMessage(bag_message);
    deserialized_bag_message_vector->push_back(
        anytype_msg);  // ToDo: reserve the vector instead of pushing
                       // back
  }
  return deserialized_bag_message_vector;
}

bool DataStreamReaderBase::seekByIndex(int32_t index) {
  openAndGetMessageType();
  seeked_ = true;
  return reader_->seek_by_index(index);
}

bool DataStreamReaderBase::seekByTimestamp(rcutils_time_point_value_t time) {
  openAndGetMessageType();
  seeked_ = true;
  return reader_->seek_by_timestamp(time);
}

std::shared_ptr<VTRMessage> DataStreamReaderBase::readNextFromSeek() {
  if (!seeked_) {
    seekByIndex(1);  // read the whole database
  }
  auto bag_message = reader_->read_next_from_seek();
  return convertBagMessage(bag_message);
}

}  // namespace storage
}  // namespace vtr