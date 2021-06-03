#include <vtr_storage/data_bubble.hpp>

namespace vtr {
namespace storage {

DataBubble::DataBubble()
    : endIdx_(0),
      loadFromIndex_(false),
      loadFromTime_(false),
      memoryUsageBytes_(0),
      is_loaded_(false) {}

DataBubble::~DataBubble() { reset(); }

void DataBubble::reset() {
  endIdx_ = 0;
  loadFromIndex_ = false;
  loadFromTime_ = false;
  memoryUsageBytes_ = 0;
  is_loaded_ = false;
  indices_ = ChunkIndices();
  data_map_.clear();
  time_map_.clear();
  data_stream_.reset();
}

void DataBubble::initialize(std::shared_ptr<DataStreamReaderBase> data_stream) {
  data_stream_ = data_stream;
}

int32_t DataBubble::size() { return data_map_.size(); }

bool DataBubble::isLoaded(int32_t idx) { return data_map_.count(idx); }

bool DataBubble::isLoaded(TimeStamp time) {
  auto time_it = time_map_.find(time);  // get the index from the time map
  return time_it != time_map_.end() && isLoaded(time_it->second);
}

bool DataBubble::isLoaded(VTRTimeStamp time) {
  return isLoaded(toTimeStamp(time));
}

void DataBubble::load() {
  if (is_loaded_ == true) {
    return;
  }
  // seek to the start idx
  if (loadFromIndex_) {
    load(indices_.start_index, indices_.stop_index);
    is_loaded_ = true;
  }
  if (loadFromTime_) {
    loadTime(indices_.start_time, indices_.stop_time);
    is_loaded_ = true;
  }
  if (!loadFromIndex_ && !loadFromTime_) {
    is_loaded_ = false;
    // throw std::runtime_error("DataBubble::load() called but no indices were
    // set in bubble."); LOG(ERROR) << __func__ << "ERROR No indices provided,
    // call setIndices, or setTimeIndices";  // ToDo: this still doesn't build
  }
}

void DataBubble::load(int32_t local_idx) {
  auto anytype_message =
      data_stream_->readAtIndex(indices_.start_index + local_idx);
  if (anytype_message) {
    data_map_.insert({local_idx, *anytype_message});
    // ToDo: why is this in the original code?
    // if(data_stream_->next(data_map_[idx]) == false) {
    //     data_map_.erase(idx);
    //     return;
    // }
    if (anytype_message->has_timestamp()) {
      time_map_.insert({anytype_message->get_timestamp(), local_idx});
    }
    // memoryUsageBytes_+= data_map_[local_idx].ByteSize(); // ToDo get bytesize
    // from ros messages somehow?
  }
}

void DataBubble::load(int32_t global_idx0, int32_t global_idx1) {
  auto anytype_message_vector =
      data_stream_->readAtIndexRange(global_idx0, global_idx1);
  if (anytype_message_vector->size() > 0) {
    for (int32_t local_idx = 0; local_idx <= global_idx1 - global_idx0;
         ++local_idx) {
      auto anytype_message = anytype_message_vector->at(local_idx);
      data_map_.insert({local_idx, *anytype_message});
      // if(data_stream_->next(data_map_[idx]) == false) {
      //     data_map_.erase(idx);
      //     return;
      // }
      if (anytype_message->has_timestamp()) {
        time_map_.insert({anytype_message->get_timestamp(), local_idx});
      }
      // memoryUsageBytes_+= data_map_[local_idx].ByteSize();
    }
  }
}

void DataBubble::loadTime(TimeStamp time) {
  auto anytype_message = data_stream_->readAtTimestamp(time);
  if (anytype_message) {
    auto local_idx = anytype_message->get_index() - indices_.start_index;
    data_map_.insert({local_idx, *anytype_message});
    time_map_.insert({time, local_idx});
  }

  /// Seek in the stream to this time.
  // if(data_stream_->readAtTimestamp(time)) {
  //   Message message;
  //   if(true) // Todo: make alternative for: data_stream_->next(message) ==
  //   true) {
  //     // determine the appropriate index.
  //     auto seq_id = message.header().sequence_id();
  //     if(loadFromIndex_ == true && (seq_id < indices_.start_index || seq_id >
  //     indices_.stop_index)) {
  //       LOG(DEBUG) << "The seek timestamp for the vertex is inconsistent with
  //       this bubble''s indices."; LOG(DEBUG) << "seek time: " <<
  //       time.nanoseconds_since_epoch() << " msg time: " <<
  //       message.header().sensor_time_stamp().nanoseconds_since_epoch();
  //       LOG(DEBUG) << " msg seq_id: " << seq_id << " bubble indices: (" <<
  //       indices_.start_index << "," << indices_.stop_index << ")";
  //       // uh oh, we might have grabbed the wrong message. try to seek from
  //       index. message.Clear();
  //       // if this is there is only one message in the bubble, then grab it
  //       using the index. if(indices_.start_index == indices_.stop_index) {
  //         LOG(DEBUG) << "Attempting to seek from index...";
  //         if(data_stream_->seek(indices_.start_index) &&
  //         data_stream_->next(message)) {
  //           auto seq_id = message.header().sequence_id();
  //           LOG(DEBUG) << "seek time: " << time.nanoseconds_since_epoch() <<
  //           " msg time: " <<
  //           message.header().sensor_time_stamp().nanoseconds_since_epoch();
  //           LOG(DEBUG) << " msg seq_id: " << seq_id << " bubble indices: ("
  //           << indices_.start_index << "," << indices_.stop_index << ")";
  //           auto index = seq_id - indices_.start_index;
  //           LOG(DEBUG) << "success! ";
  //           data_map_[index] = message;
  //           time_map_.insert({message.header().sensor_time_stamp().nanoseconds_since_epoch(),index});
  //           return;
  //         }
  //       }
  //       throw std::runtime_error("Timestaps are inconsistent with Indices in
  //       the stream data!");
  //     }
  //     auto index = seq_id - indices_.start_index;
  //     data_map_[index] = message;
  //     time_map_.insert({message.header().sensor_time_stamp().nanoseconds_since_epoch(),index});
  //   }
  // } else {
  //   LOG(DEBUG) << __func__ << "Boom! we failed to seek!";
  //   throw std::runtime_error("Boom! we failed to seek!");
  // }
}

void DataBubble::load(VTRTimeStamp time) { return loadTime(toTimeStamp(time)); }

void DataBubble::loadTime(TimeStamp time0, TimeStamp time1) {
  auto anytype_message_vector =
      data_stream_->readAtTimestampRange(time0, time1);
  if (anytype_message_vector->size() > 0) {
    for (auto anytype_message : *anytype_message_vector) {
      auto local_idx = anytype_message->get_index() - indices_.start_index;
      auto time = anytype_message->get_timestamp();
      data_map_.insert({local_idx, *anytype_message});
      time_map_.insert({time, local_idx});
      // if(data_stream_->next(data_map_[idx]) == false) {
      //     data_map_.erase(idx);
      //     return;
      // }
      // memoryUsageBytes_+= data_map_[local_idx].ByteSize();
    }
  }
  // bool continue_load = true;
  // if(data_stream_->seek(time0)) {
  //     while(continue_load) {
  //         endIdx_++;
  //         data_map_[endIdx_] = msgs::RobochunkMessage();
  //         if(data_stream_->next(data_map_[endIdx_]) == false ||
  //         data_map_[endIdx_].header().sensor_time_stamp().nanoseconds_since_epoch()
  //         > time1.nanoseconds_since_epoch()) {
  //             data_map_.erase(endIdx_);
  //             endIdx_--;
  //             indices_.stop_index = size()-1;
  //             return;
  //         }
  //         auto stamp =
  //         data_map_[endIdx_].baseMessage().header().sensor_time_stamp();
  //         time_map_.insert({stamp.nanoseconds_since_epoch(),endIdx_});
  //         if(indices_.start_index == 0) {
  //             indices_.start_index =
  //             data_map_[endIdx_].baseMessage().header().sequence_id();
  //         }
  //     }
  // }
  // else {
  //     LOG(ERROR) << "seek falied!";
  // }
}

void DataBubble::load(VTRTimeStamp time0, VTRTimeStamp time1) {
  return loadTime(toTimeStamp(time0), toTimeStamp(time1));
}

void DataBubble::unload() {
  is_loaded_ = false;
  memoryUsageBytes_ = 0;
  data_map_.clear();
}

void DataBubble::unload(int32_t local_idx) {
  // memoryUsageBytes_ -= data_map_[local_idx].ByteSize();
  data_map_.erase(local_idx);
}

void DataBubble::unload(int32_t local_idx0, int32_t local_idx1) {
  for (int32_t local_index = local_idx0; local_index < local_idx1;
       ++local_index) {
    unload(local_index);
  }
}

void DataBubble::insert(const VTRMessage& message) {
  data_map_.insert({endIdx_, message});
  // time_map_.insert({message.header().sensor_time_stamp().nanoseconds_since_epoch(),endIdx_});
  endIdx_++;
  // memoryUsageBytes_+=message.ByteSize();
}

VTRMessage DataBubble::retrieve(int32_t local_idx) {
  if (!isLoaded(local_idx)) load(local_idx);
  if (!isLoaded(local_idx))
    throw std::out_of_range("DataBubble has no data at this index.");

  return data_map_[local_idx];
}

VTRMessage DataBubble::retrieve(VTRTimeStamp time) {
  return retrieveTime(toTimeStamp(time));
}

VTRMessage DataBubble::retrieveTime(TimeStamp time) {
  if (!isLoaded(time)) loadTime(time);
  if (!isLoaded(time))
    throw std::out_of_range("DataBubble has no data at this time.");

  return data_map_[time_map_[time]];
  // // check to see if its in the time map
  // auto stamp_nanoseconds = time.nanoseconds_since_epoch();
  // if(time_map_.find(stamp_nanoseconds) != std::end(time_map_)) {
  //     // hooray we have a valid idx
  //     return retrieve(time_map_[stamp_nanoseconds]);
  // }

  // // The message is not in the bubble's memory, try to load it.
  // if(loadFromIndex_ == true || loadFromTime_ == true) {
  //     // Load from disk.
  //     load(time);

  //     // If we successfully loaded, then return the data
  //     if(time_map_.find(stamp_nanoseconds) != std::end(time_map_)) {
  //       return retrieve(time_map_[stamp_nanoseconds]);
  //     }
  // }
  // // otherwise throw.
  // throw std::out_of_range("DataBubble has no data at this time stamp.");
}

bool DataBubble::setIndices(uint64_t index_begin, uint64_t index_end) {
  if (index_end < index_begin) {
    // ToDo: include easylogging++
    // LOG(ERROR) << "ERROR: Invalid inex range (" << index_begin <<  ","  <<
    // index_end << ")";
    return false;
  }

  indices_.start_index = index_begin;
  indices_.stop_index = index_end;
  loadFromIndex_ = true;
  endIdx_ = (indices_.stop_index - indices_.start_index);
  if (endIdx_ == 0) {
    endIdx_ = -1;
  }
  return true;
}

bool DataBubble::setTimeIndices(TimeStamp time_begin, TimeStamp time_end) {
  // if(time_end.nanoseconds_since_epoch() <
  // time_begin.nanoseconds_since_epoch()) {
  //     LOG(ERROR) << "ERROR: Invalid index range (" <<
  //     time_begin.nanoseconds_since_epoch() << "," <<
  //     time_end.nanoseconds_since_epoch() << ")"; return false;
  // }

  indices_.start_time = time_begin;
  indices_.stop_time = time_end;
  loadFromTime_ = true;
  return true;
}

bool DataBubble::setTimeIndices(VTRTimeStamp time_begin,
                                VTRTimeStamp time_end) {
  return setTimeIndices(toTimeStamp(time_begin), toTimeStamp(time_end));
}

}  // namespace storage
}  // namespace vtr