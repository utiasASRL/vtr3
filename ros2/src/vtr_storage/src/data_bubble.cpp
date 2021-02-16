#include <vtr_storage/data_bubble.hpp>

namespace vtr {
namespace storage {

void DataBubble::load(bool to_data_vec) {
  if (global_index_set_ && timestamp_set_)
    throw std::runtime_error{"Cannot have both indices and time set to load."};
  // seek to the start idx
  if (global_index_set_)
    loadIndexRange_(indices_.start_index, indices_.stop_index, to_data_vec);
  else if (timestamp_set_)
    loadTimeRange_(indices_.start_time, indices_.stop_time, to_data_vec);
}

void DataBubble::load(Index local_idx, bool to_data_vec) {
  if (!global_index_set_) throw std::runtime_error{"Global index not set."};

  auto global_idx = indices_.start_index + local_idx;
  auto vtr_message = data_stream_->readAtIndex(global_idx);
  if (!vtr_message) return;

  index_map_.insert({local_idx, vtr_message});

  if (vtr_message->has_timestamp())
    time_map_.insert({vtr_message->get_timestamp(), vtr_message});

  if (to_data_vec) data_vec_.push_back(vtr_message);
}

void DataBubble::loadTime(TimeStamp time, bool to_data_vec) {
  auto vtr_message = data_stream_->readAtTimestamp(time);
  if (!vtr_message) return;

  time_map_.insert({time, vtr_message});

  if (vtr_message->has_index())
    index_map_.insert({vtr_message->get_index(), vtr_message});

  if (to_data_vec) data_vec_.push_back(vtr_message);
}

void DataBubble::unload() {
  indices_ = ChunkIndices{};
  global_index_set_ = false;
  timestamp_set_ = false;
  index_map_.clear();
  time_map_.clear();
  data_vec_.clear();
}

void DataBubble::insert(const VTRMessage& vtr_message) {
  auto vtr_message_ptr = std::make_shared<VTRMessage>(vtr_message);
  data_vec_.push_back(vtr_message_ptr);

  if (vtr_message_ptr->has_index())
    index_map_.insert({vtr_message_ptr->get_index(), vtr_message_ptr});

  if (vtr_message_ptr->has_timestamp())
    time_map_.insert({vtr_message_ptr->get_timestamp(), vtr_message_ptr});
}

VTRMessage DataBubble::retrieve(Index local_idx) {
  if (!global_index_set_) throw std::runtime_error{"Global index not set."};

  if (!isLoaded(local_idx)) load(local_idx);
  if (!isLoaded(local_idx))
    throw std::out_of_range("DataBubble has no data at this index: " +
                            std::to_string(local_idx));

  auto global_idx = indices_.start_index + local_idx;
  return *index_map_.at(global_idx);
}

VTRMessage DataBubble::retrieveTime(TimeStamp time) {
  if (!isLoaded(time)) loadTime(time);
  if (!isLoaded(time))
    throw std::out_of_range("DataBubble has no data at this time: " +
                            std::to_string(time));
  return *time_map_.at(time);
}

bool DataBubble::setIndices(Index index_begin, Index index_end) {
  if (!index_end) index_end = index_begin;
  if (index_end < index_begin)
    throw std::invalid_argument{"index_end cannot be less than index_begin: " +
                                std::to_string(index_begin) + ", " +
                                std::to_string(index_end)};
  indices_.start_index = index_begin;
  indices_.stop_index = index_end;
  global_index_set_ = true;
  return true;
}

bool DataBubble::setTimeIndices(TimeStamp time_begin, TimeStamp time_end) {
  indices_.start_time = time_begin;
  indices_.stop_time = time_end;
  timestamp_set_ = true;
  return true;
}

void DataBubble::loadIndexRange_(Index global_idx0, Index global_idx1,
                                 bool to_data_vec) {
  auto vtr_message_vector =
      data_stream_->readAtIndexRange(global_idx0, global_idx1);
  if (vtr_message_vector->size() == 0) return;
  for (auto vtr_message : *vtr_message_vector) {
    if (!vtr_message->has_index())
      throw std::runtime_error{"Loaded message does not have index set."};
    index_map_.insert({vtr_message->get_index(), vtr_message});

    if (vtr_message->has_timestamp())
      time_map_.insert({vtr_message->get_timestamp(), vtr_message});

    if (to_data_vec) data_vec_.push_back(vtr_message);
  }
}

void DataBubble::loadTimeRange_(TimeStamp time0, TimeStamp time1,
                                bool to_data_vec) {
  auto vtr_message_vector = data_stream_->readAtTimestampRange(time0, time1);
  if (vtr_message_vector->size() == 0) return;
  for (auto vtr_message : *vtr_message_vector) {
    time_map_.insert({vtr_message->get_timestamp(), vtr_message});

    if (!vtr_message->has_index())
      throw std::runtime_error{"Loaded message does not have index set."};
    index_map_.insert({vtr_message->get_index(), vtr_message});

    if (to_data_vec) data_vec_.push_back(vtr_message);
  }
}

}  // namespace storage
}  // namespace vtr