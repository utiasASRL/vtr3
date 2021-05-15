
#pragma once

#include <vtr_pose_graph/interface/rc_stream_interface.hpp>

namespace vtr {
namespace pose_graph {

template <typename MessageType>
void RCStreamInterface::addStreamIndices(const std::string &stream_name,
                                         const Interval &interval,
                                         bool overwrite) {
  if (streamNames_ == nullptr) streamNames_.reset(new LockableFieldMap());

  // add the stream names to the map if id does not exsist.
  FieldMap::mapped_type idx;
  {
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr_bool = locked_stream_names.get().emplace(
        stream_name, locked_stream_names.get().size());
    idx = stream_itr_bool.first->second;
  }

  // insert the index into the map.
  if (overwrite)
    streamIndices_.locked().get()[idx] = interval;
  else
    streamIndices_.locked().get().emplace(idx, interval);

  if (data_stream_map_ == nullptr) {
    LOG(ERROR) << "Streams have not been initialized for this run!!";
    return;
  }

  auto data_bubble = data_bubble_map_->locked()
                         .get()
                         .emplace(idx, std::make_shared<DataBubble>())
                         .first->second;
  auto guard = lockStream(idx);
  data_bubble->initialize(data_stream_map_->locked().get().at(idx).first);
  data_bubble->unload();
  data_bubble->setIndices(interval.first, interval.second);
}

#if 0
template <typename MessageType>
std::vector<std::shared_ptr<MessageType>> RCStreamInterface::retrieveData(
    const std::string &streamName) {
  std::vector<std::shared_ptr<MessageType>> message_vector;

  // Get the stream index.
  FieldMap::mapped_type stream_idx;
  {
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(streamName);
    if (stream_itr == locked_stream_names.get().end()) {
      // LOG(WARNING) << "Stream " << streamName << " not tied to this vertex!";
      return message_vector;
    }
    stream_idx = stream_itr->second;
  }

  // Get the data bubble.
  BubbleMap::mapped_type bubble;
  {
    auto locked_data_bubble_map = dataBubbleMap_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);
    if (bubble_itr == locked_data_bubble_map.get().end()) {
      // LOG(INFO) << "Stream " << streamName << " has no data for this vertex";
      return message_vector;
    }
    bubble = bubble_itr->second;
  }

  // load all of the data
  if (bubble == nullptr) {
    LOG(FATAL) << "Data bubble " << stream_idx << " has not been initialized";
    throw std::runtime_error("Data bubble  has not been initialized");
  }

  // grab the mutex from the stream map
  auto guard = lockStream(stream_idx, true, false);

  // Load the data bubble.
  bubble->load();

  // extract the shared pointers
  for (uint32_t idx = 0; idx < bubble->size(); ++idx) {
    // extract the base message
    auto &message = bubble->retrieve(idx);
    message_vector.push_back(message.extractSharedPayload<MessageType>());
  }
  return message_vector;
}
#endif

template <typename MessageType>
std::shared_ptr<MessageType> RCStreamInterface::retrieveData(
    const std::string &streamName, uint32_t index, bool allow_nullptr) {
  // get stream idx and data bubble
  FieldMap::mapped_type stream_idx;
  auto data_bubble = getDataBubbleAndStreamIndex(streamName, stream_idx);

  // load all of the data
  if (data_bubble == nullptr) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Data bubble associated with " << streamName
          << " has not been initialized";
      LOG(ERROR) << msg.str();
      throw std::runtime_error{msg.str()};
    }
    return nullptr;
  }

  // Retrieve the data
  if (data_bubble->isLoaded(int32_t(index))) {
    auto vtr_message = data_bubble->retrieve(int32_t(index));
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  }

  try {
    // grab the mutex from the stream map
    auto guard = lockStream(stream_idx, true, false);
    auto vtr_message = data_bubble->retrieve(int32_t(index));
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  } catch (...) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Could not retrieve data from " << streamName << "at " << index;
      LOG(ERROR) << msg.str();
      throw std::runtime_error{msg.str()};
    }
  }

  return nullptr;
}

template <typename MessageType>
std::shared_ptr<MessageType> RCStreamInterface::retrieveData(
    const std::string &streamName, vtr_messages::msg::TimeStamp &time,
    bool allow_nullptr) {
  // get stream idx and data bubble
  FieldMap::mapped_type stream_idx;
  auto data_bubble = getDataBubbleAndStreamIndex(streamName, stream_idx);

  // load all of the data
  if (data_bubble == nullptr) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Data bubble associated with " << streamName
          << " has not been initialized";
      LOG(ERROR) << msg.str();
      throw std::runtime_error{msg.str()};
    }
    return nullptr;
  }

  // Retrieve the data
  if (data_bubble->isLoaded(time)) {
    auto vtr_message = data_bubble->retrieve(time);
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  }
  try {
    // grab the mutex from the stream map
    if (!data_bubble->isInitialized()) {
      LOG(ERROR) << "Data bubble has not been initialized!";
    }

    auto guard = lockStream(stream_idx, true, false);
    auto vtr_message = data_bubble->retrieve(time);
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  } catch (...) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Could not retrieve data from " << streamName << " at "
          << time.nanoseconds_since_epoch;
      LOG(ERROR) << msg.str();
      throw std::runtime_error{msg.str()};
    }
  }
  return nullptr;
}

#if 0
template <typename MessageType>
bool RCStreamInterface::insertAndWrite(
    const std::string &stream_name, const MessageType &message,
    const robochunk::std_msgs::TimeStamp &stamp) {
  // Create the data
  robochunk::msgs::RobochunkMessage msg;
  auto *msg_stamp = msg.mutable_header()->mutable_sensor_time_stamp();
  *msg_stamp = stamp;
  msg.setPayload(message);

  // Get the serializer;
  auto stream_idx = streamNames_->locked().get().at(stream_name);
  auto serializer = stream_map_->locked().get().at(stream_idx).second;
  if (serializer == nullptr) {
    LOG(ERROR) << "The serializer for stream " << stream_name
               << " is not initialized, (is this a read-only stream?)";
    return false;
  }
  auto return_structure = serializer->serialize(msg.baseMessage());

  // add the index
  auto locked_stream_indices = streamIndices_.locked();
  auto stream_itr_bool = locked_stream_indices.get().emplace(
      stream_idx,
      std::make_pair(return_structure.index, return_structure.index));
  if (!stream_itr_bool.second) {
    stream_itr_bool.first->second.second = return_structure.index;
  }

  // add the time
  if (timeRange_.first == 0) {
    timeRange_.first = stamp.nanoseconds_since_epoch();
    timeRange_.second = stamp.nanoseconds_since_epoch();
  } else {
    timeRange_.second = stamp.nanoseconds_since_epoch();
  }
  insert(stream_name, message, stamp);
  return true;
}
#endif

}  // namespace pose_graph
}  // namespace vtr
