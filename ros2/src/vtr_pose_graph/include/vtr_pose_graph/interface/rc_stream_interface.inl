
#pragma once

#include <vtr_pose_graph/interface/rc_stream_interface.hpp>

namespace vtr {
namespace pose_graph {
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
    const std::string &streamName, uint32_t index) {
  FieldMap::mapped_type stream_idx;
  {
    // Get the stream index.
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(streamName);
    if (stream_itr == locked_stream_names.get().end()) {
      // LOG(WARNING) << "Stream " << streamName << " not tied to this vertex!";
      return nullptr;
    }
    stream_idx = stream_itr->second;
  }

#if 0
  // Get the data bubble.
  BubbleMap::mapped_type bubble;
  {
    auto locked_data_bubble_map = dataBubbleMap_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);
    if (bubble_itr == locked_data_bubble_map.get().end()) {
      // LOG(INFO) << "Stream " << streamName << " has no data for this vertex";
      return nullptr;
    }
    bubble = bubble_itr->second;
  }

  // load all of the data
  if (bubble == nullptr) {
    LOG(INFO) << "Data bubble " << stream_idx << " has not been initialized";
    return nullptr;
  }

  // Retrieve the data
  if (bubble->isLoaded(index)) {
    return bubble->retrieve(index).extractSharedPayload<MessageType>();
  }

  try {
    // grab the mutex from the stream map
    auto guard = lockStream(stream_idx, true, false);
    auto result = bubble->retrieve(index).extractSharedPayload<MessageType>();
    return result;
  } catch (...) {
    LOG(ERROR) << "Could not retrieve data from " << streamName << "at "
               << index;
    return nullptr;
  }
#endif

  DataBubbleMap::mapped_type data_bubble;
  {
    auto locked_data_bubble_map = data_bubble_map_->locked();
    auto bubble_itr = locked_data_bubble_map.get().find(stream_idx);
    if (bubble_itr == locked_data_bubble_map.get().end()) {
      // LOG(INFO) << "Stream " << streamName << " has no data for this vertex";
      return nullptr;
    }
    data_bubble = bubble_itr->second;
  }

  // load all of the data
  if (data_bubble == nullptr) {
    LOG(INFO) << "Data bubble " << stream_idx << " has not been initialized";
    return nullptr;
  }

  // Retrieve the data
  if (data_bubble->isLoaded(int32_t(index))) {
    return std::make_shared<MessageType>(data_bubble->retrieve(int32_t(index)));
  }

  return nullptr;
}

#if 0
template <typename MessageType>
std::shared_ptr<MessageType> RCStreamInterface::retrieveKeyframeData(
    const std::string &streamName) {
  return retrieveData<MessageType>(streamName, keyFrameTime_);
}

template <typename MessageType>
std::shared_ptr<MessageType> RCStreamInterface::retrieveData(
    const std::string &streamName, robochunk::std_msgs::TimeStamp &time) {
  FieldMap::mapped_type stream_idx;
  {
    // Get the stream index.
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(streamName);
    if (stream_itr == locked_stream_names.get().end()) {
      // LOG(WARNING) << "Stream " << streamName << " not tied to this vertex!";
      return nullptr;
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
      return nullptr;
    }
    bubble = bubble_itr->second;
  }

  // load all of the data
  if (bubble == nullptr) {
    // LOG(INFO) << "Data bubble " << stream_idx << " has not been initialized";
    return nullptr;
  }

  // Retrieve the data
  if (bubble->isLoaded(time)) {
    return bubble->retrieve(time).extractSharedPayload<MessageType>();
  }
  try {
    // grab the mutex from the stream map
    auto guard = lockStream(stream_idx, true, false);
    auto result = bubble->retrieve(time).extractSharedPayload<MessageType>();
    return result;
  } catch (...) {
    // LOG(ERROR) << "Could not retrieve data from " << streamName << " at " <<
    // time.nanoseconds_since_epoch();
    return nullptr;
  }
}

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

/// template <typename MessageType>
/// bool RCStreamInterface::insert(const std::string &stream_name,
///                                const MessageType &message,
///                                const robochunk::std_msgs::TimeStamp &stamp)
///                                {
///   // grab the mutex from the stream map
///   // auto guard = lockStream(stream_idx);
///
///   // Convert to a RobochunkMessage
///   robochunk::msgs::RobochunkMessage msg;
///   msg.mutable_header()
///       ->mutable_sensor_time_stamp()
///       ->set_nanoseconds_since_epoch(stamp.nanoseconds_since_epoch());
///   msg.setPayload(message);
///
///   // insert into the vertex
///   insert(stream_name, msg);
///
///   return true;
/// }
template <typename MessageType>
bool RCStreamInterface::insert(const std::string &stream_name,
                               MessageType &message,
                               const vtr_messages::msg::TimeStamp &stamp) {
  (void)stamp;
#if 0
  // \note used to convert MessageType to RobochunkMessage through setPayload.
  message.header.sensor_time_stamp = stamp;
#endif
  // insert into the vertex
  insert(stream_name, message);

  return true;
}

/// Check original implementation in source file
template <typename MessageType>
bool RCStreamInterface::insert(const std::string &stream_name,
                               MessageType &msg) {
  FieldMap::mapped_type stream_idx;
  {
    // Get the stream index.
    auto locked_stream_names = streamNames_->locked();
    auto stream_itr = locked_stream_names.get().find(stream_name);
    if (stream_itr == locked_stream_names.get().end()) {
      LOG(WARNING) << "Stream " << stream_name << " not tied to this vertex!";
      return false;
    }
    stream_idx = stream_itr->second;
  }
#if 0
  // Get the data bubble.
  BubbleMap::mapped_type bubble;
  {
    auto locked_data_bubble_map = dataBubbleMap_->locked();
    auto bubble_itr_bool = locked_data_bubble_map.get().emplace(
        stream_idx, std::make_shared<robochunk::base::DataBubble>());
    bubble = bubble_itr_bool.first->second;

    // If insert was successful, we need to intialize the new bubble.
    if (bubble_itr_bool.second) {
      bubble->initialize(stream_map_->locked().get().at(stream_idx).first);
    }
  }
#endif
  DataBubbleMap::mapped_type data_bubble;
  {
    auto locked_data_bubble_map = data_bubble_map_->locked();
    auto bubble_itr_bool = locked_data_bubble_map.get().emplace(
        stream_idx, std::make_shared<DataBubble>());
    data_bubble = bubble_itr_bool.first->second;

    // If insert was successful, we need to intialize the new bubble.
    if (bubble_itr_bool.second) {
      data_bubble->initialize(
          data_stream_map_->locked().get().at(stream_idx).first);
    }
  }

  // grab the mutex from the stream map
  // auto guard = lockStream(stream_idx);

  // insert the data
#if 0
  bubble->insert(msg);
#endif
  data_bubble->insert(msg);

  return true;
}

}  // namespace pose_graph
}  // namespace vtr
