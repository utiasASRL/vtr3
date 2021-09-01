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
 * \file rc_stream_interface.inl
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_pose_graph/interface/rc_stream_interface.hpp>

namespace vtr {
namespace pose_graph {

template <typename MessageType>
void RCStreamInterface::addStreamIndices(const std::string &stream_name,
                                         const Interval &interval,
                                         bool overwrite) {
  if (data_stream_map_ == nullptr) {
    std::string error{"Streams have not been initialized for this run!!"};
    LOG(ERROR) << error;
    throw std::runtime_error{error};
  }

  if (stream_names_ == nullptr) stream_names_.reset(new LockableFieldMap());

  // add the stream names to the map if id does not exsist.
  FieldMap::mapped_type idx;
  {
    auto locked_stream_names = stream_names_->locked();
    auto stream_itr_bool = locked_stream_names.get().emplace(
        stream_name, locked_stream_names.get().size());
    idx = stream_itr_bool.first->second;
  }

  auto locked_data_bubble_map = data_bubble_map_->locked();

  // insert the index into the map.
  if (overwrite)
    stream_indices_.locked().get()[idx] = interval;
  else
    stream_indices_.locked().get().emplace(idx, interval);

  auto data_bubble = locked_data_bubble_map.get()
                         .emplace(idx, std::make_shared<DataBubble>())
                         .first->second;
  auto guard = lockReadWriteStream(idx);
  data_bubble->initialize(data_stream_map_->locked().get().at(idx).first);
  data_bubble->unload();
  data_bubble->setIndices(interval.first, interval.second);
}

#if 0
template <typename MessageType>
std::vector<std::shared_ptr<MessageType>> RCStreamInterface::retrieveData(
    const std::string &stream_name) {
  std::vector<std::shared_ptr<MessageType>> message_vector;

  // Get the stream index.
  FieldMap::mapped_type stream_idx;
  {
    auto locked_stream_names = stream_names_->locked();
    auto stream_itr = locked_stream_names.get().find(stream_name);
    if (stream_itr == locked_stream_names.get().end()) {
      // LOG(WARNING) << "Stream " << stream_name << " not tied to this vertex!";
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
      // LOG(INFO) << "Stream " << stream_name << " has no data for this vertex";
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
  auto guard = lockReadWriteStream(stream_idx, true, false);

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
    const std::string &stream_name, uint32_t index, bool allow_nullptr) {
  // get stream idx and data bubble
  FieldMap::mapped_type stream_idx;
  auto data_bubble = getDataBubbleAndStreamIndex(stream_name, stream_idx);

  // load all of the data
  if (data_bubble == nullptr) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Data bubble associated with " << stream_name
          << " has not been initialized";
      LOG(ERROR) << msg.str();
      throw std::runtime_error{msg.str()};
    }
    return nullptr;
  }

  // \todo use the data bubble map lock to protect stream indices map for now
  auto locked_data_bubble_map = data_bubble_map_->locked();

  // Retrieve the data
  if (data_bubble->isLoaded(int32_t(index))) {
    auto vtr_message = data_bubble->retrieve(int32_t(index));
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  }

  try {
    // grab the mutex from the stream map
    auto guard = lockReadWriteStream(stream_idx, true, false);
    auto vtr_message = data_bubble->retrieve(int32_t(index));
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  } catch (...) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Could not retrieve data from " << stream_name << "at " << index;
      LOG(ERROR) << msg.str();
      throw std::runtime_error{msg.str()};
    }
  }

  return nullptr;
}

template <typename MessageType>
std::shared_ptr<MessageType> RCStreamInterface::retrieveData(
    const std::string &stream_name, vtr_messages::msg::TimeStamp &time,
    bool allow_nullptr) {
  // get stream idx and data bubble
  FieldMap::mapped_type stream_idx;
  auto data_bubble = getDataBubbleAndStreamIndex(stream_name, stream_idx);

  // load all of the data
  if (data_bubble == nullptr) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Data bubble associated with " << stream_name
          << " has not been initialized";
      LOG(ERROR) << msg.str();
      LOG(ERROR) << el::base::debug::StackTrace();
      throw std::runtime_error{msg.str()};
    }
    return nullptr;
  }

  // \todo use the data bubble map lock to protect stream indices map for now
  auto locked_data_bubble_map = data_bubble_map_->locked();

  // Retrieve the data
  if (data_bubble->isLoaded(time)) {
    auto vtr_message = data_bubble->retrieve(time);
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  }
  try {
    // grab the mutex from the stream map
    auto guard = lockReadWriteStream(stream_idx, true, false);
    auto vtr_message = data_bubble->retrieve(time);
    return std::make_shared<MessageType>(
        vtr_message.template get<MessageType>());
  } catch (...) {
    if (!allow_nullptr) {
      std::stringstream msg;
      msg << "Could not retrieve data from " << stream_name << " at "
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
  auto stream_idx = stream_names_->locked().get().at(stream_name);
  auto serializer = stream_map_->locked().get().at(stream_idx).second;
  if (serializer == nullptr) {
    LOG(ERROR) << "The serializer for stream " << stream_name
               << " is not initialized, (is this a read-only stream?)";
    return false;
  }
  auto return_structure = serializer->serialize(msg.baseMessage());

  // add the index
  auto locked_stream_indices = stream_indices_.locked();
  auto stream_itr_bool = locked_stream_indices.get().emplace(
      stream_idx,
      std::make_pair(return_structure.index, return_structure.index));
  if (!stream_itr_bool.second) {
    stream_itr_bool.first->second.second = return_structure.index;
  }

  // add the time
  if (time_range_.first == 0) {
    time_range_.first = stamp.nanoseconds_since_epoch();
    time_range_.second = stamp.nanoseconds_since_epoch();
  } else {
    time_range_.second = stamp.nanoseconds_since_epoch();
  }
  insert(stream_name, message, stamp);
  return true;
}
#endif

}  // namespace pose_graph
}  // namespace vtr
