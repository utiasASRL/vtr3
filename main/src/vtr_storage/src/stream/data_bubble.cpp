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
 * \file data_bubble.cpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_storage/stream/data_bubble.hpp>

namespace vtr {
namespace storage {

// template <typename DataType>
// DataBubble<DataType>::~DataBubble() {
//   const LockGuard lock(mutex_);
//   for (const auto& value : time2message_map_) {
//     /// This is only an approximated use count
//     /// \todo check: data bubble access is single threaded, and access to
//     /// message is assumed single threaded - these may guarantee that the use
//     /// count returned is always accurate.
//     if (value.second.use_count() > 1) {
//       CLOG(WARNING, "storage")
//           << "Data bubble destructor - message with time stamp " << value.first
//           << " has use count " << value.second.use_count()
//           << ". Data may not be saved.";
//     }
//     if (value.second->sharedLocked().get().getSaved() == false) {
//       CLOG(WARNING, "storage")
//           << "Data bubble destructor - message with time stamp " << value.first
//           << " has not been saved. Call unload first.";
//     }
//   }
// }

// template <typename DataType>
// bool DataBubble<DataType>::hasAccessor() const {
//   const LockGuard lock(mutex_);
//   return !accessor_.expired();
// }

// template <typename DataType>
// void DataBubble<DataType>::setAccessor(const AccessorBasePtr& accessor) {
//   const LockGuard lock(mutex_);
//   accessor_ = std::dynamic_pointer_cast<AccessorPtr>(accessor);
// }

// template <typename DataType>
// void DataBubble<DataType>::resetAccessor() {
//   const LockGuard lock(mutex_);
//   accessor_.reset();
// }

// template <typename DataType>
// bool DataBubble<DataType>::load(const TimestampRange& time_range) {
//   const LockGuard lock(mutex_);

//   auto accessor = accessor_.lock();
//   if (!accessor) {
//     CLOG(WARNING, "storage")
//         << "Accessor has expired or not set. Skip loading.";
//     return false;
//   }

//   auto messages =
//       accessor->readAtTimestampRange(time_range.first, time_range.second);
//   if (messages.size() == 0) return false;

//   for (const auto& message : messages) {
//     const auto time = message->locked().get().getTimestamp();
//     if (time2message_map_.count(time)) {
//       CLOG(WARNING, "storage") << "Message with time stamp " << time
//                                << " exists. Skip adding this message to cache.";
//       continue;
//     }
//     time2message_map_.insert({time, message});
//   }

//   return true;
// }

// template <typename DataType>
// bool DataBubble<DataType>::load(const Timestamp& time) {
//   const LockGuard lock(mutex_);

//   auto accessor = accessor_.lock();
//   if (!accessor) {
//     CLOG(WARNING, "storage") << "Accessor has expired or not set. Skip loading";
//     return false;
//   }

//   const auto message = accessor->readAtTimestamp(time);

//   if (!message) return false;

//   if (time2message_map_.count(time)) {
//     CLOG(WARNING, "storage")
//         << "Message with time stamp " << time
//         << " exists. Skip loading this message into cache.";
//   } else {
//     time2message_map_.insert({time, message});
//   }

//   return true;
// }

// template <typename DataType>
// bool DataBubble<DataType>::loaded(const Timestamp& time) {
//   const LockGuard lock(mutex_);
//   return time2message_map_.count(time);
// }

// template <typename DataType>
// bool DataBubble<DataType>::unload(bool clear) {
//   const LockGuard lock(mutex_);

//   if (time2message_map_.empty()) return true;

//   auto accessor = accessor_.lock();
//   if (!accessor) {
//     CLOG(WARNING, "storage") << "Accessor has expired. Skip unloading.";
//     return false;
//   }

//   // scope to ensure deletion of messages after writing
//   {
//     std::vector<MessagePtr> messages;
//     messages.reserve(time2message_map_.size());
//     std::for_each(time2message_map_.begin(), time2message_map_.end(),
//                   [&](Time2MessageMap::value_type& element) {
//                     messages.push_back(element.second);
//                   });

//     accessor->write(messages);
//   }

//   if (!clear) return true;

//   for (const auto& value : time2message_map_) {
//     /// This is only an approximated use count
//     /// \todo check: data bubble access is single threaded, and access to
//     /// message is assumed single threaded - these may guarantee that the use
//     /// count returned is always accurate.
//     if (value.second.use_count() > 1) {
//       CLOG(WARNING, "storage")
//           << "Message with time stamp " << value.first << " has use count "
//           << value.second.use_count() << ". Keep all messages in cache.";
//       return false;
//     }
//   }

//   time2message_map_.clear();
//   return true;
// }

// template <typename DataType>
// bool DataBubble<DataType>::insert(const MessagePtr& message) {
//   const LockGuard lock(mutex_);
//   const auto time = message->locked().get().getTimestamp();
//   if (time2message_map_.count(time)) {
//     CLOG(WARNING, "storage")
//         << "Message with time stamp " << time
//         << " exists. Skip inserting this message into cache.";
//     return false;
//   }
//   time2message_map_.insert({time, message});
//   return true;
// }

// template <typename DataType>
// auto DataBubble<DataType>::retrieve(const Timestamp& time) -> MessagePtr {
//   const LockGuard lock(mutex_);
//   if (!loaded(time) && !load(time)) {
//     CLOG(WARNING, "storage")
//         << "Message with time stamp " << time
//         << " does not exist in cache or disk. Return a nullptr.";
//     return nullptr;
//   }
//   return time2message_map_.at(time);
// }

// template <typename DataType>
// size_t DataBubble<DataType>::size() const {
//   const LockGuard lock(mutex_);
//   return time2message_map_.size();
// }

}  // namespace storage
}  // namespace vtr