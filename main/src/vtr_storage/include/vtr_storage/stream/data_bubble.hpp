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
 * \file data_bubble.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <map>

#include "rcutils/types.h"

#include <vtr_logging/logging.hpp>
#include <vtr_storage/stream/data_stream_accessor.hpp>

namespace vtr {
namespace storage {

class DataBubbleBase {
 public:
  using AccessorBasePtr = std::shared_ptr<DataStreamAccessorBase>;

  using MutexType = std::recursive_mutex;
  using LockGuard = std::lock_guard<MutexType>;

  using TimestampRange = std::pair<Timestamp, Timestamp>;

  DataBubbleBase() = default;
  DataBubbleBase(const DataBubbleBase&) = delete;
  DataBubbleBase(DataBubbleBase&&) = delete;
  DataBubbleBase& operator=(const DataBubbleBase&) = delete;
  DataBubbleBase& operator=(DataBubbleBase&&) = delete;

  virtual ~DataBubbleBase() = default;

  virtual bool hasAccessor() const = 0;
  /** \brief Initializes a data bubble with a data stream accessor. */
  virtual void setAccessor(const AccessorBasePtr& accessor) = 0;
  virtual void resetAccessor() = 0;

  /** \brief loads messages based on the specified time range.*/
  virtual bool load(const Timestamp& start, const Timestamp& stop) = 0;
  /** \brief loads a message based on time. */
  virtual bool load(const Timestamp& time) = 0;

  virtual bool loaded(const Timestamp& time) = 0;

  /**
   * \brief Unloads all data associated with the vertex.
   * \return false if clear is set true but map is not cleared due to other
   * potential user of the message; otherwise true
   */
  virtual bool unload(bool clear = true) = 0;

  /** \brief Gets the size of the bubble. */
  virtual size_t size() const = 0;

 protected:
  /** \brief mutex to protect access to the time2message map and accessor. */
  mutable MutexType mutex_;
};

/**
 * \brief Cache container class for a specified range of messages in a stream of
 * data. This class assumes unique time stamp
 */
template <typename DataType>
class DataBubble : public DataBubbleBase {
 public:
  using MessagePtr = typename LockableMessage<DataType>::Ptr;
  using Time2MessageMap = std::map<Timestamp, MessagePtr>;

  using Ptr = std::shared_ptr<DataBubble<DataType>>;
  using AccessorType = DataStreamAccessor<DataType>;
  using AccessorPtr = std::shared_ptr<AccessorType>;
  using AccessorWeakPtr = std::weak_ptr<AccessorType>;

  DataBubble() = default;
  DataBubble(const AccessorBasePtr& accessor)
      : accessor_(std::dynamic_pointer_cast<AccessorType>(accessor)) {}
  virtual ~DataBubble();

  bool hasAccessor() const override;
  /** \brief Initializes a data bubble with a data stream accessor. */
  void setAccessor(const AccessorBasePtr& accessor) override;
  void resetAccessor() override;

  /** \brief loads messages based on the specified time range.*/
  bool load(const Timestamp& start, const Timestamp& stop) override;
  /** \brief loads a message based on time. */
  bool load(const Timestamp& time) override;

  bool loaded(const Timestamp& time) override;

  /**
   * \brief Unloads all data associated with the vertex.
   * \return false if clear is set true but map is not cleared due to other
   * potential user of the message; otherwise true
   */
  bool unload(bool clear = true) override;

  /** \brief Inserts a message into the bubble. */
  bool insert(const MessagePtr& vtr_message);

  /** \brief Retrieves a reference to the message. */
  MessagePtr retrieve(const Timestamp& time);

  /** \brief Retrieves a reference to the message. */
  std::vector<MessagePtr> retrieve(const Timestamp& start,
                                   const Timestamp& stop);

  /** \brief Gets the size of the bubble. */
  size_t size() const override;

 private:
  /** \brief A pointer to the data stream accessor. */
  AccessorWeakPtr accessor_;

  /** \brief the container, maps timestamp to message ordered by time stamp. */
  Time2MessageMap time2message_map_;
};

template <typename DataType>
DataBubble<DataType>::~DataBubble() {
  const LockGuard lock(mutex_);
  for (const auto& value : time2message_map_) {
    /// This is only an approximated use count
    /// \todo check: data bubble access is single threaded, and access to
    /// message is assumed single threaded - these may guarantee that the use
    /// count returned is always accurate.
    if (value.second.use_count() > 1) {
      CLOG(WARNING, "storage")
          << "Data bubble destructor - message with time stamp " << value.first
          << " has use count " << value.second.use_count()
          << ". Data may not be saved.";
    }
    if (value.second->sharedLocked().get().getSaved() == false) {
      CLOG(WARNING, "storage")
          << "Data bubble destructor - message with time stamp " << value.first
          << " has not been saved. Call unload first.";
    }
  }
}

template <typename DataType>
bool DataBubble<DataType>::hasAccessor() const {
  const LockGuard lock(mutex_);
  return !accessor_.expired();
}

template <typename DataType>
void DataBubble<DataType>::setAccessor(const AccessorBasePtr& accessor) {
  const LockGuard lock(mutex_);
  accessor_ = std::dynamic_pointer_cast<AccessorType>(accessor);
}

template <typename DataType>
void DataBubble<DataType>::resetAccessor() {
  const LockGuard lock(mutex_);
  accessor_.reset();
}

template <typename DataType>
bool DataBubble<DataType>::load(const Timestamp& start, const Timestamp& stop) {
  const LockGuard lock(mutex_);

  auto accessor = accessor_.lock();
  if (!accessor) {
    CLOG(WARNING, "storage")
        << "Accessor has expired or not set. Skip loading.";
    return false;
  }

  auto messages = accessor->readAtTimestampRange(start, stop);
  if (messages.size() == 0) return false;

  for (const auto& message : messages) {
    const auto time = message->locked().get().getTimestamp();
    if (time2message_map_.count(time)) {
      CLOG(DEBUG, "storage") << "Message with time stamp " << time
                             << " exists. Skip adding this message to cache.";
      continue;
    }
    time2message_map_.insert({time, message});
  }

  return true;
}

template <typename DataType>
bool DataBubble<DataType>::load(const Timestamp& time) {
  const LockGuard lock(mutex_);

  auto accessor = accessor_.lock();
  if (!accessor) {
    CLOG(WARNING, "storage") << "Accessor has expired or not set. Skip loading";
    return false;
  }

  const auto message = accessor->readAtTimestamp(time);

  if (!message) return false;

  if (time2message_map_.count(time)) {
    CLOG(WARNING, "storage")
        << "Message with time stamp " << time
        << " exists. Skip loading this message into cache.";
  } else {
    time2message_map_.insert({time, message});
  }

  return true;
}

template <typename DataType>
bool DataBubble<DataType>::loaded(const Timestamp& time) {
  const LockGuard lock(mutex_);
  return time2message_map_.count(time);
}

template <typename DataType>
bool DataBubble<DataType>::unload(bool clear) {
  const LockGuard lock(mutex_);

  if (time2message_map_.empty()) return true;

  auto accessor = accessor_.lock();
  if (!accessor) {
    CLOG(WARNING, "storage") << "Accessor has expired. Skip unloading.";
    return false;
  }

  // scope to ensure deletion of messages after writing
  {
    std::vector<MessagePtr> messages;
    messages.reserve(time2message_map_.size());
    std::for_each(time2message_map_.begin(), time2message_map_.end(),
                  [&](std::pair<const Timestamp, MessagePtr>& element) {
                    messages.push_back(element.second);
                  });

    accessor->write(messages);
  }

  if (!clear) return true;

  for (const auto& value : time2message_map_) {
    /// This is only an approximated use count
    /// \todo check: data bubble access is single threaded, and access to
    /// message is assumed single threaded - these may guarantee that the use
    /// count returned is always accurate.
    if (value.second.use_count() > 1) {
      CLOG(WARNING, "storage")
          << "Message with time stamp " << value.first << " has use count "
          << value.second.use_count() << ". Keep all messages in cache.";
      return false;
    }
  }

  time2message_map_.clear();
  return true;
}

template <typename DataType>
bool DataBubble<DataType>::insert(const MessagePtr& message) {
  const LockGuard lock(mutex_);
  const auto time = message->locked().get().getTimestamp();
  if (time2message_map_.count(time)) {
    CLOG(WARNING, "storage")
        << "Message with time stamp " << time
        << " exists. Skip inserting this message into cache.";
    return false;
  }
  time2message_map_.insert({time, message});
  return true;
}

template <typename DataType>
auto DataBubble<DataType>::retrieve(const Timestamp& time) -> MessagePtr {
  const LockGuard lock(mutex_);
  if (!loaded(time) && !load(time)) {
    CLOG(WARNING, "storage")
        << "Message with time stamp " << time
        << " does not exist in cache or disk. Return a nullptr.";
    return nullptr;
  }
  return time2message_map_.at(time);
}

template <typename DataType>
auto DataBubble<DataType>::retrieve(const Timestamp& start,
                                    const Timestamp& stop)
    -> std::vector<MessagePtr> {
  const LockGuard lock(mutex_);
  std::vector<MessagePtr> messages;
  load(start, stop);
  for (auto it = time2message_map_.lower_bound(start);
       it != time2message_map_.upper_bound(stop); it++)
    messages.push_back(it->second);

  return messages;
}

template <typename DataType>
size_t DataBubble<DataType>::size() const {
  const LockGuard lock(mutex_);
  return time2message_map_.size();
}

}  // namespace storage
}  // namespace vtr
