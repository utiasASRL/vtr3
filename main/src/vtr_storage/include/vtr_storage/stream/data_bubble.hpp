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
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <map>

#include "rcutils/types.h"

#include <vtr_logging/logging.hpp>
#include <vtr_storage/stream/data_stream_accessor.hpp>

namespace vtr {
namespace storage {

/**
 * \brief Cache container class for a specified range of messages in a stream of
 * data. This class assumes unique time stamp
 */
class DataBubble {
 public:
  using AccessorPtr = std::shared_ptr<DataStreamAccessorBase>;
  using AccessorWeakPtr = std::weak_ptr<DataStreamAccessorBase>;

  using MessagePtr = LockableMessage::Ptr;

  using MutexType = std::recursive_mutex;
  using LockGuard = std::lock_guard<MutexType>;

  using TimestampRange = std::pair<Timestamp, Timestamp>;
  using Time2MessageMap = std::map<Timestamp, MessagePtr>;

  DataBubble(const AccessorPtr& accessor = nullptr) : accessor_(accessor) {}
  virtual ~DataBubble();

  DataBubble(const DataBubble&) = delete;
  DataBubble(DataBubble&&) = delete;
  DataBubble& operator=(const DataBubble&) = delete;
  DataBubble& operator=(DataBubble&&) = delete;

  bool hasAccessor() const;
  /** \brief Initializes a data bubble with a data stream accessor. */
  void setAccessor(const AccessorPtr& accessor);
  void resetAccessor();

  /** \brief loads messages based on the specified time range.*/
  bool load(const TimestampRange& time_range);
  /** \brief loads a message based on time. */
  bool load(const Timestamp& time);

  bool loaded(const Timestamp& time);

  /**
   * \brief Unloads all data associated with the vertex.
   * \return false if clear is set true but map is not cleared due to other
   * potential user of the message; otherwise true
   */
  bool unload(bool clear = true);

  /** \brief Inserts a message into the bubble. */
  bool insert(const MessagePtr& vtr_message);

  /** \brief Retrieves a reference to the message. */
  MessagePtr retrieve(const Timestamp& time);

  /** \brief Gets the size of the bubble. */
  size_t size() const;

 private:
  /** \brief mutex to protect access to the time2message map and accessor. */
  mutable MutexType mutex_;

  /** \brief A pointer to the data stream accessor. */
  AccessorWeakPtr accessor_;

  /** \brief the container, maps timestamp to message ordered by time stamp. */
  Time2MessageMap time2message_map_;
};
}  // namespace storage
}  // namespace vtr
