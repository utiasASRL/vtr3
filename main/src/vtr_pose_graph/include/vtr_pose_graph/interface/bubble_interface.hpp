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
 * \file bubble_interface.hpp
 * \brief BubbleInterface class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_common/utils/lockable.hpp>
#include <vtr_storage/stream/data_bubble.hpp>

namespace vtr {
namespace pose_graph {

using Timestamp = storage::Timestamp;
using TimestampRange = storage::DataBubbleBase::TimestampRange;

class BubbleInterface {
 public:
  using Name2AccessorMap = common::SharedLockable<std::unordered_map<
      std::string, std::shared_ptr<storage::DataStreamAccessorBase>>>;
  using Name2AccessorMapPtr = std::shared_ptr<Name2AccessorMap>;
  using Name2AccessorMapWeakPtr = std::weak_ptr<Name2AccessorMap>;

  using Name2BubbleMap =
      std::unordered_map<std::string, std::shared_ptr<storage::DataBubbleBase>>;

  using MutexType = std::shared_mutex;
  using UniqueLock = std::unique_lock<MutexType>;
  using SharedLock = std::shared_lock<MutexType>;

  BubbleInterface() = delete;
  BubbleInterface(const BubbleInterface &) = delete;
  BubbleInterface(BubbleInterface &&) = delete;
  BubbleInterface &operator=(const BubbleInterface &) = delete;
  BubbleInterface &operator=(BubbleInterface &&) = delete;

  BubbleInterface(const Name2AccessorMapPtr &name2accessor_map);

  /** \brief Unloads all data associated with this vertex. */
  bool unload(const bool clear = true);

  /** \brief Inserts data into the databubble of stream name. */
  template <typename DataType>
  bool insert(const std::string &stream_name,
              const typename storage::LockableMessage<DataType>::Ptr &message);

  /** \brief Retrieves data from stream name of timestamp time. */
  template <typename DataType>
  typename storage::LockableMessage<DataType>::Ptr retrieve(
      const std::string &stream_name, const Timestamp &time);

 private:
  template <typename DataType>
  typename storage::DataBubble<DataType>::Ptr getBubble(
      const std::string &stream_name, const bool set_accessor = true);

 private:
  /**
   * \brief Map from stream name to data stream accessor for disk IO.
   * \note weak ptr because the actual map is owned by the "RUN" in graph, while
   * this stream interface class is instantiated by each vertex.
   */
  Name2AccessorMapWeakPtr name2accessor_map_;

  /** \brief Protects access to the name2bubble map. */
  mutable MutexType name2bubble_map_mutex_;

  /** \brief Map from stream name to data bubble for caching. */
  Name2BubbleMap name2bubble_map_;
};

template <typename DataType>
bool BubbleInterface::insert(
    const std::string &stream_name,
    const typename storage::LockableMessage<DataType>::Ptr &message) {
  return getBubble<DataType>(stream_name)->insert(message);
}

template <typename DataType>
auto BubbleInterface::retrieve(const std::string &stream_name,
                               const Timestamp &time) ->
    typename storage::LockableMessage<DataType>::Ptr {
  return getBubble<DataType>(stream_name)->retrieve(time);
}

template <typename DataType>
typename storage::DataBubble<DataType>::Ptr BubbleInterface::getBubble(
    const std::string &stream_name, const bool set_accessor) {
  const UniqueLock lock(name2bubble_map_mutex_);

  const auto bubble =
      name2bubble_map_
          .emplace(stream_name,
                   std::make_shared<storage::DataBubble<DataType>>())
          .first->second;

  if (!set_accessor || bubble->hasAccessor())
    return std::dynamic_pointer_cast<storage::DataBubble<DataType>>(bubble);

  // provide bubble the stream accessor
  const auto name2accessor_map = name2accessor_map_.lock();
  if (!name2accessor_map) {
    CLOG(WARNING, "pose_graph.interface")
        << "Name2Accessor map has expired. Data bubble not iniitalized with an "
           "accessor.";
  } else {
    const auto name2accessor_map_locked = name2accessor_map->sharedLocked();
    const auto accessor_itr = name2accessor_map_locked.get().find(stream_name);
    if (accessor_itr == name2accessor_map_locked.get().end()) {
      CLOG(WARNING, "pose_graph.interface")
          << "Cannot find stream name " << stream_name
          << " from the Name2Accessor map. Databubble not initialized with an "
             "accessor.";
    } else {
      bubble->setAccessor(accessor_itr->second);
    }
  }
  return std::dynamic_pointer_cast<storage::DataBubble<DataType>>(bubble);
}

}  // namespace pose_graph
}  // namespace vtr