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
  using Name2AccessorMapBase =
      std::unordered_map<std::string,
                         std::shared_ptr<storage::DataStreamAccessorBase>>;
  using Name2AccessorMap = common::SharedLockable<
      std::pair<Name2AccessorMapBase, const std::string>>;
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
  bool insert(const std::string &stream_name, const std::string &stream_type,
              const typename storage::LockableMessage<DataType>::Ptr &message);

  /** \brief Retrieves data from stream name of timestamp time. */
  template <typename DataType>
  typename storage::LockableMessage<DataType>::Ptr retrieve(
      const std::string &stream_name, const std::string &stream_type,
      const Timestamp &time);

  /** \brief Retrieves data from stream name of timestamp time range. */
  template <typename DataType>
  std::vector<typename storage::LockableMessage<DataType>::Ptr> retrieve(
      const std::string &stream_name, const std::string &stream_type,
      const Timestamp &start, const Timestamp &stop);

 private:
  template <typename DataType>
  typename storage::DataBubble<DataType>::Ptr getBubble(
      const std::string &stream_name, const std::string &stream_type,
      const bool set_accessor = true);

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
    const std::string &stream_name, const std::string &stream_type,
    const typename storage::LockableMessage<DataType>::Ptr &message) {
  return getBubble<DataType>(stream_name, stream_type)->insert(message);
}

template <typename DataType>
auto BubbleInterface::retrieve(const std::string &stream_name,
                               const std::string &stream_type,
                               const Timestamp &time) ->
    typename storage::LockableMessage<DataType>::Ptr {
  return getBubble<DataType>(stream_name, stream_type)->retrieve(time);
}

template <typename DataType>
auto BubbleInterface::retrieve(const std::string &stream_name,
                               const std::string &stream_type,
                               const Timestamp &start, const Timestamp &stop)
    -> std::vector<typename storage::LockableMessage<DataType>::Ptr> {
  return getBubble<DataType>(stream_name, stream_type)->retrieve(start, stop);
}

template <typename DataType>
typename storage::DataBubble<DataType>::Ptr BubbleInterface::getBubble(
    const std::string &stream_name, const std::string &stream_type,
    const bool set_accessor) {
  const UniqueLock lock(name2bubble_map_mutex_);

  const auto bubble =
      name2bubble_map_
          .try_emplace(stream_name,
                       std::make_shared<storage::DataBubble<DataType>>())
          .first->second;

  if (!set_accessor || bubble->hasAccessor())
    return std::dynamic_pointer_cast<storage::DataBubble<DataType>>(bubble);

  /// Provide bubble the stream accessor
  // early return if map has expired
  const auto name2accessor_map = name2accessor_map_.lock();
  if (!name2accessor_map) {
    CLOG(WARNING, "pose_graph.interface")
        << "Name2Accessor map has expired. Data bubble not iniitalized with an "
           "accessor.";
    return std::dynamic_pointer_cast<storage::DataBubble<DataType>>(bubble);
  }

  // check if exists (with a shared lock so that it does not block other reads)
  {
    const auto name2accessor_map_locked = name2accessor_map->sharedLocked();
    const auto &name2accessor_map_ref = name2accessor_map_locked.get();

    const auto accessor_itr = name2accessor_map_ref.first.find(stream_name);
    if (accessor_itr != name2accessor_map_ref.first.end()) {
      bubble->setAccessor(accessor_itr->second);
      return std::dynamic_pointer_cast<storage::DataBubble<DataType>>(bubble);
    }
  }

  // perform insertion to the map
  const auto name2accessor_map_locked = name2accessor_map->locked();
  auto &name2accessor_map_ref = name2accessor_map_locked.get();
  const auto accessor_itr = name2accessor_map_ref.first.try_emplace(
      stream_name, std::make_shared<storage::DataStreamAccessor<DataType>>(
                       name2accessor_map_ref.second, stream_name, stream_type));

  bubble->setAccessor(accessor_itr.first->second);
  return std::dynamic_pointer_cast<storage::DataBubble<DataType>>(bubble);
}

}  // namespace pose_graph
}  // namespace vtr