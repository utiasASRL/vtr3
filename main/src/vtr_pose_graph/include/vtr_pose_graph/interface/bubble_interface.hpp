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
using TimestampRange = storage::DataBubble::TimestampRange;

class BubbleInterface {
 public:
  using MessagePtr = storage::DataBubble::MessagePtr;

  using Name2AccessorMap = common::SharedLockable<std::unordered_map<
      std::string, std::shared_ptr<storage::DataStreamAccessorBase>>>;
  using Name2AccessorMapPtr = std::shared_ptr<Name2AccessorMap>;
  using Name2AccessorMapWeakPtr = std::weak_ptr<Name2AccessorMap>;

  using Name2BubbleMap =
      std::unordered_map<std::string, std::shared_ptr<storage::DataBubble>>;

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
  virtual bool insert(const std::string &stream_name,
                      const MessagePtr &message);

  /** \brief Retrieves data from stream name of timestamp time. */
  MessagePtr retrieve(const std::string &stream_name, const Timestamp &time);

 private:
  Name2BubbleMap::mapped_type getBubble(const std::string &stream_name,
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

}  // namespace pose_graph
}  // namespace vtr