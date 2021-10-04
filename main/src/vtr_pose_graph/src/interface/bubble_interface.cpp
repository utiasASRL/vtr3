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
 * \file bubble_interface.cpp
 * \brief BubbleInterface class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_pose_graph/interface/bubble_interface.hpp>

namespace vtr {
namespace pose_graph {

BubbleInterface::BubbleInterface(const Name2AccessorMapPtr &name2accessor_map)
    : name2accessor_map_(name2accessor_map) {}

bool BubbleInterface::unload(const bool clear) {
  const SharedLock lock(name2bubble_map_mutex_);
  bool success = true;
  for (const auto &itr : name2bubble_map_) success &= itr.second->unload(clear);
  return success;
}

bool BubbleInterface::insert(const std::string &stream_name,
                             const MessagePtr &message) {
  return getBubble(stream_name)->insert(message);
}

auto BubbleInterface::retrieve(const std::string &stream_name,
                               const Timestamp &time) -> MessagePtr {
  return getBubble(stream_name)->retrieve(time);
}

auto BubbleInterface::getBubble(const std::string &stream_name,
                                const bool set_accessor)
    -> Name2BubbleMap::mapped_type {
  const UniqueLock lock(name2bubble_map_mutex_);

  const auto bubble =
      name2bubble_map_
          .emplace(stream_name, std::make_shared<storage::DataBubble>())
          .first->second;

  if (!set_accessor || bubble->hasAccessor()) return bubble;

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
  return bubble;
}

}  // namespace pose_graph
}  // namespace vtr
