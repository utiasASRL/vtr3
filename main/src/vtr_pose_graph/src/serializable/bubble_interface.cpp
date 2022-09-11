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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_pose_graph/serializable/bubble_interface.hpp"

namespace vtr {
namespace pose_graph {

BubbleInterface::BubbleInterface(const Name2AccessorMapPtr &name2accessor_map)
    : name2accessor_map_(name2accessor_map) {}

bool BubbleInterface::unload(const bool clear) {
  SharedLock lock(name2bubble_map_mutex_);
  bool success = true;
  for (const auto &itr : name2bubble_map_) success &= itr.second->unload(clear);
  return success;
}

}  // namespace pose_graph
}  // namespace vtr
