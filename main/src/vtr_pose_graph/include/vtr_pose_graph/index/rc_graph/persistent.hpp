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
 * \file persistent.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <functional>

#include <vtr_common/utils/hash.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_messages/msg/graph_persistent_id.hpp>

namespace vtr {
namespace pose_graph {

struct PersistentIdHasher {
  std::size_t operator()(
      const vtr_messages::msg::GraphPersistentId &persistent_id) const {
    std::size_t seed = 0;
    common::hash_combine(seed, persistent_id.stamp, persistent_id.robot);
    return seed;
  }
};

}  // namespace pose_graph
}  // namespace vtr

namespace std {

template <>
struct equal_to<vtr_messages::msg::GraphPersistentId> {
  bool operator()(const vtr_messages::msg::GraphPersistentId &a,
                  const vtr_messages::msg::GraphPersistentId &b) const {
    return a.robot == b.robot && a.stamp == b.stamp;
  }
};

}  // namespace std
