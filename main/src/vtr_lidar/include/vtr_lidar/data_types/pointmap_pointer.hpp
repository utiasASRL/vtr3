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
 * \file pointmap.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/types.hpp"

#include "vtr_lidar_msgs/msg/point_map_pointer.hpp"

namespace vtr {
namespace lidar {

struct PointMapPointer {
  PTR_TYPEDEFS(PointMapPointer);
  using PointMapPointerMsg = vtr_lidar_msgs::msg::PointMapPointer;

  /// for vtr storage
  /** \brief construct from a storable message */
  static PointMapPointer::Ptr fromStorable(const PointMapPointerMsg& storable);
  /** \brief to storable message */
  PointMapPointerMsg toStorable() const;

  /// data
  tactic::VertexId this_vid = tactic::VertexId::Invalid();
  tactic::VertexId map_vid = tactic::VertexId::Invalid();
  tactic::EdgeTransform T_v_this_map = tactic::EdgeTransform(true);
};

}  // namespace lidar
}  // namespace vtr