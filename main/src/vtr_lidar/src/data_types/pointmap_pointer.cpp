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
 * \file costmap.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/data_types/pointmap_pointer.hpp"

#include "vtr_common/conversions/ros_lgmath.hpp"

namespace vtr {
namespace lidar {

auto PointMapPointer::fromStorable(const PointMapPointerMsg& storable)
    -> PointMapPointer::Ptr {
  //
  auto data = std::make_shared<PointMapPointer>();
  // load vertex ids
  data->this_vid = tactic::VertexId(storable.this_vid);
  data->map_vid = tactic::VertexId(storable.map_vid);
  // load transform
  using namespace vtr::common;
  conversions::fromROSMsg(storable.t_v_this_map, data->T_v_this_map);
  //
  return data;
}

auto PointMapPointer::toStorable() const -> PointMapPointerMsg {
  //
  PointMapPointerMsg storable;
  // save vertex ids
  storable.this_vid = this->this_vid;
  storable.map_vid = this->map_vid;
  // save transform
  using namespace vtr::common;
  conversions::toROSMsg(this->T_v_this_map, storable.t_v_this_map);
  //
  return storable;
}

}  // namespace lidar
}  // namespace vtr