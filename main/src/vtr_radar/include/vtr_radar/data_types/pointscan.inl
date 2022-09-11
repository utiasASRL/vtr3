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
 * \file pointscan.inl
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_radar/data_types/pointscan.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_common/conversions/ros_lgmath.hpp"

namespace vtr {
namespace radar {

template <class PointT>
auto PointScan<PointT>::fromStorable(const PointScanMsg& storable) -> Ptr {
  // construct with dl
  auto data = std::make_shared<PointScan<PointT>>();
  // load point cloud data
  pcl::fromROSMsg(storable.point_cloud, data->point_cloud_);
  // load vertex id
  data->vertex_id_ = tactic::VertexId(storable.vertex_id);
  // load transform
  using namespace vtr::common;
  conversions::fromROSMsg(storable.t_vertex_this, data->T_vertex_this_);
  return data;
}

template <class PointT>
auto PointScan<PointT>::toStorable() const -> PointScanMsg {
  PointScanMsg storable;
  // save point cloud data
  pcl::toROSMsg(this->point_cloud_, storable.point_cloud);
  // save vertex id
  storable.vertex_id = this->vertex_id_;
  // save transform
  using namespace vtr::common;
  conversions::toROSMsg(this->T_vertex_this_, storable.t_vertex_this);
  return storable;
}

}  // namespace radar
}  // namespace vtr