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
 * \file pointscan.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "pcl/point_cloud.h"

#include "vtr_tactic/types.hpp"

#include "vtr_radar_msgs/msg/point_scan.hpp"

namespace vtr {
namespace radar {

template <class PointT>
class PointScan {
 public:
  using PointCloudType = pcl::PointCloud<PointT>;
  PTR_TYPEDEFS(PointScan<PointT>);

  using PointScanMsg = vtr_radar_msgs::msg::PointScan;
  /** \brief Static function that constructs this class from ROS2 message */
  static Ptr fromStorable(const PointScanMsg& storable);
  /** \brief Returns the ROS2 message to be stored */
  PointScanMsg toStorable() const;

  virtual ~PointScan() = default;

  size_t size() const { return point_cloud_.size(); }

  PointCloudType& point_cloud() { return point_cloud_; }
  const PointCloudType& point_cloud() const { return point_cloud_; }

  tactic::VertexId& vertex_id() { return vertex_id_; }
  const tactic::VertexId& vertex_id() const { return vertex_id_; }

  tactic::EdgeTransform& T_vertex_this() { return T_vertex_this_; }
  const tactic::EdgeTransform& T_vertex_this() const { return T_vertex_this_; }

 protected:
  PointCloudType point_cloud_;
  /** \brief the associated vertex id */
  tactic::VertexId vertex_id_ = tactic::VertexId::Invalid();
  /** \brief the transform from this scan/map to its associated vertex */
  tactic::EdgeTransform T_vertex_this_ = tactic::EdgeTransform(true);
};

}  // namespace radar
}  // namespace vtr

#include "vtr_radar/data_types/pointscan.inl"