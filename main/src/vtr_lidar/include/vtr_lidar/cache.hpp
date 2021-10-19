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
 * \file cache.hpp
 * \brief LidarQueryCache class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/pointmap/pointmap_v2.hpp"
#include "vtr_lidar/types.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace vtr {
namespace lidar {

struct LidarQueryCache : public tactic::QueryCache {
  using Ptr = std::shared_ptr<LidarQueryCache>;

  tactic::Cache<sensor_msgs::msg::PointCloud2> pointcloud_msg;  // ros input
  tactic::Cache<Eigen::MatrixXd> points;  // alternative input not from ros

  tactic::Cache<pcl::PointCloud<PointWithInfo>> raw_point_cloud;
  tactic::Cache<pcl::PointCloud<PointWithInfo>> preprocessed_point_cloud;
  tactic::Cache<pcl::PointCloud<PointWithInfo>> undistorted_point_cloud;

  tactic::Cache<PointMap<PointWithInfo>> new_map_odo;
  tactic::Cache<PointMap<PointWithInfo>> curr_map_odo;

  tactic::Cache<PointMap<PointWithInfo>> curr_map_loc;

  tactic::Cache<std::string> lidar_frame;
  tactic::Cache<tactic::EdgeTransform> T_s_r;

  tactic::Cache<float> matched_points_ratio;
};
}  // namespace lidar
}  // namespace vtr