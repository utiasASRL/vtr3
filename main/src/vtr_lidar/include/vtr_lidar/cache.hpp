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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief LidarQueryCache class definition
 */
#pragma once

#include "vtr_lidar/pointmap/occupancy_grid.hpp"
#include "vtr_lidar/pointmap/pointmap_v2.hpp"
#include "vtr_lidar/types.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace vtr {
namespace lidar {

struct LidarQueryCache : public tactic::QueryCache {
  using Ptr = std::shared_ptr<LidarQueryCache>;

  // input
  tactic::Cache<const sensor_msgs::msg::PointCloud2> pointcloud_msg;  // ros
  tactic::Cache<const Eigen::MatrixXd> points;  // alternative input non-ros
  tactic::Cache<const std::string> lidar_frame;
  tactic::Cache<const tactic::EdgeTransform> T_s_r;

  // preprocessing
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> raw_point_cloud;
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> preprocessed_point_cloud;

  // odometry & mapping
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> undistorted_point_cloud;
#if false  /// store raw point cloud
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> undistorted_raw_point_cloud;
#endif
  tactic::Cache<PointMap<PointWithInfo>> new_map_odo;          // pipeline v1
  tactic::Cache<PointMap<PointWithInfo>> curr_map_odo;         // pipeline v1
  tactic::Cache<const float> matched_points_ratio;             // pipeline v1
  tactic::Cache<PointMap<PointWithInfo>> point_map_odo;        // pipeline v2
  tactic::Cache<tactic::Timestamp> timestamp_odo;              // pipeline v2
  tactic::Cache<tactic::EdgeTransform> T_r_pm_odo;             // pipeline v2
  tactic::Cache<Eigen::Matrix<double, 6, 1>> w_pm_r_in_r_odo;  // pipeline v2

  // localization
  tactic::Cache<const PointMap<PointWithInfo>> curr_map_loc;

  // intra exp merging async
  tactic::Cache<const tactic::VertexId> intra_exp_merging_async;

  // dynamic detection async
  tactic::Cache<const tactic::VertexId> dynamic_detection_async;

  // inter exp merging async
  tactic::Cache<const std::pair<tactic::VertexId, tactic::VertexId>>
      inter_exp_merging_async;
};

struct LidarOutputCache : public tactic::OutputCache {
  using Ptr = std::shared_ptr<LidarOutputCache>;

  tactic::Cache<OccupancyGrid> change_detection_map;
};

}  // namespace lidar
}  // namespace vtr