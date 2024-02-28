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
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/core.hpp"

#include "vtr_lidar/data_types/costmap.hpp"
#include "vtr_lidar/data_types/point.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace lidar {

struct LidarQueryCache : virtual public tactic::QueryCache {
  PTR_TYPEDEFS(LidarQueryCache);

  // input
  tactic::Cache<const sensor_msgs::msg::PointCloud2> pointcloud_msg;  // ros
  tactic::Cache<const Eigen::MatrixXd> points;  // alternative input non-ros
  tactic::Cache<const tactic::EdgeTransform> T_s_r;

  // preprocessing
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> raw_point_cloud;
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> preprocessed_point_cloud;
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> nn_point_cloud;

  // odometry & mapping
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> undistorted_point_cloud;
#if false  /// store raw point cloud
  tactic::Cache<const pcl::PointCloud<PointWithInfo>> undistorted_raw_point_cloud;
#endif
  tactic::Cache<PointMap<PointWithInfo>> sliding_map_odo;
  tactic::Cache<tactic::EdgeTransform> T_r_m_odo;
  tactic::Cache<Eigen::Matrix<double, 6, 1>> w_m_r_in_r_odo;

  // localization
  tactic::Cache<const PointMap<PointWithInfo>> submap_loc;
  tactic::Cache<const bool> submap_loc_changed;
  tactic::Cache<const tactic::EdgeTransform> T_v_m_loc;

  // intra exp merging async
  tactic::Cache<const tactic::VertexId> intra_exp_merging_async;

  // dynamic detection async
  tactic::Cache<const tactic::VertexId> dynamic_detection_async;
  tactic::Cache<std::pair<cv::Mat, cv::Mat>> rendered_images; 

  // inter exp merging async (priv, curr, T_priv_curr)
  tactic::Cache<const std::tuple<tactic::VertexId, tactic::VertexId,
                                 tactic::EdgeTransform>>
      inter_exp_merging_async;
};

struct LidarOutputCache : virtual public tactic::OutputCache {
  PTR_TYPEDEFS(LidarOutputCache);

  tactic::LockableCache<BaseCostMap> change_detection_costmap;
  tactic::LockableCache<BaseCostMap> ground_extraction_costmap;
  tactic::LockableCache<BaseCostMap> obstacle_detection_costmap;
  tactic::LockableCache<BaseCostMap> terrain_assessment_costmap;
  tactic::LockableCache<BaseCostMap> safe_corridor_costmap;
};

}  // namespace lidar
}  // namespace vtr