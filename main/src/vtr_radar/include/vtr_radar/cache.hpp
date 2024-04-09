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

#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_radar/data_types/point.hpp"
#include "vtr_radar/data_types/pointmap.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace radar {

struct RadarQueryCache : virtual public tactic::QueryCache {
  PTR_TYPEDEFS(RadarQueryCache);

  // input
  tactic::Cache<sensor_msgs::msg::Image> scan_msg;  // from ROS
  tactic::Cache<cv::Mat> scan;                      // from cpp
  tactic::Cache<const tactic::EdgeTransform> T_s_r;

  // preprocessing
  tactic::Cache<float> beta;
  tactic::Cache<pcl::PointCloud<PointWithInfo>> raw_point_cloud;
  tactic::Cache<pcl::PointCloud<PointWithInfo>> preprocessed_point_cloud;

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
};

struct RadarOutputCache : virtual public tactic::OutputCache {
  PTR_TYPEDEFS(RadarOutputCache);
};

}  // namespace radar
}  // namespace vtr