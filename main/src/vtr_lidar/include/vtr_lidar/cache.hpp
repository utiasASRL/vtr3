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

#include <vtr_lidar/grid_subsampling/grid_subsampling.hpp>
#include <vtr_lidar/pointmap/pointmap.hpp>
#include <vtr_lidar/polar_processing/polar_processing.hpp>
#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vtr {
namespace lidar {

struct LidarQueryCache : public tactic::QueryCache {
  using Ptr = std::shared_ptr<LidarQueryCache>;

  LidarQueryCache()
      : tactic::QueryCache(),
        lidar_frame("lidar_frame", janitor_.get()),
        T_s_r("T_s_r", janitor_.get()),
        pointcloud_msg("pointcloud_msg", janitor_.get()),
        raw_pointcloud_time("raw_pointcloud_time", janitor_.get()),
        raw_pointcloud_cart("raw_pointcloud_cart", janitor_.get()),
        raw_pointcloud_pol("raw_pointcloud_pol", janitor_.get()),
        preprocessed_pointcloud_time("preprocessed_pointcloud_time",
                                     janitor_.get()),
        preprocessed_pointcloud("preprocessed_pointcloud", janitor_.get()),
        normals("normals", janitor_.get()),
        undistorted_pointcloud("undistorted_pointcloud", janitor_.get()),
        undistorted_normals("undistorted_normals", janitor_.get()),
        icp_scores("icp_scores", janitor_.get()),
        normal_scores("normal_scores", janitor_.get()),
        matched_points_ratio("matched_points_ratio", janitor_.get()),
        current_map_odo("current_map_odo", janitor_.get()),
        current_map_odo_vid("current_map_odo_vid", janitor_.get()),
        current_map_odo_T_v_m("current_map_odo_T_v_m", janitor_.get()),
        current_map_loc("current_map_loc", janitor_.get()),
        current_map_loc_vid("current_map_loc_vid", janitor_.get()),
        new_map("new_map", janitor_.get()),
        new_map_T_v_m("new_map_T_v_m", janitor_.get()) {}

  common::cache_ptr<std::string> lidar_frame;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_s_r;
  common::cache_ptr<sensor_msgs::msg::PointCloud2> pointcloud_msg;
  common::cache_ptr<std::vector<double>> raw_pointcloud_time;
  common::cache_ptr<std::vector<PointXYZ>> raw_pointcloud_cart;
  common::cache_ptr<std::vector<PointXYZ>> raw_pointcloud_pol;
  common::cache_ptr<std::vector<double>> preprocessed_pointcloud_time;
  common::cache_ptr<std::vector<PointXYZ>> preprocessed_pointcloud;
  common::cache_ptr<std::vector<PointXYZ>> normals;
  common::cache_ptr<std::vector<PointXYZ>> undistorted_pointcloud;
  common::cache_ptr<std::vector<PointXYZ>> undistorted_normals;
  common::cache_ptr<std::vector<float>> icp_scores;
  common::cache_ptr<std::vector<float>> normal_scores;
  common::cache_ptr<float> matched_points_ratio;

  common::cache_ptr<IncrementalPointMap> current_map_odo;
  common::cache_ptr<tactic::VertexId> current_map_odo_vid;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance>
      current_map_odo_T_v_m;
  common::cache_ptr<MultiExpPointMap> current_map_loc;
  common::cache_ptr<tactic::VertexId> current_map_loc_vid;
  common::cache_ptr<IncrementalPointMap> new_map;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> new_map_T_v_m;
};
}  // namespace lidar
}  // namespace vtr