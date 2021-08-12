#pragma once

#include <vtr_lidar/grid_subsampling/grid_subsampling.hpp>
#include <vtr_lidar/pointmap/pointmap.hpp>
#include <vtr_lidar/polar_processing/polar_processing.hpp>
#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

struct LidarQueryCache : public QueryCache {
  using Ptr = std::shared_ptr<LidarQueryCache>;

  LidarQueryCache()
      : QueryCache(),
        lidar_frame("lidar_frame", janitor_.get()),
        T_s_r("T_s_r", janitor_.get()),
        raw_pointcloud_time("raw_pointcloud_time", janitor_.get()),
        raw_pointcloud("raw_pointcloud", janitor_.get()),
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
        new_map("new_map", janitor_.get()) {}

  common::cache_ptr<std::string> lidar_frame;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_s_r;
  common::cache_ptr<std::vector<double>> raw_pointcloud_time;
  common::cache_ptr<std::vector<PointXYZ>> raw_pointcloud;
  common::cache_ptr<std::vector<double>> preprocessed_pointcloud_time;
  common::cache_ptr<std::vector<PointXYZ>> preprocessed_pointcloud;
  common::cache_ptr<std::vector<PointXYZ>> normals;
  common::cache_ptr<std::vector<PointXYZ>> undistorted_pointcloud;
  common::cache_ptr<std::vector<PointXYZ>> undistorted_normals;
  common::cache_ptr<std::vector<float>> icp_scores;
  common::cache_ptr<std::vector<float>> normal_scores;
  common::cache_ptr<float> matched_points_ratio;

  common::cache_ptr<lidar::IncrementalPointMap> current_map_odo;
  common::cache_ptr<VertexId> current_map_odo_vid;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance>
      current_map_odo_T_v_m;
  common::cache_ptr<lidar::MultiExpPointMap> current_map_loc;
  common::cache_ptr<lidar::IncrementalPointMap> new_map;
};
}  // namespace tactic
}  // namespace vtr