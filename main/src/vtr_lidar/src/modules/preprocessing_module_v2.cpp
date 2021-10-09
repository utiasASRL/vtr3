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
 * \file preprocessing_module_v2.cpp
 * \brief PreprocessingModuleV2 class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing_module_v2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/features/icp_score.hpp"
#include "vtr_lidar/features/normal.hpp"
#include "vtr_lidar/filters/grid_subsampling.hpp"
#include "vtr_lidar/utils.hpp"

namespace vtr {
namespace lidar {

namespace {

template <class PointT>
std::vector<float> getNumberOfNeighbors(const pcl::PointCloud<PointT> &points,
                                        const float &search_radius) {
  // Squared search radius (for nanoflann)
  float r2 = search_radius * search_radius;

  // Build KDTree
  NanoFLANNAdapter<PointT> adapter(points);
  KDTreeParams tree_params(10 /* max leaf */);
  auto index = std::make_unique<KDTree<PointT>>(3, adapter, tree_params);
  index->buildIndex();

  // Search
  std::vector<float> cluster_point_indices;
  cluster_point_indices.reserve(points.size());
  nanoflann::SearchParams search_params;
  search_params.sorted = false;
  for (size_t i = 0; i < points.size(); i++) {
    // initial guess of neighbors size
    std::vector<std::pair<size_t, float>> inds_dists;
    inds_dists.reserve(10);
    // find neighbors
    float point[3] = {points[i].x, points[i].y, points[i].z};
    size_t num_neighbors =
        index->radiusSearch(point, r2, inds_dists, search_params);
    cluster_point_indices.push_back(num_neighbors);
  }
  return cluster_point_indices;
}

}  // namespace

using namespace tactic;

void PreprocessingModuleV2::configFromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config_->num_threads);
#ifdef VTR_DETERMINISTIC
  LOG_IF(config_->num_threads != 1, WARNING) << "Point cloud pre-processor number of threads set to 1 in deterministic mode.";
  config_->num_threads = 1;
#endif
  config_->vertical_angle_res = node->declare_parameter<float>(param_prefix + ".vertical_angle_res", config_->vertical_angle_res);
  config_->polar_r_scale = node->declare_parameter<float>(param_prefix + ".polar_r_scale", config_->polar_r_scale);
  config_->r_scale = node->declare_parameter<float>(param_prefix + ".r_scale", config_->r_scale);
  config_->h_scale = node->declare_parameter<float>(param_prefix + ".h_scale", config_->h_scale);
  config_->frame_voxel_size = node->declare_parameter<float>(param_prefix + ".frame_voxel_size", config_->frame_voxel_size);

  config_->num_sample1 = node->declare_parameter<int>(param_prefix + ".num_sample1", config_->num_sample1);
  config_->min_norm_score1 = node->declare_parameter<float>(param_prefix + ".min_norm_score1", config_->min_norm_score1);

  config_->num_sample2 = node->declare_parameter<int>(param_prefix + ".num_sample2", config_->num_sample2);
  config_->min_norm_score2 = node->declare_parameter<float>(param_prefix + ".min_norm_score2", config_->min_norm_score2);
  config_->min_normal_estimate_dist = node->declare_parameter<float>(param_prefix + ".min_normal_estimate_dist", config_->min_normal_estimate_dist);
  config_->max_normal_estimate_angle = node->declare_parameter<float>(param_prefix + ".max_normal_estimate_angle", config_->max_normal_estimate_angle);

  config_->cluster_num_sample = node->declare_parameter<int>(param_prefix + ".cluster_num_sample", config_->cluster_num_sample);

  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void PreprocessingModuleV2::runImpl(QueryCache &qdata0,
                                    const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("filtered_scan", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Get input point cloud
  const auto point_cloud = qdata.raw_point_cloud.ptr();

  if (point_cloud->size() == 0) {
    std::string err{"Empty point cloud."};
    CLOG(ERROR, "lidar.preprocessing") << err;
    throw std::runtime_error{err};
  }

  CLOG(DEBUG, "lidar.preprocessing")
      << "raw point cloud size: " << point_cloud->size();

  auto filtered_point_cloud =
      std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);

  /// Grid subsampling

  // Get subsampling of the frame in carthesian coordinates
  gridSubsamplingCentersV2(*filtered_point_cloud, config_->frame_voxel_size);

  CLOG(DEBUG, "lidar.preprocessing")
      << "grid subsampled point cloud size: " << filtered_point_cloud->size();

  /// Compute normals and an icp score

  // Define the polar neighbors radius in the scaled polar coordinates
  float polar_r = config_->polar_r_scale * config_->vertical_angle_res;

  // Extracts normal vectors of sampled points
  auto norm_scores =
      extractNormal(point_cloud, filtered_point_cloud, polar_r,
                    config_->r_scale, config_->h_scale, config_->num_threads);

  // Sets better icp score based on distance and incidence angle
  smartICPScore(*filtered_point_cloud);

  /// Filtering based on normal scores (planarity)

  // Remove points with a low normal score
  auto sorted_norm_scores = norm_scores;
  std::sort(sorted_norm_scores.begin(), sorted_norm_scores.end());
  float min_score = sorted_norm_scores[std::max(
      0, (int)sorted_norm_scores.size() - config_->num_sample1)];
  min_score = std::max(config_->min_norm_score1, min_score);
  if (min_score >= 0) {
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    int i = 0;
    for (const auto &point : *filtered_point_cloud) {
      if (point.normal_score >= min_score) indices.emplace_back(i);
      i++;
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }

  CLOG(DEBUG, "lidar.preprocessing")
      << "planarity sampled point size: " << filtered_point_cloud->size();

  /// Filter based on a normal directions

  norm_scores =
      smartNormalScore(*filtered_point_cloud, config_->min_normal_estimate_dist,
                       config_->max_normal_estimate_angle);

  sorted_norm_scores = norm_scores;
  std::sort(sorted_norm_scores.begin(), sorted_norm_scores.end());
  min_score = sorted_norm_scores[std::max(
      0, (int)sorted_norm_scores.size() - config_->num_sample2)];
  min_score = std::max(config_->min_norm_score2, min_score);
  if (min_score >= 0) {
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    int i = 0;
    for (const auto &point : *filtered_point_cloud) {
      if (point.normal_score >= min_score) indices.emplace_back(i);
      i++;
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }

  CLOG(DEBUG, "lidar.preprocessing") << "normal direction sampled point size: "
                                     << filtered_point_cloud->size();

  /// Remove isolated points (mostly points on trees)

  const float search_radius = 2 * config_->frame_voxel_size;
  auto cluster_scores =
      getNumberOfNeighbors(*filtered_point_cloud, search_radius);

  auto sorted_cluster_scores = cluster_scores;
  std::sort(sorted_cluster_scores.begin(), sorted_cluster_scores.end());
  min_score = sorted_cluster_scores[std::max(
      0, (int)sorted_cluster_scores.size() - config_->cluster_num_sample)];
  min_score = std::max((float)1, min_score);  /// \todo config
  if (min_score >= 1) {
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    for (int i = 0; i < filtered_point_cloud->size(); i++) {
      if (cluster_scores[i] >= min_score) indices.emplace_back(i);
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }
  CLOG(DEBUG, "lidar.preprocessing")
      << "cluster point size: " << filtered_point_cloud->size();

  CLOG(DEBUG, "lidar.preprocessing")
      << "final subsampled point size: " << filtered_point_cloud->size();

  if (config_->visualize) {
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(*filtered_point_cloud, *pc2_msg);
    pc2_msg->header.frame_id = *qdata.lidar_frame;
    pc2_msg->header.stamp = *qdata.rcl_stamp;
    filtered_pub_->publish(*pc2_msg);
  }

  /// Output
  qdata.preprocessed_point_cloud = filtered_point_cloud;
}

}  // namespace lidar
}  // namespace vtr