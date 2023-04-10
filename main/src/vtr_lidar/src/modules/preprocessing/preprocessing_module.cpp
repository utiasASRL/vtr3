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
 * \file preprocessing_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/preprocessing_module.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/features/normal.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

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

auto PreprocessingModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);

  config->crop_range = node->declare_parameter<float>(param_prefix + ".crop_range", config->crop_range);
  config->vertical_angle_res = node->declare_parameter<float>(param_prefix + ".vertical_angle_res", config->vertical_angle_res);
  config->polar_r_scale = node->declare_parameter<float>(param_prefix + ".polar_r_scale", config->polar_r_scale);
  config->r_scale = node->declare_parameter<float>(param_prefix + ".r_scale", config->r_scale);
  config->h_scale = node->declare_parameter<float>(param_prefix + ".h_scale", config->h_scale);
  config->frame_voxel_size = node->declare_parameter<float>(param_prefix + ".frame_voxel_size", config->frame_voxel_size);
  config->nn_voxel_size = node->declare_parameter<float>(param_prefix + ".nn_voxel_size", config->nn_voxel_size);


  config->filter_by_normal_score = node->declare_parameter<bool>(param_prefix + ".filter_normal", config->filter_by_normal_score);
  config->num_sample1 = node->declare_parameter<int>(param_prefix + ".num_sample1", config->num_sample1);
  config->min_norm_score1 = node->declare_parameter<float>(param_prefix + ".min_norm_score1", config->min_norm_score1);

  config->num_sample2 = node->declare_parameter<int>(param_prefix + ".num_sample2", config->num_sample2);
  config->min_norm_score2 = node->declare_parameter<float>(param_prefix + ".min_norm_score2", config->min_norm_score2);
  config->min_normal_estimate_dist = node->declare_parameter<float>(param_prefix + ".min_normal_estimate_dist", config->min_normal_estimate_dist);
  config->max_normal_estimate_angle = node->declare_parameter<float>(param_prefix + ".max_normal_estimate_angle", config->max_normal_estimate_angle);

  config->cluster_num_sample = node->declare_parameter<int>(param_prefix + ".cluster_num_sample", config->cluster_num_sample);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void PreprocessingModule::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("filtered_point_cloud", 5);
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
  auto nn_downsampled_cloud = 
      std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);

  /// Range cropping
  {
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    for (size_t i = 0; i < filtered_point_cloud->size(); ++i) {
      if ((*filtered_point_cloud)[i].rho < config_->crop_range)
        indices.emplace_back(i);
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }

  CLOG(DEBUG, "lidar.preprocessing")
      << "range cropped point cloud size: " << filtered_point_cloud->size();

  /// Grid subsampling

  // Get subsampling of the frame in carthesian coordinates
  voxelDownsample(*filtered_point_cloud, config_->frame_voxel_size);
  voxelDownsample(*nn_downsampled_cloud, config_->nn_voxel_size);

  CLOG(DEBUG, "lidar.preprocessing")
      << "grid subsampled point cloud size: " << filtered_point_cloud->size();

  /// Compute normals using PCA

  // Define the polar neighbors radius in the scaled polar coordinates
  float polar_r = config_->polar_r_scale * config_->vertical_angle_res;

  // Extracts normal vectors of sampled points
  auto norm_scores =
      extractNormal(point_cloud, filtered_point_cloud, polar_r,
                    config_->r_scale, config_->h_scale, config_->num_threads);

  /// Filtering based on normal scores (planarity + linearity)

  if (config_->filter_by_normal_score){
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
  } else {
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    int i = 0;
    for (const auto &point : *filtered_point_cloud) {
      if (i < config_->num_sample1) indices.emplace_back(i);
      i++;
    }
    *filtered_point_cloud =
          pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);

  }
  
  

  CLOG(DEBUG, "lidar.preprocessing")
      << "planarity sampled point size: " << filtered_point_cloud->size();

  /// Filter based on a normal directions
#if false
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
    for (size_t i = 0; i < filtered_point_cloud->size(); i++) {
      if (cluster_scores[i] >= min_score) indices.emplace_back((int)i);
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }
  CLOG(DEBUG, "lidar.preprocessing")
      << "cluster point size: " << filtered_point_cloud->size();
#endif

  /// Delay normal computation until adding the point cloud to the map
  for (auto &p : *filtered_point_cloud) p.normal_score = -1.0;

  CLOG(DEBUG, "lidar.preprocessing")
      << "final subsampled point size: " << filtered_point_cloud->size();

  if (config_->visualize) {
    PointCloudMsg pc2_msg;
    pcl::toROSMsg(*filtered_point_cloud, pc2_msg);
    pc2_msg.header.frame_id = "lidar";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(pc2_msg);
  }

  /// Output
  qdata.preprocessed_point_cloud = filtered_point_cloud;
  qdata.nn_point_cloud = nn_downsampled_cloud;
}

}  // namespace lidar
}  // namespace vtr