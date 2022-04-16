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
#include "vtr_lidar/modules/preprocessing/preprocessing_module_v2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/features/normal.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

namespace {

template <class PointT>
void scalePolar(const pcl::PointCloud<PointT> &points, const float &theta_scale,
                const float &phi_scale,
                pcl::PointCloud<pcl::PointXYZ> &output) {
  output.clear();
  output.reserve(points.size());
  for (const auto &point : points)
    output.emplace_back(point.theta * theta_scale, point.phi * phi_scale, 0.0);
}

}  // namespace

using namespace tactic;

auto PreprocessingModuleV2::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);

  config->crop_range = node->declare_parameter<float>(param_prefix + ".crop_range", config->crop_range);

  config->frame_voxel_size = node->declare_parameter<float>(param_prefix + ".frame_voxel_size", config->frame_voxel_size);
#if false
  config->theta_scale = node->declare_parameter<float>(param_prefix + ".theta_scale", config->theta_scale);
  config->phi_scale = node->declare_parameter<float>(param_prefix + ".phi_scale", config->phi_scale);
  config->nn_radius = node->declare_parameter<float>(param_prefix + ".nn_radius", config->nn_radius);
  config->dist_threshold = node->declare_parameter<float>(param_prefix + ".dist_threshold", config->dist_threshold);
#endif
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void PreprocessingModuleV2::run_(QueryCache &qdata0, OutputCache &,
                                 const Graph::Ptr &,
                                 const TaskExecutor::Ptr &) {
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

  CLOG(DEBUG, "lidar.preprocessing")
      << "grid subsampled point cloud size: " << filtered_point_cloud->size();

#if false
  /// Find point on surface
  {
    const auto &phi_scale = config_->phi_scale;
    const auto &theta_scale = config_->theta_scale;
    // query -> filtered
    pcl::PointCloud<pcl::PointXYZ> query;
    query.reserve(filtered_point_cloud->size());
    for (const auto &p : *filtered_point_cloud)
      query.emplace_back(p.theta * theta_scale, p.phi * phi_scale, 0.0);
    // reference -> raw
    pcl::PointCloud<pcl::PointXYZ> reference;
    reference.reserve(point_cloud->size());
    for (const auto &p : *point_cloud)
      reference.emplace_back(p.theta * theta_scale, p.phi * phi_scale, 0.0);

    // create kd-tree of the map (2d only)
    NanoFLANNAdapter<pcl::PointXYZ> adapter(reference);
    KDTreeSearchParams search_params;
    KDTreeParams tree_params(10 /* max leaf */);
    auto kdtree =
        std::make_unique<KDTree<pcl::PointXYZ>>(2, adapter, tree_params);
    kdtree->buildIndex();

    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    //
    const auto sq_radius = config_->nn_radius * config_->nn_radius;
    const auto dist_threshold = config_->dist_threshold;
    for (size_t i = 0; i < query.size(); i++) {
      const auto cart = (*filtered_point_cloud)[i].getVector3fMap();
      //
      std::vector<std::pair<size_t, float>> inds_dists;
      kdtree->radiusSearch(query[i].data, sq_radius, inds_dists, search_params);
      //
      const auto num_neighbors = inds_dists.size() - 1;  // exclude itself
      if (num_neighbors < 2) continue;
      //
      float sum_dist = 0.0;
      for (const auto &ind_dist : inds_dists) {
        const auto &ind = ind_dist.first;
        sum_dist += (cart - (*point_cloud)[ind].getVector3fMap()).norm();
      }
      //
      if ((sum_dist / num_neighbors) > dist_threshold) continue;
      //
      indices.emplace_back(i);
    }
    //
    // *filtered_point_cloud =
    //     pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
    for (const auto &ind : indices) filtered_point_cloud->at(ind).flex24 = 10.0;
  }

  CLOG(DEBUG, "lidar.preprocessing")
      << "surface filtered point cloud size: " << filtered_point_cloud->size();
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
}

}  // namespace lidar
}  // namespace vtr