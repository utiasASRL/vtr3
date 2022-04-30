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
 * \file odometry_map_maintenance_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/odometry/odometry_map_maintenance_module_v2.hpp"

#include "pcl/features/normal_3d.h"
#include "pcl_conversions/pcl_conversions.h"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto OdometryMapMaintenanceModuleV2::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config->map_voxel_size);

  config->point_life_time = node->declare_parameter<float>(param_prefix + ".point_life_time", config->point_life_time);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OdometryMapMaintenanceModuleV2::run_(QueryCache &qdata0, OutputCache &,
                                          const Graph::Ptr &,
                                          const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    scan_pub_ = qdata.node->create_publisher<PointCloudMsg>("udist_point_cloud", 5);
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("submap_odo", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // construct output (construct the map if not exist)
  if (!qdata.sliding_map_odo)
    qdata.sliding_map_odo.emplace(config_->map_voxel_size);

  // Get input and output data
  // input
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m_odo = *qdata.T_r_m_odo;
  auto &sliding_map_odo = *qdata.sliding_map_odo;
  // the following has to be copied because we need to change them
  auto points = *qdata.undistorted_point_cloud;

  // Do not update the map if registration failed.
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, "lidar.odometry_map_maintenance")
        << "Point cloud registration failed - not updating the map.";
    return;
  }

  // Transform points into the map frame
  auto T_m_s = (T_s_r * T_r_m_odo).inverse().matrix().cast<float>();
  // clang-format off
  auto points_mat = points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto normal_mat = points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on
  points_mat = T_m_s * points_mat;
  normal_mat = T_m_s * normal_mat;

  // update the map with new points and refresh their life time
  auto update_cb = [&config = config_](bool, PointWithInfo &curr_pt,
                                       const PointWithInfo &) {
    curr_pt.life_time = config->point_life_time;
  };
  sliding_map_odo.update(points, update_cb);

  // update normal vector
  NanoFLANNAdapter<PointWithInfo> adapter(sliding_map_odo.point_cloud());
  KDTreeSearchParams search_param;
  KDTreeParams tree_param(/* max_leaf */ 10);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_param);
  kdtree->buildIndex();
  const auto search_radius = sliding_map_odo.dl() * 3.0;
  const auto sq_radius = search_radius * search_radius;
  auto update_normal_cb = [&point_cloud = sliding_map_odo.point_cloud(),
                           &kdtree, &sq_radius,
                           &search_param](bool, PointWithInfo &curr_pt,
                                          const PointWithInfo &) {
    std::vector<float> dists;
    std::vector<int> indices;
    NanoFLANNRadiusResultSet<float, int> result(sq_radius, dists, indices);
    kdtree->radiusSearchCustomCallback(curr_pt.data, result, search_param);

    if (indices.size() < 4) return;

    // get points for computation
    const pcl::PointCloud<PointWithInfo> neigobors(point_cloud, indices);

    // Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;
    // Estimate the XYZ centroid
    pcl::compute3DCentroid(neigobors, xyz_centroid);
    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix(neigobors, xyz_centroid, covariance_matrix);
    // Compute pca
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.compute(covariance_matrix);
    const auto &eigenvectors = es.eigenvectors();
    const auto &eigenvalues = es.eigenvalues();

    // normal direction
    curr_pt.getNormalVector3fMap() = Eigen::Vector3f(eigenvectors.col(0));
    // normal score
    curr_pt.normal_score = (eigenvalues(1) - eigenvalues(0)) / eigenvalues(2);
  };
  sliding_map_odo.update(points, update_normal_cb);

  // filter based on life time
  auto filter_life_time_cb = [](PointWithInfo &query_pt) {
    query_pt.life_time -= 1.0;
    return bool(query_pt.life_time > 0.0);
  };
  sliding_map_odo.filter(filter_life_time_cb);

  CLOG(DEBUG, "lidar.odometry_map_maintenance")
      << "Updated point map size is: " << sliding_map_odo.point_cloud().size();

  /// \note this visualization converts point map from its own frame to the
  /// vertex frame, so can be slow.
  if (config_->visualize) {
    // clang-format off
    const auto T_v_m = sliding_map_odo.T_vertex_this().matrix().cast<float>();
    // publish the map
    {
      auto point_map = sliding_map_odo.point_cloud();  // makes a copy
      auto map_point_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      map_point_mat = T_v_m * map_point_mat;

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_map, pc2_msg);
      pc2_msg.header.frame_id = "odo vertex frame";
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      map_pub_->publish(pc2_msg);
    }
    // publish the aligned points
    {
      auto scan_in_vf = points;
      auto points_mat = scan_in_vf.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      points_mat = T_v_m * points_mat;

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(scan_in_vf, pc2_msg);
      pc2_msg.header.frame_id = "odo vertex frame";
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      scan_pub_->publish(pc2_msg);
    }
    // clang-format on
  }
}

}  // namespace lidar
}  // namespace vtr