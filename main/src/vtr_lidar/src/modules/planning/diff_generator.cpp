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
 * \file diff_generator.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/diff_generator.hpp"

#include "pcl/features/normal_3d.h"
#include "vtr_lidar/filters/voxel_downsample.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

void velodyneCart2Pol(pcl::PointCloud<PointWithInfo>& point_cloud) {
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto &p = point_cloud[i];
    auto &pm1 = i > 0 ? point_cloud[i - 1] : point_cloud[i];

    p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = atan2(p.y, p.x) + M_PI / 2;

    if (i > 0 && (p.phi - pm1.phi) > 1.5 * M_PI)
      p.phi -= 2 * M_PI;
    else if (i > 0 && (p.phi - pm1.phi) < -1.5 * M_PI)
      p.phi += 2 * M_PI;
  }
}



using namespace tactic;

auto DifferenceDetector::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // change detection
  config->detection_range = node->declare_parameter<float>(param_prefix + ".detection_range", config->detection_range);
  config->minimum_distance = node->declare_parameter<float>(param_prefix + ".minimum_distance", config->minimum_distance);
  config->neighbour_threshold = node->declare_parameter<float>(param_prefix + ".neighbour_threshold", config->neighbour_threshold);

  //filtering
  config->voxel_size = node->declare_parameter<float>(param_prefix + ".voxel_size", config->voxel_size);
  config->angle_weight = node->declare_parameter<float>(param_prefix + ".angle_weight", config->angle_weight);

  // general
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void DifferenceDetector::run_(QueryCache &qdata0, OutputCache &output0,
                                   const Graph::Ptr & /* graph */,
                                   const TaskExecutor::Ptr & /* executor */) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  /// visualization setup
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    diffpcd_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_diff", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // inputs
  const auto &stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &vid_loc = *qdata.vid_loc;
  const auto &sid_loc = *qdata.sid_loc;
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &points = *qdata.raw_point_cloud;
  const auto &submap_loc = *qdata.submap_loc;
  const auto &map_point_cloud = submap_loc.point_cloud();
  const auto &T_v_m_loc = *qdata.T_v_m_loc;

  // clang-format off
  CLOG(INFO, static_name) << "Diff detection for lidar scan at stamp: " << stamp;

  // filter out points that are too far away or too close
  std::vector<int> query_indices;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &pt = points.at(i);
    if (pt.getVector3fMap().norm() < config_->detection_range && pt.getVector3fMap().norm() > config_->minimum_distance)
      query_indices.emplace_back(i);
  }
  pcl::PointCloud<PointWithInfo> query_points(points, query_indices);
  
  voxelDownsample(query_points, config_->voxel_size);

  // Eigen matrix of original data (only shallow copy of ref clouds)
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());


  const auto map_mat = map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_norms_mat = map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  // retrieve the pre-processed scan and convert it to the local map frame
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  const auto T_s_m = (T_s_r * T_r_v_loc * T_v_m_loc).matrix();
  const auto T_m_s = (T_s_r * T_r_v_loc * T_v_m_loc).inverse().matrix();
  aligned_mat = T_m_s.cast<float>() * query_mat;
  aligned_norms_mat = T_m_s.cast<float>() * query_norms_mat;

  pcl::PointCloud<PointWithInfo> aligned_map(map_point_cloud);
  auto aligned_map_mat = aligned_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_map_norms_mat = aligned_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  //Put map into lidar frame
  aligned_map_mat = T_s_m.cast<float>() * map_mat;
  aligned_map_norms_mat = T_s_m.cast<float>() * map_norms_mat;


  velodyneCart2Pol(aligned_map);

  // create kd-tree of the map
  //Could this be done ahead of time and stored?
  NanoFLANNAdapter<PointWithInfo> adapter(aligned_map);//, config_->angle_weight);
  KDTreeSearchParams search_params;
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  std::vector<long unsigned> nn_inds(query_points.size());
  std::vector<float> nn_dists(query_points.size(), -1.0f);
  // compute nearest neighbors and point to point distances
  for (size_t i = 0; i < query_points.size(); i++) {
    KDTreeResultSet result_set(1);
    result_set.init(&nn_inds[i], &nn_dists[i]);
    kdtree->findNeighbors(result_set, query_points[i].data, search_params);
  }

  std::vector<int> diff_indices;
  for (size_t i = 0; i < query_points.size(); i++) {
    aligned_points[i].flex23 = std::sqrt(nn_dists[i]);
    if (aligned_points[i].flex23  > config_->neighbour_threshold){
      diff_indices.push_back(i);
    }
  }

  // add support region
 
  // retrieve the pre-processed scan and convert it to the vertex frame
  pcl::PointCloud<PointWithInfo> aligned_points2(aligned_points);
  // clang-format off
  auto aligned_mat2 = aligned_points2.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat2 = aligned_points2.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on

  // transform to vertex frame
  aligned_mat2 = T_v_m_loc.matrix().cast<float>() * aligned_mat;
  aligned_norms_mat2 = T_v_m_loc.matrix().cast<float>() * aligned_norms_mat;

  pcl::PointCloud<PointWithInfo> filtered_points(aligned_points2, diff_indices);



  /// publish the transformed pointcloud
  if (config_->visualize) {
    PointCloudMsg filter_msg;
    pcl::toROSMsg(filtered_points, filter_msg);
    filter_msg.header.frame_id = "loc vertex frame";
    // pointcloud_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    diffpcd_pub_->publish(filter_msg);
  }


  CLOG(INFO, static_name)
      << "Diff detection for lidar scan at stamp: " << stamp << " - DONE";

}

}  // namespace lidar
}  // namespace vtr