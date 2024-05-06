// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file rangenet_change_detection_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/planning/rangenet_change_detection_module.hpp>
#include "vtr_common/timing/stopwatch.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"
#include "vtr_lidar/data_types/costmap.hpp"
#include <cmath>



namespace vtr {
namespace lidar {

namespace {
  void velodyneCart2Pol(pcl::PointCloud<PointWithInfo> &point_cloud) {
    for (size_t i = 0; i < point_cloud.size(); i++) {
      auto &p = point_cloud[i];
      auto &pm1 = i > 0 ? point_cloud[i - 1] : point_cloud[i];

      p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
      p.phi = atan2(p.y, p.x); // + M_PI / 2;

      if (i > 0 && (p.phi - pm1.phi) > 1.5 * M_PI)
        p.phi -= 2 * M_PI;
      else if (i > 0 && (p.phi - pm1.phi) < -1.5 * M_PI)
        p.phi += 2 * M_PI;
    }
  }

}

using namespace tactic;

auto RangeChangeNetModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<RangeChangeNetModule::Config>();
  auto base_config = std::static_pointer_cast<TorchModule::Config>(config);
  *base_config =  *nn::TorchModule::Config::fromROS(node, param_prefix);

  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);

  //Range Image details
  config->img_width = node->declare_parameter<int>(param_prefix + ".img_width", config->img_width);
  config->img_height = node->declare_parameter<int>(param_prefix + ".img_height", config->img_height);
  config->fov_up = node->declare_parameter<float>(param_prefix + ".fov_up", config->fov_up);
  config->fov_down = node->declare_parameter<float>(param_prefix + ".fov_down", config->fov_down);
  config->range_crop = node->declare_parameter<float>(param_prefix + ".range_crop", config->range_crop);
  config->neighbourhood = node->declare_parameter<int>(param_prefix + ".neighbourhood", config->neighbourhood);
  config->save_nn_point_cloud = node->declare_parameter<bool>(param_prefix + ".save_nn_point_cloud", config->save_nn_point_cloud);
  config->radius_filter = node->declare_parameter<float>(param_prefix + ".radius_filter", config->radius_filter);

  // clang-format on
  return config;
}


void RangeChangeNetModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &graph, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);


  auto nn_point_cloud = *qdata.nn_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;  
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &T_v_m_loc = *qdata.T_v_m_loc;
  const auto &sid_loc = *qdata.sid_loc;
  


  if(!qdata.submap_loc.valid()) {
    CLOG(WARNING, "lidar.range_change") << "Range image requires a map to work";
    return;
  }

  if(!pub_init){
    mask_pub_ = qdata.node->create_publisher<Image>("detection_mask", 5);
    live_range_pub_ = qdata.node->create_publisher<Image>("live_range_image", 5);
    map_range_pub_ = qdata.node->create_publisher<Image>("map_range_image", 5);
    diffpcd_pub_ = qdata.node->create_publisher<PointCloudMsg>("detection_cloud", 5);
  }

  auto& sub_map= *qdata.submap_loc;
  auto map_point_cloud = sub_map.point_cloud();
  auto live_points_mat = nn_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto points_mat = map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());

  CLOG(DEBUG, "lidar.range") << "Before: (" << map_point_cloud[10].x << ", " << map_point_cloud[10].y << ", "<< map_point_cloud[10].z <<")";
  const auto T_r_s = T_s_r.inverse().matrix();
  live_points_mat = T_r_s.cast<float>() * live_points_mat;
  
  const auto T_r_m = (T_r_v_loc * T_v_m_loc).matrix();
  points_mat = T_r_m.cast<float>() * points_mat;
  velodyneCart2Pol(nn_point_cloud);
  velodyneCart2Pol(map_point_cloud);


  CLOG(DEBUG, "lidar.range") << "After: (" << map_point_cloud[10].x << ", " << map_point_cloud[10].y << ", "<< map_point_cloud[10].z <<")";


  RangeImageParams image_params;
  image_params.fov_up = 20 * M_PI / 180;
  image_params.fov_down = -5 * M_PI / 180;
  image_params.crop_range = 10.0;

  RowMatrixXf scan_image = Eigen::MatrixXf::Constant(64, 1024, -1);
  RowMatrixXf mask_image = Eigen::MatrixXf::Zero(64, 1024);
  Eigen::MatrixXi scan_idxs = Eigen::MatrixXi::Constant(64, 1024, -1);
  RowMatrixXf map_image = Eigen::MatrixXf::Constant(64, 1024, -1);

  common::timing::Stopwatch timer;
  timer.start();
  generate_range_image(nn_point_cloud, scan_image, scan_idxs, image_params);
  generate_range_image(map_point_cloud, map_image, image_params);
  timer.stop();
  CLOG(DEBUG, "lidar.range") << "Range image creation takes " << timer;

  using namespace torch::indexing;


  timer.reset();
  auto scan_tensor = torch::from_blob(scan_image.data(), {64, 1024});
  auto map_tensor = torch::from_blob(map_image.data(), {64, 1024});
  auto input = at::unsqueeze(at::stack({scan_tensor, map_tensor}), 0);

  CLOG(DEBUG, "lidar.range") << "GPU load takes " << timer;


  auto tensor = evaluateModel(input, {1, 2, 64, 1024});
  auto mask = at::squeeze(at::argmax(tensor, 1), 0).to(at::kFloat);
  

  torch::from_blob(mask_image.data(), {64, 1024}) = mask;

  unproject_range_image(nn_point_cloud, mask_image, scan_idxs);
  unproject_range_image(*qdata.nn_point_cloud, mask_image, scan_idxs);
  timer.stop();
  CLOG(DEBUG, "lidar.range") << "Running inference takes " << timer;
  timer.reset();

  // filter out non-obstacle points
  std::vector<int> indices;
  indices.reserve(nn_point_cloud.size());
  for (size_t i = 0; i < nn_point_cloud.size(); ++i) {
    if (nn_point_cloud[i].flex24 > 1.5) indices.emplace_back(i);
  }
  pcl::PointCloud<PointWithInfo> obstacle_points(nn_point_cloud, indices);
  auto obstacle_points_mat = obstacle_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  obstacle_points_mat = T_r_v_loc.inverse().matrix().cast<float>() * obstacle_points_mat;

  NanoFLANNAdapter<PointWithInfo> adapter(obstacle_points);
  KDTreeSearchParams search_params;
  KDTreeParams tree_params(10);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  std::vector<int> radii_indices;
  radii_indices.reserve(obstacle_points.size());

  const auto sq_search_radius = config_->radius_filter * config_->radius_filter;
  for (size_t i = 0; i < obstacle_points.size(); i++) {
    // radius search of the closest point
    std::vector<float> dists;
    std::vector<int> indices;
    NanoFLANNRadiusResultSet<float, int> result(sq_search_radius, dists, indices);
    kdtree->radiusSearchCustomCallback(obstacle_points[i].data, result, search_params);

    // filter based on neighbors in map
    if (indices.size() > config_->neighbourhood)
      radii_indices.push_back(i);
  }

  pcl::PointCloud<PointWithInfo> radius_filtered_points(obstacle_points, radii_indices);
  qdata.changed_points.emplace(radius_filtered_points);


  // if (config_->save_nn_point_cloud) {
  //   auto vertex = graph->at(*qdata.vid_odo);

  //   auto nn_scan = std::make_shared<PointScan<PointWithInfo>>();
  //   nn_scan->point_cloud() = nn_point_cloud;
  //   nn_scan->T_vertex_this() = qdata.T_s_r->inverse();
  //   nn_scan->vertex_id() = *qdata.vid_odo;
  //   //
  //   using PointScanLM = storage::LockableMessage<PointScan<PointWithInfo>>;
  //   auto nn_scan_odo_msg =
  //       std::make_shared<PointScanLM>(nn_scan, *qdata.stamp);
  //   vertex->insert<PointScan<PointWithInfo>>(
  //       "nn_point_cloud", "vtr_lidar_msgs/msg/PointScan", nn_scan_odo_msg);

  //   CLOG(DEBUG, "lidar.pipeline") << "Saved nn pointcloud to vertex" << vertex;
  // }

  
  /// publish the transformed pointcloud
  if (config_->visualize) {
    CLOG(DEBUG, "lidar.range") << "Getting to vis took " << timer;

    PointCloudMsg filter_msg;
    pcl::toROSMsg(obstacle_points, filter_msg);
    filter_msg.header.frame_id = "loc vertex frame";
    filter_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    diffpcd_pub_->publish(filter_msg);

    mask_pub_->publish(range_to_image(mask_image));
    live_range_pub_->publish(range_to_image(scan_image));
    map_range_pub_->publish(range_to_image(map_image));

  }
                            
}

// costmap::PixKey RangeChangeNetModule::pointToKey(PointWithInfo &p) {
//     return {std::lround(p.x / config_->resolution), std::lround(p.y / config_->resolution)};
//   }

}  // namespace nn
}  // namespace vtr