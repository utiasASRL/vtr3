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
 * \file queued_inflation_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/queued_inflation_module.hpp"

#include "pcl/features/normal_3d.h"

#include "vtr_lidar/data_types/costmap.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"


namespace vtr {
namespace lidar {


using namespace tactic;

auto QueuedCostmapModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  auto base_config = CostmapInflationModule::Config::fromROS(node, param_prefix);

  auto casted_config =
      std::static_pointer_cast<CostmapInflationModule::Config>(config);
  *casted_config = *base_config;  // copy over base config

  int hist_size = node->declare_parameter<int>(param_prefix + ".costmap_history_size", config->costmap_history_size);
  config->costmap_history_size = (unsigned) hist_size;
  config->radius_filter = node->declare_parameter<double>(param_prefix + ".radius_filter", config->radius_filter);
  config->neighbourhood = node->declare_parameter<int>(param_prefix + ".neighbourhood", config->neighbourhood);
  // clang-format on
  return config;
}

QueuedCostmapModule::VtrPointCloud QueuedCostmapModule::assemble_pointcloud(tactic::QueryCache &qdata0, 
              tactic::OutputCache &output0, const tactic::Graph::Ptr &) {

  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
  auto &output = dynamic_cast<LidarOutputCache &>(output0);

  /// visualization setup
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    concat_pc_pub_ = qdata.node->create_publisher<PointCloudMsg>("change_detection_concat", 5);
    // clang-format on
    publisher_initialized_ = true;
  }


  // inputs
  const auto &stamp = *qdata.stamp;
  const auto &sid_loc = *qdata.sid_loc;
  const auto &changed_points = *qdata.changed_points;
  const auto &chain = *output.chain;

  // clang-format off
  CLOG(DEBUG, "lidar.obstacle_inflation") << "Adding point cloud to queue: " << stamp;


  auto concat_pc = pcl::PointCloud<PointWithInfo>();

  detected_history.push_front(std::make_pair(sid_loc, changed_points));

  if (detected_history.size() > config_->costmap_history_size)
    detected_history.pop_back();

  for (auto &pair : detected_history) {
    auto &p_loc_sid = pair.first;
    auto &point_cloud = pair.second;
    const auto &T_v_loc_v_detect = chain.T_trunk_target(p_loc_sid);

    auto point_cloud_copy = pcl::PointCloud<PointWithInfo>(point_cloud);

    auto aligned_point_cloud = point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto aligned_norms = point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());


    auto aligned_point_cloud_copy = point_cloud_copy.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto aligned_norms_copy = point_cloud_copy.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());


    aligned_point_cloud_copy = T_v_loc_v_detect.matrix().cast<float>() * aligned_point_cloud;
    aligned_norms_copy = T_v_loc_v_detect.matrix().cast<float>() * aligned_norms;

    for (size_t i = 0; i < point_cloud_copy.size(); ++i){
      point_cloud_copy[i].flex23 = p_loc_sid;
    }

    concat_pc += point_cloud_copy;

    CLOG(DEBUG, "lidar.obstacle_inflation") << "Point cloud of size " << point_cloud.size() << " is connected to sid: " << p_loc_sid;

  }

  NanoFLANNAdapter<PointWithInfo> adapter(concat_pc);
  KDTreeSearchParams search_params;
  KDTreeParams tree_params(10);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  std::vector<int> radii_indices;
  radii_indices.reserve(concat_pc.size());

  const auto sq_search_radius = config_->radius_filter * config_->radius_filter;
  for (size_t i = 0; i < concat_pc.size(); i++) {
    // radius search of the closest point
    std::vector<float> dists;
    std::vector<int> indices;
    NanoFLANNRadiusResultSet<float, int> result(sq_search_radius, dists, indices);
    kdtree->radiusSearchCustomCallback(concat_pc[i].data, result, search_params);

    int other_points_size = 0;
    for (auto &idx : indices) {
      if (concat_pc[idx].flex23 != concat_pc[i].flex23)
        ++other_points_size;
    }
    // filter based on neighbors in map
    if (other_points_size > config_->neighbourhood)
      radii_indices.push_back(i);
  }

  concat_pc = pcl::PointCloud<PointWithInfo>(concat_pc, radii_indices);


  /// publish the transformed pointcloud
  if (config_->visualize) {

    PointCloudMsg concat_msg;
    pcl::toROSMsg(concat_pc, concat_msg);
    concat_msg.header.frame_id = "loc vertex frame";
    concat_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    concat_pc_pub_->publish(concat_msg);
  }

  return concat_pc;
}

}  // namespace lidar
}  // namespace vtr