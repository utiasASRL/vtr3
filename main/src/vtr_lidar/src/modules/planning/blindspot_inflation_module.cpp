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
 * \file blindspot_inflation_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/blindspot_inflation_module.hpp"

#include "pcl/features/normal_3d.h"

#include "vtr_lidar/data_types/costmap.hpp"
#include "vtr_lidar/filters/voxel_downsample.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {


using namespace tactic;

auto BlindspotCostmapModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  auto base_config = CostmapInflationModule::Config::fromROS(node, param_prefix);

  auto casted_config =
      std::static_pointer_cast<CostmapInflationModule::Config>(config);
  *casted_config = *base_config;  // copy over base config

  config->blind_spot_radius = node->declare_parameter<float>(param_prefix + ".blind_spot_radius", config->blind_spot_radius);
  config->lifetime = node->declare_parameter<float>(param_prefix + ".lifetime", config->lifetime);

  // clang-format on
  return config;
}

BlindspotCostmapModule::VtrPointCloud BlindspotCostmapModule::assemble_pointcloud(tactic::QueryCache &qdata0, 
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

  // Add reset

  if (*qdata.first_frame) {
    first_frame_ = true;
    all_points_.clear();
  }


  // inputs
  const auto &stamp = *qdata.stamp;
  const auto &sid_loc = *qdata.sid_loc;
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &changed_points = *qdata.changed_points;
  const auto &chain = *output.chain;

  if (!chain.isLocalized()){
    return all_points_;
  } else if (first_frame_){
    active_sid_ = sid_loc;
    first_frame_ = false;
    CLOG(WARNING, "lidar.obstacle_inflation") << chain.T_trunk_target(active_sid_);
  }


  auto concat_pc = pcl::PointCloud<PointWithInfo>(changed_points);

  const auto &T_w_v_loc = chain.pose(sid_loc);
  const auto T_r_w = T_r_v_loc*(T_w_v_loc.inverse());

  auto aligned_point_cloud = concat_pc.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  aligned_point_cloud = T_w_v_loc.matrix().cast<float>() * aligned_point_cloud;

  concat_pc += all_points_;
  all_points_ = pcl::PointCloud<PointWithInfo>(concat_pc);


  auto aligned_concat_cloud = concat_pc.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  aligned_concat_cloud = T_r_w.matrix().cast<float>() * aligned_concat_cloud;

  std::vector<int> indices;
  indices.reserve(concat_pc.size());
  const float r_sq_thresh = config_->blind_spot_radius * config_->blind_spot_radius;
  for (size_t i = 0; i < concat_pc.size(); ++i) {
    auto &p = concat_pc[i];
    auto r_sq = p.x*p.x + p.y*p.y;
    if (r_sq < r_sq_thresh) indices.emplace_back(i);
    else if (abs(stamp - p.timestamp) < config_->lifetime*1e9) indices.emplace_back(i);
  }
  pcl::PointCloud<PointWithInfo> obstacle_points(concat_pc, indices);
  auto aligned_obstacle_cloud = obstacle_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  aligned_obstacle_cloud = T_r_v_loc.inverse().matrix().cast<float>() * aligned_obstacle_cloud;
  
  // pcl::PointCloud<PointWithInfo> reduced_world(concat_pc, indices);
  all_points_ = pcl::PointCloud<PointWithInfo>(all_points_, indices);

  
  
  CLOG(DEBUG, "lidar.obstacle_inflation") << "Point cloud of size " << all_points_.size() << " is connected to sid: " << active_sid_;


  /// publish the transformed pointcloud
  if (config_->visualize) {
    PointCloudMsg concat_msg;
    pcl::toROSMsg(all_points_, concat_msg);
    concat_msg.header.frame_id = "world";
    concat_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    concat_pc_pub_->publish(concat_msg);
  }
  return obstacle_points;
}

}  // namespace lidar
}  // namespace vtr