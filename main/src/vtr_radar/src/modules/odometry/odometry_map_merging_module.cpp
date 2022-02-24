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
 * \file odometry_map_merging_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_map_merging_module.hpp"

#include "pcl_conversions/pcl_conversions.h"

namespace vtr {
namespace radar {

using namespace tactic;

auto OdometryMapMergingModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config->map_voxel_size);
  config->crop_range_front = node->declare_parameter<float>(param_prefix + ".crop_range_front", config->crop_range_front);
  config->back_over_front_ratio = node->declare_parameter<float>(param_prefix + ".back_over_front_ratio", config->back_over_front_ratio);

  config->point_life_time = node->declare_parameter<float>(param_prefix + ".point_life_time", config->point_life_time);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OdometryMapMergingModule::run_(QueryCache &qdata0, OutputCache &,
                                    const Graph::Ptr &,
                                    const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("new_map", 5);
    publisher_initialized_ = true;
  }

  // construct output (construct the map if not exist)
  if (!qdata.point_map_odo)
    qdata.point_map_odo.emplace(config_->map_voxel_size);

  // Get input and output data
  // input
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m_odo = *qdata.T_r_m_odo;
  auto &point_map_odo = *qdata.point_map_odo;
  // the following has to be copied because we need to change them
  auto points = *qdata.undistorted_point_cloud;

  // Do not update the map if registration failed.
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, "radar.odometry_map_merging")
        << "Point cloud registration failed - not updating the map.";
    return;
  }

  // Transform points into the map frame
  auto T_m_s = (T_s_r * T_r_m_odo).inverse().matrix().cast<float>();
  // clang-format off
  auto points_mat = points.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto normal_mat = points.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());
  // clang-format on
  Eigen::Matrix3f C_m_s = T_m_s.block<3, 3>(0, 0);
  Eigen::Vector3f r_s_m_in_m = T_m_s.block<3, 1>(0, 3);
  points_mat = (C_m_s * points_mat).colwise() + r_s_m_in_m;
  normal_mat = C_m_s * normal_mat;

  /// Update the point map with the set of new points

  // set life time of the points in this map
  if (config_->point_life_time >= 0.0) {
    // reduce life time of points in the map, remove those that have expired
    point_map_odo.subtractLifeTime();
    for (auto &point : points) point.life_time = config_->point_life_time;
  }

  // The update function is called only on subsampled points
  point_map_odo.update(points);

  // crop box filter
  point_map_odo.crop(T_r_m_odo.matrix().cast<float>(),
                     config_->crop_range_front, config_->back_over_front_ratio);

  CLOG(DEBUG, "radar.odometry_map_merging")
      << "Updated point map size is: " << point_map_odo.point_map().size();

  /// \note this visualization converts point map from its own frame to the
  /// vertex frame, so can be slow.
  if (config_->visualize) {
    // clang-format off
    const auto T_v_m = point_map_odo.T_vertex_map().matrix();
    auto point_map = point_map_odo.point_map();  // makes a copy
    auto map_point_mat = point_map.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());

    Eigen::Matrix3f C_v_m = (T_v_m.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_m_v_in_v = (T_v_m.block<3, 1>(0, 3)).cast<float>();
    map_point_mat = (C_v_m * map_point_mat).colwise() + r_m_v_in_v;

    PointCloudMsg pc2_msg;
    pcl::toROSMsg(point_map, pc2_msg);
    pc2_msg.header.frame_id = "odometry keyframe";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    map_pub_->publish(pc2_msg);
  }
}

}  // namespace radar
}  // namespace vtr