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
 * \brief OdometryMapMergingModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/odometry_map_merging_module.hpp"

#include "pcl_conversions/pcl_conversions.h"

namespace vtr {
namespace lidar {

using namespace tactic;

void OdometryMapMergingModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config_->map_voxel_size);
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void OdometryMapMergingModule::runImpl(QueryCache &qdata0, const Graph::Ptr &,
                                       const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("new_map", 5);
    publisher_initialized_ = true;
  }

  // Get input and output data
  // input
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m = *qdata.T_r_m_odo;
  // the following has to be copied because we will need to change them
  auto points = *qdata.undistorted_point_cloud;
  // output (construct the map if not exist)
  if (!qdata.new_map_odo) qdata.new_map_odo.emplace(config_->map_voxel_size);
  auto &new_map_odo = *qdata.new_map_odo;
  new_map_odo.T_vertex_map() = T_r_m;

  // Do not update the map if registration failed.
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, "lidar.odometry_map_merging")
        << "Point cloud registration failed - not updating the map.";
    return;
  }

  // Transform points into the map frame
  auto T_m_s = (T_s_r * T_r_m).inverse().matrix().cast<float>();

  auto points_mat = points.getMatrixXfMap(3, PointWithInfo::size(),
                                          PointWithInfo::cartesian_offset());
  auto normal_mat = points.getMatrixXfMap(3, PointWithInfo::size(),
                                          PointWithInfo::normal_offset());

  Eigen::Matrix3f R_tot = T_m_s.block<3, 3>(0, 0);
  Eigen::Vector3f T_tot = T_m_s.block<3, 1>(0, 3);
  points_mat = (R_tot * points_mat).colwise() + T_tot;
  normal_mat = R_tot * normal_mat;
  // Update the point map with the set of new points
  // The update function is called only on subsampled points as the others
  // have no normal
  new_map_odo.update(points);

  if (config_->visualize) {
    PointCloudMsg pc2_msg;
    pcl::toROSMsg(new_map_odo.point_map(), pc2_msg);
    pc2_msg.header.frame_id = "odometry keyframe";
    pc2_msg.header.stamp = *qdata.rcl_stamp;
    map_pub_->publish(pc2_msg);
  }
}

}  // namespace lidar
}  // namespace vtr