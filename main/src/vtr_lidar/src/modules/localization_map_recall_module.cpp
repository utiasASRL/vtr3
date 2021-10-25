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
 * \file localization_map_recall_module.cpp
 * \brief LocalizationMapRecallModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/localization_map_recall_module.hpp"

#include "pcl_conversions/pcl_conversions.h"

namespace vtr {
namespace lidar {

using namespace tactic;

void LocalizationMapRecallModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->map_version = node->declare_parameter<std::string>(param_prefix + ".map_version", config_->map_version);
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void LocalizationMapRecallModule::runImpl(QueryCache &qdata0,
                                          const Graph::ConstPtr &graph) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("curr_map_loc", 5);
    test_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("curr_map_loc_test", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  /// Input
  const auto &map_id = *qdata.map_id;

  if (qdata.curr_map_loc && qdata.curr_map_loc->vertex_id() == map_id) {
    CLOG(DEBUG, "lidar.localization_map_recall")
        << "Map already loaded, simply return. Map size is: "
        << qdata.curr_map_loc->size();
    return;
  } else {
    auto vertex = graph->at(map_id);
    // load the default multi exp pointmap
    if (config_->map_version == "multi_exp_point_map") {
      const auto multi_exp_map_msg =
          vertex->retrieve<MultiExpPointMap<PointWithInfo>>(
              "multi_exp_point_map");
      if (multi_exp_map_msg != nullptr) {
        auto locked_multi_exp_map_msg = multi_exp_map_msg->sharedLocked();
        qdata.curr_map_loc = std::make_shared<PointMap<PointWithInfo>>(
            locked_multi_exp_map_msg.get().getData());
      } else {
        CLOG(WARNING, "lidar.localization_map_recall")
            << "Multi-experience point map not found for vertex " << map_id
            << ", fallback to single experience [point map] stream.";
        const auto map_msg =
            vertex->retrieve<PointMap<PointWithInfo>>("point_map");
        auto locked_map_msg = map_msg->sharedLocked();
        qdata.curr_map_loc = std::make_shared<PointMap<PointWithInfo>>(
            locked_map_msg.get().getData());
      }
    }
    // load a non-default map
    else {
      const auto specified_map_msg =
          vertex->retrieve<PointMap<PointWithInfo>>(config_->map_version);
      if (specified_map_msg != nullptr) {
        auto locked_specified_map_msg = specified_map_msg->sharedLocked();
        qdata.curr_map_loc = std::make_shared<PointMap<PointWithInfo>>(
            locked_specified_map_msg.get().getData());
      } else {
        CLOG(WARNING, "lidar.localization_map_recall")
            << "Specified point map " << config_->map_version
            << "not found for vertex " << map_id
            << ", fallback to single experience [point map] stream.";
        const auto map_msg =
            vertex->retrieve<PointMap<PointWithInfo>>("point_map");
        auto locked_map_msg = map_msg->sharedLocked();
        qdata.curr_map_loc = std::make_shared<PointMap<PointWithInfo>>(
            locked_map_msg.get().getData());
      }
    }
  }

#if false
  /// DEBUGGING: compare with single experience map to double check transformation
  if (config_->visualize) {
    auto vertex = graph->at(map_id);
    const auto map_msg = vertex->retrieve<PointMap<PointWithInfo>>("point_map");
    auto locked_map_msg = map_msg->sharedLocked();
    auto point_map_data = locked_map_msg.get().getData();

    const auto T_v_m = point_map_data.T_vertex_map().matrix();
    auto point_map = point_map_data.point_map();  // makes a copy
    auto map_point_mat = point_map.getMatrixXfMap(
        3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto map_normal_mat = point_map.getMatrixXfMap(
        3, PointWithInfo::size(), PointWithInfo::normal_offset());

    Eigen::Matrix3f R_tot = (T_v_m.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f T_tot = (T_v_m.block<3, 1>(0, 3)).cast<float>();
    map_point_mat = (R_tot * map_point_mat).colwise() + T_tot;

    PointCloudMsg pc2_msg;
    pcl::toROSMsg(point_map, pc2_msg);
    pc2_msg.header.frame_id = "localization keyframe (offset)";
    pc2_msg.header.stamp = *qdata.rcl_stamp;
    test_map_pub_->publish(pc2_msg);
  }
#endif
  /// \note this visualization converts point map from its own frame to the
  /// vertex frame, so can be slow.
  if (config_->visualize) {
    const auto T_v_m = qdata.curr_map_loc->T_vertex_map().matrix();
    auto point_map = qdata.curr_map_loc->point_map();  // makes a copy
    auto map_point_mat = point_map.getMatrixXfMap(
        3, PointWithInfo::size(), PointWithInfo::cartesian_offset());

    Eigen::Matrix3f R_tot = (T_v_m.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f T_tot = (T_v_m.block<3, 1>(0, 3)).cast<float>();
    map_point_mat = (R_tot * map_point_mat).colwise() + T_tot;

    PointCloudMsg pc2_msg;
    pcl::toROSMsg(point_map, pc2_msg);
    pc2_msg.header.frame_id = "localization keyframe (offset)";
    pc2_msg.header.stamp = *qdata.rcl_stamp;
    map_pub_->publish(pc2_msg);
  }
}

}  // namespace lidar
}  // namespace vtr