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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/localization/localization_map_recall_module.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "vtr_radar/data_types/pointmap_pointer.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

auto LocalizationMapRecallModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->map_version = node->declare_parameter<std::string>(param_prefix + ".map_version", config->map_version);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void LocalizationMapRecallModule::run_(QueryCache &qdata0, OutputCache &,
                                       const Graph::Ptr &graph,
                                       const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("submap_loc", 5);
    test_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("submap_loc_test", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  if(!qdata.radar_data)
  {
    return;
  }



  /// Input
  const auto &vid_loc = *qdata.vid_loc;

  /// load the map pointer
  const auto pointmap_ptr = [&] {
    const auto vertex = graph->at(vid_loc);
    const auto msg = vertex->retrieve<PointMapPointer>(
        "pointmap_ptr", "vtr_radar_msgs/msg/PointMapPointer");
    auto locked_msg = msg->sharedLocked();
    return locked_msg.get().getData();
  }();

  CLOG(INFO, "radar.localization_map_recall")
      << "Loaded pointmap pointer with this_vid " << pointmap_ptr.this_vid
      << " and map_vid " << pointmap_ptr.map_vid;

  /// sanity check
  if (pointmap_ptr.this_vid != vid_loc) {
    CLOG(ERROR, "radar.localization_map_recall")
        << "pointmap pointer this_vid mismatch.";
    throw std::runtime_error("pointmap pointer this_vid mismatch.");
  }

  

  /// load the submap if we have switched to a new one
  if (qdata.submap_loc &&
      qdata.submap_loc->vertex_id() == pointmap_ptr.map_vid) {
    CLOG(INFO, "radar.localization_map_recall")
        << "Map already loaded, simply return. Map size is: "
        << qdata.submap_loc->size();
    // signal that loc map did not change
    qdata.submap_loc_changed.emplace(false);
  } else {
    auto vertex = graph->at(pointmap_ptr.map_vid);
    CLOG(INFO, "radar.localization_map_recall")
        << "Loading map " << config_->map_version << " from vertex " << vid_loc;
    const auto specified_map_msg = vertex->retrieve<PointMap<PointWithInfo>>(
        config_->map_version, "vtr_radar_msgs/msg/PointMap");
    if (specified_map_msg == nullptr) {
      CLOG(ERROR, "radar.localization_map_recall")
          << "Could not find map " << config_->map_version << " at vertex "
          << vid_loc;
      throw std::runtime_error("Could not find map " + config_->map_version +
                               " at vertex " + std::to_string(vid_loc));
    }
    auto locked_specified_map_msg = specified_map_msg->sharedLocked();
    qdata.submap_loc = std::make_shared<PointMap<PointWithInfo>>(
        locked_specified_map_msg.get().getData());
    // signal that loc map did change
    qdata.submap_loc_changed.emplace(true);
  }

  /// update the submap to vertex transformation
  qdata.T_v_m_loc.emplace(pointmap_ptr.T_v_this_map *
                          qdata.submap_loc->T_vertex_this());

  /// \note this visualization converts point map from its own frame to the
  /// vertex frame, so can be slow.
  if (config_->visualize) {
    // clang-format off
    const auto T_v_m = qdata.T_v_m_loc->matrix();
    auto point_map = qdata.submap_loc->point_cloud();  // makes a copy
    auto map_point_mat = point_map.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());

    Eigen::Matrix3f C_v_m = (T_v_m.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_m_v_in_v = (T_v_m.block<3, 1>(0, 3)).cast<float>();
    map_point_mat = (C_v_m * map_point_mat).colwise() + r_m_v_in_v;

    PointCloudMsg pc2_msg;
    pcl::toROSMsg(point_map, pc2_msg);
    pc2_msg.header.frame_id = "loc vertex frame (offset)";
    pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    map_pub_->publish(pc2_msg);
    // clang-format on
  }
}

}  // namespace radar
}  // namespace vtr