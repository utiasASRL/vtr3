// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file direct_scan_to_pointmap_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/direct_scan_to_pointmap_module.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace vtr {
namespace radar {

using namespace tactic;

auto ScanToMapModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->scan_resolution = node->declare_parameter<float>(param_prefix + ".scan_resolution", config->scan_resolution);
  config->map_resolution = node->declare_parameter<float>(param_prefix + ".map_resolution", config->map_resolution);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void ScanToMapModule::run_(QueryCache &qdata0, OutputCache &,
                                        const Graph::Ptr &,
                                        const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    return;
  }

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    map_pub_ = qdata.node->create_publisher<PointCloudMsg>("submap_odo", 5);
    // clang-format on
    publisher_initialized_ = true;
  }


  // construct output (construct the map if not exist)
  if (!qdata.sliding_map_odo)
    qdata.sliding_map_odo.emplace(config_->map_resolution);

      // Do not update the map if registration failed.
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, static_name) << "DRO failed - not converting to map.";
    return;
  }

  // Get input and output data
  // input
  const auto T_r_s = qdata.T_s_r->inverse();
  auto& T_r_m_odo = *qdata.T_r_m_odo;
  auto &sliding_map_odo = *qdata.sliding_map_odo;
  const auto &scan_img = *qdata.smoothed_scan;


  if (*qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX) { 
    sliding_map_odo.clear();
    cv::Mat map_img;
    const float scale_factor = config_->scan_resolution / config_->map_resolution;
    cv::resize(scan_img, map_img, cv::Size(), scale_factor, scale_factor, cv::INTER_AREA);




    pcl::PointCloud<PointWithInfo> scan_pc;

    scan_pc.width = map_img.cols;
    scan_pc.height = map_img.rows;
    scan_pc.is_dense = true;
    scan_pc.points.resize(scan_pc.width * scan_pc.height);

    const float res = config_->map_resolution;
    const float x_c = config_->map_resolution * (static_cast<float>(map_img.rows) / 2 + 0.5);
    const float y_c = config_->map_resolution * (static_cast<float>(map_img.cols) / 2 + 0.5);

    map_img.forEach<float_t>([&scan_pc, &map_img, &res, &x_c, &y_c](float_t &pixel, const int *position) -> void {
        PointWithInfo point;
        int r = position[0];
        int c = position[1];
        point.x =  res * static_cast<float>(map_img.rows - r) - x_c; 
        point.y = res * static_cast<float>(c) - y_c;
        point.z = 0.0f;                  
        point.intensity = pixel;

        scan_pc.at(r, c) = point;
      });
    
    sliding_map_odo.update(scan_pc);
    T_r_m_odo = T_r_s;

    CLOG(DEBUG, static_name) << "Subamp has size " << sliding_map_odo.size() << " compared to " << scan_pc.size();
  }
  /// \note this visualization converts point map from its own frame to the
  /// vertex frame, so can be slow.
  if (config_->visualize) {
    // clang-format off
    // publish the map
    {
      auto point_map = sliding_map_odo.point_cloud();  // makes a copy
      auto map_point_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      map_point_mat = T_r_m_odo.matrix().cast<float>() * map_point_mat;

      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_map, pc2_msg);
      pc2_msg.header.frame_id = "odo vertex frame";
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      map_pub_->publish(pc2_msg);
    }

    // clang-format on
  }
}

}  // namespace radar
}  // namespace vtr
