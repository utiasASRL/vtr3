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
 * \file honeycomb_conversion_module.cpp
 * \brief HoneycombConversionModuleV2 class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/conversions/honeycomb_conversion_module_v2.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "vtr_lidar/types.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto HoneycombConversionModuleV2::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void HoneycombConversionModuleV2::runImpl(QueryCache &qdata0,
                                          const Graph::Ptr &,
                                          const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    pub_ = qdata.node->create_publisher<PointCloudMsg>("raw_scan", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  /// Input
  const auto &msg = qdata.pointcloud_msg.ptr();

  /// \note:
  /// Assumed Honeycomb setting:
  ///   Spin frequency: 5 Hz
  ///   Laser frequency: 1500 Hz
  ///   Horizontal FOV: 210 degree  (avoid the yaw jump near +-180 degree)
  /// Honeycomb lidar has two beam sides, identified via 0 and 1 from beam_side
  /// field. Since we only get 210 degree horizontal fov when mounting sensor on
  /// the bracket, and we set 0 yaw to x-axis (front), here we assume that we
  /// never get yaw close to +-180 degrees where the number may jump from -180
  /// to +180 or vice versa. With this assumption, the two beams gives the
  /// following data:
  ///   beam side 0: from -180 to 180 in 0.2 seconds
  ///   beam side 1: from 0 to 180 then -180 to 0 in 0.2 seconds
  /// From the Hondycomb documentation, the message time stamp is given at the
  /// center of spin, where beam side 0 is 0 degree and beam side 1 is +-180
  /// degree.

  auto point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(
      msg->width * msg->height, 1);

  // time stamp at the center of the spin
  const double center_time =
      msg->header.stamp.sec + (double)msg->header.stamp.nanosec / 1e9;

  // iterators
  // clang-format off
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_rho(*msg, "range"), iter_theta(*msg, "pitch"), iter_phi(*msg, "yaw");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_beam_side(*msg, "beam_side");
  // clang-format on

  float phi0, phi1;
  size_t i0 = 0, i1 = 0;
  constexpr double PI2 = 2 * M_PI;
  for (size_t idx = 0; iter_x != iter_x.end(); ++idx, ++iter_x, ++iter_y,
              ++iter_z, ++iter_rho, ++iter_theta, ++iter_phi,
              ++iter_beam_side) {
    // cartesian coordinates - copied directly
    point_cloud->at(idx).x = *iter_x;
    point_cloud->at(idx).y = *iter_y;
    point_cloud->at(idx).z = *iter_z;

    // polar coordinates - we add 2pi to beam 1 so that lasers from beam 0 and
    // beam 1 are separated - this is required for nearest neighbor search while
    // avoiding motion distortion issues.
    const auto theta = (*iter_theta) * M_PI / 180;
    auto phi = (*iter_phi) * M_PI / 180;
    if (*iter_beam_side == 0) {
      if (i0 && (phi - phi0) > M_PI)
        phi -= 2 * M_PI;
      else if (i0 && (phi - phi0) < -M_PI)
        phi += 2 * M_PI;
      phi0 = phi;
      i0++;
    } else if (*iter_beam_side == 1) {
      phi += PI2;
      if (i1 && (phi - phi1) > M_PI)
        phi -= 2 * M_PI;
      else if (i1 && (phi - phi1) < -M_PI)
        phi += 2 * M_PI;
      phi1 = phi;
      i1++;
    } else {
      std::string err{"Unknown beam side."};
      CLOG(ERROR, "lidar.honeycomb_converter") << err;
      throw std::runtime_error{err};
    }
    point_cloud->at(idx).rho = *iter_rho;
    point_cloud->at(idx).theta = theta;
    point_cloud->at(idx).phi = phi;

    // time stamp
    double point_time;
    if (*iter_beam_side == 0) {
      // from -180 to 180 in 0.2 seconds
      point_time = center_time + (double)(*iter_phi) / 1800.0;  // 5Hz(180*0.1s)
    } else if (*iter_beam_side == 1) {
      // from 0 to 180 then -180 to 0 in 0.2 seconds
      if (*iter_phi > 0) {
        point_time = center_time + ((double)(*iter_phi) - 180.0) / 1800.0;
      } else {
        point_time = center_time + ((double)(*iter_phi) + 180.0) / 1800.0;
      }
    } else {
      std::string err{"Unknown beam side."};
      CLOG(ERROR, "lidar.honeycomb_converter") << err;
      throw std::runtime_error{err};
    }
    point_cloud->at(idx).time = point_time;
  }

  /// Output
  qdata.raw_point_cloud = point_cloud;

  /// Visualize
  if (config_->visualize) {
    pcl::PointCloud<PointWithInfo> point_cloud_tmp(*point_cloud);
    auto &points = point_cloud_tmp.points;
    std::for_each(points.begin(), points.end(),
                  [&](PointWithInfo &point) { point.time -= center_time; });
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(point_cloud_tmp, *pc2_msg);
    pc2_msg->header.frame_id = *qdata.lidar_frame;
    pc2_msg->header.stamp = *qdata.rcl_stamp;
    pub_->publish(*pc2_msg);
  }
}

}  // namespace lidar
}  // namespace vtr