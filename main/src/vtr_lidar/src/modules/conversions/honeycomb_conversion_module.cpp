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
 * \brief HoneycombConversionModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vtr_lidar/modules/conversions/honeycomb_conversion_module.hpp>

namespace vtr {
namespace lidar {

using namespace tactic;

void HoneycombConversionModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void HoneycombConversionModule::runImpl(QueryCache &qdata0,
                                        const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    yaw_pub_ = qdata.node->create_publisher<PointCloudMsg>("raw_points_yaw", 5);
    time_pub_ = qdata.node->create_publisher<PointCloudMsg>("raw_points_time", 5);
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

  // Copy over points and time
  const auto raw_pointcloud_cart = std::make_shared<std::vector<PointXYZ>>();
  const auto raw_pointcloud_pol = std::make_shared<std::vector<PointXYZ>>();
  const auto raw_pointcloud_time = std::make_shared<std::vector<double>>();
  const auto N = (size_t)(msg->width * msg->height);
  raw_pointcloud_cart->reserve(N);
  raw_pointcloud_pol->reserve(N);
  raw_pointcloud_time->reserve(N);

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
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rho,
                                 ++iter_theta, ++iter_phi, ++iter_beam_side) {
    // cartesian coordinates - copied directly
    raw_pointcloud_cart->emplace_back(*iter_x, *iter_y, *iter_z);

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
    raw_pointcloud_pol->emplace_back(*iter_rho, theta, phi);

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
    raw_pointcloud_time->emplace_back(point_time);
  }

  /// Output
  qdata.raw_pointcloud_time = raw_pointcloud_time;
  qdata.raw_pointcloud_cart = raw_pointcloud_cart;
  qdata.raw_pointcloud_pol = raw_pointcloud_pol;

  /// Visualize
  if (config_->visualize) {
    {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      auto cartitr = qdata.raw_pointcloud_cart->begin();
      auto politr = qdata.raw_pointcloud_pol->begin();
      for (; cartitr != qdata.raw_pointcloud_cart->end(); cartitr++, politr++) {
        pcl::PointXYZI pt;
        pt.x = cartitr->x;
        pt.y = cartitr->y;
        pt.z = cartitr->z;
        pt.intensity = politr->z;
        cloud.points.push_back(pt);
      }
      auto pc2_msg = std::make_shared<PointCloudMsg>();
      pcl::toROSMsg(cloud, *pc2_msg);
      pc2_msg->header.frame_id = *qdata.lidar_frame;
      pc2_msg->header.stamp = *qdata.rcl_stamp;

      yaw_pub_->publish(*pc2_msg);
    }
    {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      auto cartitr = qdata.raw_pointcloud_cart->begin();
      auto titr = qdata.raw_pointcloud_time->begin();
      for (; cartitr != qdata.raw_pointcloud_cart->end(); cartitr++, titr++) {
        pcl::PointXYZI pt;
        pt.x = cartitr->x;
        pt.y = cartitr->y;
        pt.z = cartitr->z;
        pt.intensity = *titr - center_time;
        cloud.points.push_back(pt);
      }
      auto pc2_msg = std::make_shared<PointCloudMsg>();
      pcl::toROSMsg(cloud, *pc2_msg);
      pc2_msg->header.frame_id = *qdata.lidar_frame;
      pc2_msg->header.stamp = *qdata.rcl_stamp;

      time_pub_->publish(*pc2_msg);
    }
  }
}

}  // namespace lidar
}  // namespace vtr