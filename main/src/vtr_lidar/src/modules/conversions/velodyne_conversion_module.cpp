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
 * \file velodyne_conversion_module.cpp
 * \brief VelodyneConversionModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vtr_lidar/modules/conversions/velodyne_conversion_module.hpp>

namespace vtr {
namespace lidar {

using namespace tactic;

namespace {

/**
 * \brief Converts cartesian to polar and force polar coordinates to be
 * continuous.
 */
void cart2Pol(const std::vector<PointXYZ> &cart, std::vector<PointXYZ> &polar) {
  polar.clear();
  polar.reserve(cart.size());

  for (size_t i = 0; i < cart.size(); i++) {
    const auto &p = cart[i];

    const float rho = sqrt(p.sq_norm());
    const float theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    float phi = atan2(p.y, p.x) + M_PI / 2;

    if (i > 0 && (phi - polar[i - 1].z) > 1.5 * M_PI)
      phi -= 2 * M_PI;
    else if (i > 0 && (phi - polar[i - 1].z) < -1.5 * M_PI)
      phi += 2 * M_PI;

    polar.emplace_back(rho, theta, phi);
  }
}

}  // namespace

void VelodyneConversionModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void VelodyneConversionModule::runImpl(QueryCache &qdata0,
                                       const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Input
  const auto &msg = qdata.pointcloud_msg.ptr();

  /// Output
  auto &raw_pointcloud_time = *qdata.raw_pointcloud_time.fallback();
  auto &raw_pointcloud_cart = *qdata.raw_pointcloud_cart.fallback();
  auto &raw_pointcloud_pol = *qdata.raw_pointcloud_pol.fallback();

  // Copy over points and time
  const auto N = (size_t)(msg->width * msg->height);
  raw_pointcloud_time.reserve(N);
  raw_pointcloud_cart.reserve(N);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
      iter_y(*msg, "y"), iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<double> iter_time(*msg, "t");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_time) {
    raw_pointcloud_cart.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
    raw_pointcloud_time.push_back(*iter_time);
  }

  // Velodyne has no polar coordinates, so compute them manually.
  cart2Pol(raw_pointcloud_cart, raw_pointcloud_pol);
}

}  // namespace lidar
}  // namespace vtr