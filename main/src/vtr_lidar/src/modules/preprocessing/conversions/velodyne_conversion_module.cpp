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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/conversions/velodyne_conversion_module.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

namespace {

std::vector<int> limitFOV(const pcl::PointCloud<PointWithInfo> &point_cloud,
                          const float &center, const float &fov) {
  //
  const float fov2 = fov / 2.0;

  std::vector<int> indices;
  indices.reserve(point_cloud.size());

  for (size_t i = 0; i < point_cloud.size(); i++) {
    const auto &p = point_cloud[i];
    const auto phi = std::atan2(p.y, p.x);
    auto diff = std::abs(phi - center);
    diff = std::min(diff, static_cast<float>(2 * M_PI - diff));
    if (diff < fov2) indices.emplace_back(i);
  }

  return indices;
}

void velodyneCart2Pol(pcl::PointCloud<PointWithInfo> &point_cloud) {
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto &p = point_cloud[i];
    auto &pm1 = i > 0 ? point_cloud[i - 1] : point_cloud[i];

    p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = atan2(p.y, p.x) + M_PI / 2;

    if (i > 0 && (p.phi - pm1.phi) > 1.5 * M_PI)
      p.phi -= 2 * M_PI;
    else if (i > 0 && (p.phi - pm1.phi) < -1.5 * M_PI)
      p.phi += 2 * M_PI;
  }
}

}  // namespace

auto VelodyneConversionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->use_fov_limit = node->declare_parameter<bool>(param_prefix + ".use_fov_limit", config->use_fov_limit);
  config->fov_center = node->declare_parameter<float>(param_prefix + ".fov_center", config->fov_center);
  config->fov = node->declare_parameter<float>(param_prefix + ".fov", config->fov);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void VelodyneConversionModule::run_(QueryCache &qdata0, OutputCache &,
                                    const Graph::Ptr &,
                                    const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    pub_ = qdata.node->create_publisher<PointCloudMsg>("raw_point_cloud", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Input
  const auto &points = *qdata.points;

  auto point_cloud =
      std::make_shared<pcl::PointCloud<PointWithInfo>>(points.rows(), 1);

  for (size_t idx = 0; idx < (size_t)points.rows(); idx++) {
    // cartesian coordinates
    point_cloud->at(idx).x = points(idx, 0);
    point_cloud->at(idx).y = points(idx, 1);
    point_cloud->at(idx).z = points(idx, 2);

    // pointwise timestamp
    point_cloud->at(idx).timestamp = static_cast<int64_t>(points(idx, 5) * 1e9);
  }

  //
  if (config_->use_fov_limit) {
    const auto center_rad = config_->fov_center * M_PI / 180;
    const auto fov_rad = config_->fov * M_PI / 180;
    auto indices = limitFOV(*point_cloud, center_rad, fov_rad);
    *point_cloud = pcl::PointCloud<PointWithInfo>(*point_cloud, indices);
  }

  // Velodyne has no polar coordinates, so compute them manually.
  velodyneCart2Pol(*point_cloud);

  // Output
  qdata.raw_point_cloud = point_cloud;

  // Visualize
  if (config_->visualize) {
    pcl::PointCloud<PointWithInfo> point_cloud_tmp(*point_cloud);
    std::for_each(point_cloud_tmp.begin(), point_cloud_tmp.end(),
                  [&](PointWithInfo &point) {
                    point.flex21 = static_cast<float>(
                        (point.timestamp - *qdata.stamp) / 1e9);
                  });
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(point_cloud_tmp, *pc2_msg);
    pc2_msg->header.frame_id = "lidar";
    pc2_msg->header.stamp = rclcpp::Time(*qdata.stamp);
    pub_->publish(*pc2_msg);
  }
}

}  // namespace lidar
}  // namespace vtr