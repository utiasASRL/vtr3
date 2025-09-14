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
 * \file aeva_conversion_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/conversions/aeva_conversion_module_v2.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto AevaConversionModuleV2::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                           const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->filter_z = node->declare_parameter<bool>(param_prefix + ".filter_z", config->filter_z);
  config->z_threshold = node->declare_parameter<bool>(param_prefix + ".z_threshold", config->z_threshold);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void aevaCart2Polv2(pcl::PointCloud<PointWithInfo> &point_cloud) {
    for (size_t i = 0; i < point_cloud.size(); i++) {
      auto &p = point_cloud[i];
      auto &pm1 = i > 0 ? point_cloud[i - 1] : point_cloud[i];
  
      p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
      p.phi = atan2(p.y, p.x); // + M_PI / 2;
  
      if (i > 0 && (p.phi - pm1.phi) > 1.5 * M_PI)
        p.phi -= 2 * M_PI;
      else if (i > 0 && (p.phi - pm1.phi) < -1.5 * M_PI)
        p.phi += 2 * M_PI;
    }
  }

void AevaConversionModuleV2::run_(QueryCache &qdata0, OutputCache &,
                                const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    pub_ = qdata.node->create_publisher<PointCloudMsg>("raw_point_cloud", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Input
  const auto &msg = qdata.pointcloud_msg.ptr();

  auto point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(msg->width * msg->height, 1);

  CLOG(DEBUG, "lidar.aeva_converter_v2")
      << "raw point cloud size: " << point_cloud->size()
      << ", width: " << msg->width
      << ", height: " << msg->height;

  // iterators
  // clang-format off
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_time(*msg, "time_offset_ns");
  sensor_msgs::PointCloud2ConstIterator<float> iter_velocity(*msg, "velocity");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensty(*msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_flags(*msg, "point_flags_lsb");
  // clang-format on

  size_t idx = 0;
  for (; iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_time, ++iter_velocity, ++iter_intensty, ++iter_flags) {
    if (*iter_z > config_->z_threshold && config_->filter_z)
      continue;
    // cartesian coordinates
    point_cloud->at(idx).x = *iter_x;
    point_cloud->at(idx).y = *iter_y;
    point_cloud->at(idx).z = *iter_z;
    // radial velocity and intensity
    point_cloud->at(idx).radial_velocity = *iter_velocity;
    point_cloud->at(idx).intensity = *iter_intensty;
    // pointwise timestamp [ns]
    point_cloud->at(idx).timestamp = static_cast<int64_t>(*iter_time) + *qdata.stamp;
    // point flags
    uint32_t point_flags = *iter_flags;
    point_cloud->at(idx).line_id = ((point_flags >> 8) & 0xFF);
    point_cloud->at(idx).face_id = ((point_flags >> 22) & 0xF);
    ++idx;
  }
  point_cloud->resize(idx);

  auto filtered_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);

  // Aeva has no polar coordinates, so compute them manually.
  aevaCart2Polv2(*filtered_point_cloud);

  // Output
  qdata.raw_point_cloud = filtered_point_cloud;

  // Visualize
  if (config_->visualize) {
    pcl::PointCloud<PointWithInfo> point_cloud_tmp(*filtered_point_cloud);
    std::for_each(point_cloud_tmp.begin(), point_cloud_tmp.end(),
                  [&](PointWithInfo &point) {
                    point.flex21 = static_cast<float>(
                        (point.timestamp) / 1e9);
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