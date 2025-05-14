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
 * \file ouster_conversion_module.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/conversions/ouster_conversion_module.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

namespace {

void ousterCart2Pol(pcl::PointCloud<PointWithInfo> &point_cloud) {
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

auto OusterConversionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->filter_warthog_points = node->declare_parameter<bool>(param_prefix + ".filter_warthog", config->filter_warthog_points);
  config->filter_z_max = node->declare_parameter<float>(param_prefix + ".filter_z_max", config->filter_z_max);
  config->filter_z_min = node->declare_parameter<float>(param_prefix + ".filter_z_min", config->filter_z_min);
  auto filter_radius = node->declare_parameter<float>(param_prefix + ".filter_radius", config->filter_radius_sq);
  config->filter_radius_sq = filter_radius * filter_radius;

  CLOG(INFO, "ouster") << config->filter_z_max << "-" << config->filter_z_min;
  CLOG(INFO, "ouster") << config->filter_warthog_points << "-" << config->filter_radius_sq;

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OusterConversionModule::run_(QueryCache &qdata0, OutputCache &,
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
  const auto &msg = qdata.pointcloud_msg.ptr();

  auto point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(
      msg->width * msg->height, 1);

  // iterators
  // clang-format off
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_time(*msg, "t");
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

  // clang-format on

  for (size_t idx = 0; iter_x != iter_x.end();
       ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_time, ++iter_intensity) {
    // cartesian coordinates
    point_cloud->at(idx).x = *iter_x;
    point_cloud->at(idx).y = *iter_y;
    point_cloud->at(idx).z = *iter_z;
    point_cloud->at(idx).intensity = *iter_intensity;

    // pointwise timestamp
    //CLOG(DEBUG, "lidar.ouster_converter") << "Timestamp is: " << *iter_time;
    //CLOG(DEBUG, "lidar.ouster_converter") << "Message Header Stamp (sec)" << (msg->header).stamp.sec;
    //CLOG(DEBUG, "lidar.ouster_converter") << "Message Header Stamp (nanosec)" << (msg->header).stamp.nanosec;


    point_cloud->at(idx).timestamp = static_cast<int64_t>(*iter_time) + *qdata.stamp;
    //CLOG(DEBUG, "lidar.ouster_converter") << "First point info - x: " << *iter_x << " y: " << *iter_y << " z: " << *iter_z << " timestamp: " << static_cast<int64_t>(*iter_time * 1e9);
    //CLOG(DEBUG, "lidar.ouster_converter") << "Second point info - x: " << *iter_x << " y: " << *iter_y << " z: " << *iter_z << " timestamp: " << static_cast<int64_t>(*iter_time);
  }

  auto filtered_point_cloud =
      std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);

  /// Range cropping
  if (config_->filter_warthog_points){
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    for (size_t i = 0; i < filtered_point_cloud->size(); ++i) {
      auto& p = (*filtered_point_cloud)[i];
      if (p.x*p.x + p.y*p.y > config_->filter_radius_sq || (p.z > config_->filter_z_max || p.z < config_->filter_z_min))
        indices.emplace_back(i);
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }


  // ouster has no polar coordinates, so compute them manually.
  ousterCart2Pol(*filtered_point_cloud);

  // Output
  qdata.raw_point_cloud = filtered_point_cloud;

  // Visualize
  if (config_->visualize) {
    pcl::PointCloud<PointWithInfo> point_cloud_tmp(*filtered_point_cloud);
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