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
 * \file aeye_conversion_module.cpp
 * \author Nikhil Thiyagarajan, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/conversions/aeye_conversion_module.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

namespace {

void estimateTime(pcl::PointCloud<PointWithInfo> &point_cloud, int64_t time_offset) {
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto &p = point_cloud[i];

    p.timestamp = time_offset
  }
}

}  // namespace

auto AeyeConversionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  config->echo_limit = node->declare_parameter<int>(param_prefix + ".echo_limit", config->echo_limit);
  config->horizontal_downsample = node->declare_parameter<int>(param_prefix + ".downsample_ratio", config->horizontal_downsample);

  // clang-format on
  return config;
}

void AeyeConversionModule::run_(QueryCache &qdata0, OutputCache &,
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
  sensor_msgs::PointCloud2ConstIterator<float> iter_range(*msg, "range"), iter_azimuth(*msg, "azimuth"), iter_elevation(*msg, "elevation");
  sensor_msgs::PointCloud2ConstIterator<float> iter_reflectivity(*msg, "reflectivity"), iter_echo_index(*msg, "echo_index");
  for (size_t idx = 0; iter_x != iter_x.end();
       ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_range, ++iter_azimuth, ++iter_elevation, ++iter_reflectivity) {
    // cartesian coordinates
    point_cloud->at(idx).x = *iter_x;
    point_cloud->at(idx).y = *iter_y;
    point_cloud->at(idx).z = *iter_z;
    // polar coordinates
    point_cloud->at(idx).rho = *iter_range;
    point_cloud->at(idx).phi = *iter_azimuth;
    point_cloud->at(idx).theta = *iter_elevation;
    // reflectivity
    point_cloud->at(idx).intensity = *iter_intensity;
    // echo index
    point_cloud->at(idx).echo_index = *iter_echo_index;
  }

 
  // clang-format on

  // No timestamp data, so estimate
  estimateTime(*point_cloud, *qdata.stamp);


  //If a lower horizontal resolution is acceptable, then set the horizontal downsample > 1.
  //A value of 2 will leave 1/2 the points, in general 1/n points will be retained.

  auto filtered_point_cloud =
    std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);
  CLOG(DEBUG, "lidar.aeye_converter") << "Reducing the point cloud density by " << config_->horizontal_downsample
      << "original size was " << point_cloud->size();
  if (config_->horizontal_downsample > 1) {  
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    for (size_t i = 0; i < filtered_point_cloud->size(); ++i) {
      if (i % config_->horizontal_downsample == 0 ) && ((*point_cloud)[i].echo_index <= config_->echo_limit)
        indices.emplace_back(i);
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  
  }


  // Output
  qdata.raw_point_cloud = filtered_point_cloud;

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