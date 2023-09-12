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
 * \file velodyne_conversion_module_v2.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/preprocessing/conversions/velodyne_conversion_module_v2.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

namespace {

void velodyneCart2Pol(pcl::PointCloud<PointWithInfo> &point_cloud) {
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

void estimateTime(pcl::PointCloud<PointWithInfo> &point_cloud, int64_t time_offset, double angular_vel) {
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto &p = point_cloud[i];

    p.timestamp = time_offset + static_cast<int64_t>((p.phi) / angular_vel * 1e9);
  }
}

}  // namespace

auto VelodyneConversionModuleV2::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  config->estimate_time = node->declare_parameter<bool>(param_prefix + ".estimate_time", config->estimate_time);
  config->angular_vel = node->declare_parameter<double>(param_prefix + ".angular_vel", config->angular_vel);
  config->horizontal_downsample = node->declare_parameter<int>(param_prefix + ".downsample_ratio", config->horizontal_downsample);

  // clang-format on
  return config;
}

void VelodyneConversionModuleV2::run_(QueryCache &qdata0, OutputCache &,
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
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
  for (size_t idx = 0; iter_x != iter_x.end();
       ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    // cartesian coordinates
    point_cloud->at(idx).x = *iter_x;
    point_cloud->at(idx).y = *iter_y;
    point_cloud->at(idx).z = *iter_z;
    point_cloud->at(idx).intensity = *iter_intensity;
  }

 
  // clang-format on
  

  // Velodyne has no polar coordinates, so compute them manually.
  velodyneCart2Pol(*point_cloud);

  if (config_->estimate_time){
      CLOG(INFO, "lidar.velodyne_converter_v2") << "Timings wil be estimated from yaw angle";
      estimateTime(*point_cloud, *qdata.stamp, config_->angular_vel);
  } else {
    try {
      try{
        sensor_msgs::PointCloud2ConstIterator<double> iter_time(*msg, "t");
        // pointwise timestamp
        for (size_t idx = 0; iter_time != iter_time.end();
          ++idx, ++iter_time) {
            point_cloud->at(idx).timestamp = static_cast<int64_t>(*iter_time * 1e9);
        }
        CLOG(INFO, "lidar.velodyne_converter_v2") << "Timings from t";
      }
      catch (...){
        sensor_msgs::PointCloud2ConstIterator<double> iter_time(*msg, "time");
        // pointwise timestamp
        for (size_t idx = 0; iter_time != iter_time.end();
          ++idx, ++iter_time) {
            point_cloud->at(idx).timestamp = static_cast<int64_t>(*iter_time * 1e9);
        }
        CLOG(INFO, "lidar.velodyne_converter_v2") << "Timings from time";

      }
    }
    catch(...){
      CLOG(INFO, "lidar.velodyne_converter_v2") << "Timings wil be estimated from yaw angle";
      estimateTime(*point_cloud, *qdata.stamp, config_->angular_vel);
    }
  }

  

  //If a lower horizontal resolution is acceptable, then set the horizontal downsample > 1.
  //A value of 2 will leave 1/2 the points, in general 1/n points will be retained.

  auto filtered_point_cloud =
    std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);
  CLOG(DEBUG, "lidar.velodyne_converter_v2") << "Reducing the point cloud density by " << config_->horizontal_downsample
      << "original size was " << point_cloud->size();
  if (config_->horizontal_downsample > 1) {  
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    for (size_t i = 0; i < filtered_point_cloud->size(); ++i) {
      if (i % config_->horizontal_downsample == 0 )//&& (*point_cloud)[i].phi < -M_PI)
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