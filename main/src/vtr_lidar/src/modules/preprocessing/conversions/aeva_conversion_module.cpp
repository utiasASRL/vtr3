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
#include "vtr_lidar/modules/preprocessing/conversions/aeva_conversion_module.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

auto AevaConversionModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                           const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void aevaCart2Pol(pcl::PointCloud<PointWithInfo> &point_cloud) {
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

void AevaConversionModule::run_(QueryCache &qdata0, OutputCache &,
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

  // std::cout << "PointCloud Message Info:\n";
  // std::cout << "Frame ID: " << msg->header.frame_id << std::endl;
  // std::cout << "Timestamp: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl;
  // std::cout << "Width: " << msg->width << ", Height: " << msg->height << std::endl;
  // std::cout << "Point Step: " << msg->point_step << ", Row Step: " << msg->row_step << std::endl;
  // std::cout << "Total Data Size: " << msg->data.size() << " bytes\n";


  auto point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(
      msg->width * msg->height, 1);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
  // sensor_msgs::PointCloud2ConstIterator<double> iter_time(*msg, "t");
  // sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

  for (size_t idx = 0; iter_x != iter_x.end();
       ++idx, ++iter_x, ++iter_y, ++iter_z) {
    // cartesian coordinates
    point_cloud->at(idx).x = *iter_x;
    point_cloud->at(idx).y = *iter_y;
    point_cloud->at(idx).z = *iter_z;
    // point_cloud->at(idx).intensity = *iter_intensity;

    // pointwise timestamp
    //CLOG(DEBUG, "aeva.ouster_converter") << "Timestamp is: " << *iter_time;
    //CLOG(DEBUG, "aeva.ouster_converter") << "Message Header Stamp (sec)" << (msg->header).stamp.sec;
    //CLOG(DEBUG, "aeva.ouster_converter") << "Message Header Stamp (nanosec)" << (msg->header).stamp.nanosec;


    // point_cloud->at(idx).timestamp = static_cast<int64_t>(*iter_time * 1e9);
    // std::cout << "First point info - x: " << *iter_x << " y: " << *iter_y << " z: " << *iter_z << std::endl;
    //CLOG(DEBUG, "aeva.ouster_converter") << "Second point info - x: " << *iter_x << " y: " << *iter_y << " z: " << *iter_z << " timestamp: " << static_cast<int64_t>(*iter_time);
  }


  aevaCart2Pol(*point_cloud);

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


  // auto point_cloud =
  //     std::make_shared<pcl::PointCloud<PointWithInfo>>(points.rows(), 1);

  // for (size_t idx = 0; idx < (size_t)points.rows(); idx++) {
  //   auto &point = point_cloud->at(idx);
  //   // cartesian coordinates
  //   point.x = points(idx, 0);
  //   point.y = points(idx, 1);
  //   point.z = points(idx, 2);

  //   std::cout << 'Points in x: ' << point.x << std::endl;
  //   //
  //   point.radial_velocity = points(idx, 3);
  //   point.intensity = points(idx, 4);
  //   point.timestamp = static_cast<int64_t>(points(idx, 5));
  //   // // IDs
  //   // point.beam_id = points(idx, 6);
  //   // point.line_id = points(idx, 7);
  //   // point.face_id = points(idx, 8);
  //   // point.sensor_id = points(idx, 9);
  // }
  
  // aevaCart2Pol(*point_cloud);

  // // Output
  // qdata.raw_point_cloud = point_cloud;

  // // Visualize
  // if (config_->visualize) {
  //   pcl::PointCloud<PointWithInfo> point_cloud_tmp(*point_cloud);
  //   std::for_each(point_cloud_tmp.begin(), point_cloud_tmp.end(),
  //                 [&](PointWithInfo &point) {
  //                   point.flex21 = static_cast<float>(
  //                       (point.timestamp - *qdata.stamp) / 1e9);
  //                 });
  //   auto pc2_msg = std::make_shared<PointCloudMsg>();
  //   pcl::toROSMsg(point_cloud_tmp, *pc2_msg);
  //   pc2_msg->header.frame_id = "lidar";
  //   pc2_msg->header.stamp = rclcpp::Time(*qdata.stamp);
  //   pub_->publish(*pc2_msg);
  // }
}

}  // namespace lidar
}  // namespace vtr