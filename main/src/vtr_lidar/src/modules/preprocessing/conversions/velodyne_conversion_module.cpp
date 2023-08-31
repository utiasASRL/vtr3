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
  // Stamp loading was added for unknown reason. Current velodyne data (e.g. from Boreas)
  // has timestamps loaded in absolute time, starting from stamp time. Thus, adding stamp
  // to the timestamp in line 89/90 is incorrect. Perhaps there is data where this is not true...
  const auto &stamp = *qdata.stamp; 
  const auto &points = *qdata.points;

  auto point_cloud =
      std::make_shared<pcl::PointCloud<PointWithInfo>>(points.rows(), 1);

    
    CLOG(INFO, "lidar.velodyne_converter") << "Made PCL";

  for (size_t idx = 0; idx < (size_t)points.rows(); idx++) {
    // cartesian coordinates
    point_cloud->at(idx).x = points(idx, 0);
    point_cloud->at(idx).y = points(idx, 1);
    point_cloud->at(idx).z = points(idx, 2);
    
    // pointwise timestamp
    point_cloud->at(idx).timestamp =
        static_cast<int64_t>(points(idx, 5) * 1e9);// + stamp; See comment above where stamp is loaded
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
                    point.flex21 =
                        static_cast<float>(point.timestamp - stamp) / 1e9;
                  });
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(point_cloud_tmp, *pc2_msg);
    pc2_msg->header.frame_id = "lidar";
    pc2_msg->header.stamp = rclcpp::Time(stamp);
    pub_->publish(*pc2_msg);
  }
}

}  // namespace lidar
}  // namespace vtr