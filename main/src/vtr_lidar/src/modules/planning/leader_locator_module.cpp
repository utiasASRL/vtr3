// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file leader_locator.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/leader_locator_module.hpp"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace vtr {
namespace lidar {

using namespace tactic;

auto LeaderLocator::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // change detection
  config->maximum_distance = node->declare_parameter<float>(param_prefix + ".maximum_distance", config->maximum_distance);
  config->minimum_distance = node->declare_parameter<float>(param_prefix + ".minimum_distance", config->minimum_distance);
  config->intensity_threshold = node->declare_parameter<float>(param_prefix + ".intensity_threshold", config->intensity_threshold);
  config->publish = node->declare_parameter<bool>(param_prefix + ".publish", config->publish);

  // clang-format on
  return config;
}

void LeaderLocator::run_(QueryCache &qdata0, OutputCache &,
                                   const Graph::Ptr & /* graph */,
                                   const TaskExecutor::Ptr & /* executor */) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // inputs
  const auto &stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &points = *qdata.raw_point_cloud;

  if (!publisher_initialized_) {
    distance_pub_ = qdata.node->create_publisher<FloatMsg>("leader_distance", rclcpp::QoS(1).best_effort().durability_volatile());
    publisher_initialized_ = true;
  }

  // clang-format off

  // filter out points that are too far away or too close
  std::vector<int> query_indices;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto &pt = points.at(i);
    if (pt.getVector3fMap().norm() < config_->maximum_distance && pt.getVector3fMap().norm() > config_->minimum_distance
            && pt.intensity > config_->intensity_threshold)
      query_indices.emplace_back(i);
  }
  auto query_points = std::make_shared<pcl::PointCloud<PointWithInfo>>(points, query_indices);
  
  // Eigen matrix of original data (only shallow copy of ref clouds)
  
//   const auto query_norms_mat = query_points->getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointWithInfo> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (query_points);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () < 10) {
    CLOG(WARNING, static_name) << "Fewer than 10 points of the leader were detected!";
    return;
  }

  auto marker = std::make_shared<pcl::PointCloud<PointWithInfo>>(*query_points, inliers->indices);
  const auto centroid = marker->getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset()).rowwise().mean();
  Eigen::Vector3d normal{
    coefficients->values[0],
    coefficients->values[1],
    coefficients->values[2]
  };
  normal.normalize();
  Eigen::Vector4d leader_position = centroid.cast<double>();
  leader_position.head<3>() += 0.0454*normal;

  Eigen::Vector4d leader_in_follower = T_s_r.inverse().matrix() * leader_position;
  leader_in_follower(2) -= 10.75*0.0254;


  CLOG(DEBUG, static_name)
      << "Plane fit of Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3];
  CLOG(DEBUG, static_name) << "Centroid \n" << centroid;
  CLOG(DEBUG, static_name) << "Leader position \n" << leader_in_follower;

  const float leader_dist = leader_in_follower.head<2>().norm();
  CLOG(INFO, static_name) << "Leader lidar distance: " << leader_dist << " at stamp " << stamp;
  
  if (config_->publish){
    FloatMsg dist_msg;
    dist_msg.data = leader_dist;
    distance_pub_->publish(dist_msg);
  }
}

}  // namespace lidar
}  // namespace vtr