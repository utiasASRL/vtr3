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
 * \file mcransac_module.cpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/preprocessing/mcransac_module.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "vtr_radar/utils/utils.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

auto MCRANSACModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->tolerance = node->declare_parameter<float>(param_prefix + ".tolerance", config->tolerance);
  config->inlier_ratio = node->declare_parameter<float>(param_prefix + ".inlier_ratio", config->inlier_ratio);
  config->iterations = node->declare_parameter<float>(param_prefix + ".iterations", config->iterations);
  config->gn_iterations = node->declare_parameter<int>(param_prefix + ".gn_iterations", config->gn_iterations);
  config->epsilon_converge = node->declare_parameter<int>(param_prefix + ".epsilon_converge", config->epsilon_converge);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void MCRANSACModule::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("filtered_point_cloud", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Get input point cloud
  auto point_cloud = qdata.preprocessed_point_cloud.ptr();
  const auto &cartesian = *qdata.cartesian.emplace();

  if (point_cloud->size() == 0) {
    std::string err{"Empty point cloud."};
    CLOG(ERROR, "radar.mcransac") << err;
    throw std::runtime_error{err};
  }

  CLOG(DEBUG, "radar.mcransac")
      << "raw point cloud size: " << point_cloud->size();

  // Compute (ORB | RSD) descriptors and then match them here.
  // TODO: need to cache the temporal previous cartesian radar image
  // and pointcloud (post filtering1)
  cv::Ptr<cv::ORB> detector = cv::ORB::create();
  detector->setPatchSize(patch_size);
  detector->setEdgeThreshold(patch_size);
  cv::Mat desc1, desc2;
  // TODO: convert pointclouds to KeyPoints for OpenCV (shallow copy if possible)
  std::vector<cv::KeyPoint> kp1, kp2;
  detector->compute(cartesian_prev, kp1, desc1);
  detector->compute(cartesian, kp2, desc2);
  // Match keypoint descriptors
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher->knnMatch(desc1, desc2, knn_matches, 2);
  // Filter matches using nearest neighbor distance ratio (Lowe, Szeliski)
  std::vector<cv::DMatch> good_matches;
  for (uint j = 0; j < knn_matches.size(); ++j) {
      if (!knn_matches[j].size())
          continue;
      if (knn_matches[j][0].distance < nndr * knn_matches[j][1].distance) {
          good_matches.push_back(knn_matches[j][0]);
      }
  }

  // use the correspondences from matching and remove
  // matches which fail NNDR test
  // filter and rearrange pointclouds based on matching
  // use the filtered, aligned pointclouds in mcransac
  // overwrite processed_point_cloud with the inliers of mcransac
  // initialize ICP with motion computed by mcransac
  MCRansac mcransac(threshold, inlier_ratio, iterations, max_gn_iterations, epsilon_converge, 2);
  Eigen::VectorXd w_2_1;  // T_1_2 = vec2tran(delta_t * w_2_1)
  std::vector<int> best_inliers;
  mcransac.run(pc1, pc2, w_2_1, best_inliers);

  auto filtered_point_cloud =
      std::make_shared<pcl::PointCloud<PointWithInfo>>(*point_cloud);

  /// Grid subsampling

  // Get subsampling of the frame in carthesian coordinates
  gridSubsamplingCentersV2(*filtered_point_cloud, config_->frame_voxel_size);

  CLOG(DEBUG, "radar.mcransac")
      << "grid subsampled point cloud size: " << filtered_point_cloud->size();

  /// Compute normals

  // Define the polar neighbors radius in the scaled polar coordinates
  float radius = config_->window_size * config_->azimuth_res;

  // Extracts normal vectors of sampled points
  auto norm_scores = extractNormal(point_cloud, filtered_point_cloud, radius,
                                   config_->rho_scale, config_->num_threads);

  /// Filtering based on normal scores (linearity)

  // Remove points with a low normal score
  auto sorted_norm_scores = norm_scores;
  std::sort(sorted_norm_scores.begin(), sorted_norm_scores.end());
  float min_score = sorted_norm_scores[std::max(
      0, (int)sorted_norm_scores.size() - config_->num_sample_linearity)];
  min_score = std::max(config_->min_linearity_score, min_score);
  if (min_score >= 0) {
    std::vector<int> indices;
    indices.reserve(filtered_point_cloud->size());
    int i = 0;
    for (const auto &point : *filtered_point_cloud) {
      if (point.normal_score >= min_score) indices.emplace_back(i);
      i++;
    }
    *filtered_point_cloud =
        pcl::PointCloud<PointWithInfo>(*filtered_point_cloud, indices);
  }

  CLOG(DEBUG, "lidar.mcransac")
      << "linearity sampled point size: " << filtered_point_cloud->size();

  CLOG(DEBUG, "radar.mcransac")
      << "final subsampled point size: " << filtered_point_cloud->size();

  if (config_->visualize) {
    auto point_cloud_tmp = *filtered_point_cloud;
    const auto ref_time = (double)(*qdata.stamp / 1000) / 1e6;
    std::for_each(point_cloud_tmp.begin(), point_cloud_tmp.end(),
                  [&](PointWithInfo &point) { point.time -= ref_time; });
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(point_cloud_tmp, *pc2_msg);
    pc2_msg->header.frame_id = "radar";
    pc2_msg->header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(*pc2_msg);    
  }

  /// Output
  qdata.preprocessed_point_cloud = filtered_point_cloud;
}

}  // namespace radar
}  // namespace vtr