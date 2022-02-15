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

#include "opencv2/opencv.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include "vtr_radar/features/normal.hpp"
#include "vtr_radar/filters/grid_subsampling.hpp"
#include "vtr_radar/mcransac/mcransac.hpp"
#include "vtr_radar/utils.hpp"
#include "vtr_radar/utils/utils.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

namespace {

// Converts points in radar frame to 2d keypoint locations in BEV (u,v) wrt TL
// Filters out points that lie outside the square BEV image.
void convert_to_bev(const float cart_resolution, const int cart_pixel_width,
                    const int patch_size, pcl::PointCloud<PointWithInfo> &pc,
                    std::vector<cv::KeyPoint> &kp) {
  float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
  if (cart_pixel_width % 2 == 0)
    cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;
  kp.clear();
  kp.reserve(pc.size());
  std::vector<int> inliers;
  inliers.reserve(pc.size());
  for (size_t i = 0; i < pc.size(); ++i) {
    const auto p = pc.at(i).getVector3fMap().cast<double>();
    const double u = (cart_min_range + p(1)) / cart_resolution;
    const double v = (cart_min_range - p(0)) / cart_resolution;
    if (0 < u - patch_size && u + patch_size < cart_pixel_width &&
        0 < v - patch_size && v + patch_size < cart_pixel_width) {
      kp.emplace_back(cv::KeyPoint(u, v, patch_size));
      inliers.push_back(i);
    }
  }
  pc = pcl::PointCloud<PointWithInfo>(pc, inliers);
}

}  // namespace

auto McransacModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                     const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->tolerance = node->declare_parameter<float>(param_prefix + ".tolerance", config->tolerance);
  config->inlier_ratio = node->declare_parameter<float>(param_prefix + ".inlier_ratio", config->inlier_ratio);
  config->iterations = node->declare_parameter<float>(param_prefix + ".iterations", config->iterations);
  config->gn_iterations = node->declare_parameter<int>(param_prefix + ".gn_iterations", config->gn_iterations);
  config->epsilon_converge = node->declare_parameter<int>(param_prefix + ".epsilon_converge", config->epsilon_converge);
  config->patch_size = node->declare_parameter<int>(param_prefix + ".patch_size", config->patch_size);
  config->nndr = node->declare_parameter<int>(param_prefix + ".nndr", config->nndr);
  config->filter_pc = node->declare_parameter<bool>(param_prefix + ".filter_pc", config->filter_pc);
  config->init_icp = node->declare_parameter<bool>(param_prefix + ".init_icp", config->init_icp);
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void McransacModule::run_(QueryCache &qdata0, OutputCache &, const Graph::Ptr &,
                          const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("ransac_filtered_point_cloud", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // this module requires a previous frame to run, skip for frame 0
  if (!qdata.cartesian_odo) {
    qdata.cartesian_odo = qdata.cartesian.ptr();
    qdata.point_cloud_odo = qdata.preprocessed_point_cloud.ptr();
    CLOG(INFO, "radar.mcransac") << "First frame, simply return.";
    return;
  }

  // Input
  const auto &query_cartesian = *qdata.cartesian;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &ref_cartesian = *qdata.cartesian_odo;
  const auto &ref_points = *qdata.point_cloud_odo;
  const auto &cart_resolution = *qdata.cart_resolution;

  if (query_points.size() == 0) {
    std::string err{"Empty point cloud."};
    CLOG(ERROR, "radar.mcransac") << err;
    throw std::runtime_error{err};
  }

  CLOG(DEBUG, "radar.mcransac")
      << "initial cloud size: " << query_points.size();

  auto filtered_query_points = query_points;
  auto filtered_ref_points = ref_points;

  // Compute (ORB | RSD) descriptors and then match them here.
  // and pointcloud (post filtering1)
  cv::Ptr<cv::ORB> detector = cv::ORB::create();
  detector->setPatchSize(config_->patch_size);
  detector->setEdgeThreshold(config_->patch_size);
  // Convert pointclouds to KeyPoints for OpenCV
  std::vector<cv::KeyPoint> query_keypoints, ref_keypoints;
  convert_to_bev(cart_resolution, query_cartesian.cols, config_->patch_size,
                 filtered_query_points, query_keypoints);
  convert_to_bev(cart_resolution, query_cartesian.cols, config_->patch_size,
                 filtered_ref_points, ref_keypoints);
  CLOG(DEBUG, "radar.mcransac")
      << "BEV cloud size: " << filtered_query_points.size();

  cv::Mat query_descs, ref_descs;
  detector->compute(query_cartesian, query_keypoints, query_descs);
  detector->compute(ref_cartesian, ref_keypoints, ref_descs);

  // Match keypoint descriptors
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher->knnMatch(query_descs, ref_descs, knn_matches, 2);
  // Filter matches using nearest neighbor distance ratio (Lowe, Szeliski)
  std::vector<cv::DMatch> good_matches;
  good_matches.reserve(knn_matches.size());
  for (size_t j = 0; j < knn_matches.size(); ++j) {
    if (knn_matches[j].size() < 2) continue;
    if (knn_matches[j][0].distance <
        config_->nndr * knn_matches[j][1].distance) {
      good_matches.emplace_back(knn_matches[j][0]);
    }
  }

  std::vector<int> query_indices;
  query_indices.reserve(good_matches.size());
  std::vector<int> ref_indices;
  ref_indices.reserve(good_matches.size());
  for (const auto &match : good_matches) {
    query_indices.emplace_back(match.queryIdx);
    ref_indices.emplace_back(match.trainIdx);
  }

  // Downsample and re-order points based on matching and NNDR
  filtered_query_points =
      pcl::PointCloud<PointWithInfo>(filtered_query_points, query_indices);
  filtered_ref_points =
      pcl::PointCloud<PointWithInfo>(filtered_ref_points, ref_indices);

  CLOG(DEBUG, "radar.mcransac")
      << "matching + NNDR point size: " << filtered_query_points.size();

  // overwrite processed_point_cloud with the inliers of mcransac
  // initialize ICP with motion computed by mcransac
  // template <class PointWithInfo>
  auto mcransac = std::make_unique<MCRansac<PointWithInfo>>(
      config_->tolerance, config_->inlier_ratio, config_->iterations,
      config_->gn_iterations, config_->epsilon_converge, 2);
  // T_sensornew_sensorold = vec2tran(dt * w_m_s_in_s)
  Eigen::VectorXd w_m_s_in_s;
  std::vector<int> best_inliers;
  mcransac->run(filtered_query_points, filtered_ref_points, w_m_s_in_s,
                best_inliers);

  CLOG(DEBUG, "radar.mcransac")
      << "MC-RANSAC inlier point size: " << best_inliers.size();

  if (config_->filter_pc) {
    *qdata.preprocessed_point_cloud = pcl::PointCloud<PointWithInfo>(
        *qdata.preprocessed_point_cloud, best_inliers);
  }

  CLOG(DEBUG, "radar.mcransac") << "final filtered point size: "
                                << qdata.preprocessed_point_cloud->size();

  if (config_->init_icp) {
    /// \todo convert velocity vectory to robot frame!!!!! currently only
    /// correct if T_s_r is identity!!!
    *qdata.w_m_r_in_r_odo = w_m_s_in_s;
  }

  // store this frame's bev image and pointcloud
  qdata.cartesian_odo = qdata.cartesian.ptr();
  qdata.point_cloud_odo = qdata.preprocessed_point_cloud.ptr();

  if (config_->visualize) {
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(*qdata.preprocessed_point_cloud, *pc2_msg);
    pc2_msg->header.frame_id = "radar";
    pc2_msg->header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(*pc2_msg);
  }
}

}  // namespace radar
}  // namespace vtr