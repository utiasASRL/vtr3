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
#include "opencv2/opencv.hpp"
#include "vtr_radar/modules/preprocessing/mcransac_module.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "vtr_radar/utils/utils.hpp"
#include "vtr_radar/features/normal.hpp"
#include "vtr_radar/filters/grid_subsampling.hpp"
#include "vtr_radar/utils.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

// Converts points in radar frame to 2d keypoint locations in BEV (u,v) wrt TL
// Filters out points that lie outside the square BEV image.
void convert_to_bev(pcl::PointCloud<PointWithInfo> &pc,
                    const float cart_resolution,
                    const int cart_pixel_width,
                    const int patch_size,
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

void McransacModule::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    filtered_pub_ = qdata.node->create_publisher<PointCloudMsg>("filtered_point_cloud", 5);
    // clang-format onnv
    publisher_initialized_ = true;
  }

  // Input
  const auto pc2orig = qdata.preprocessed_point_cloud.ptr();
  const auto pc1orig = qdata.prev_prep_pc.ptr();
  const auto &cartesian = *qdata.cartesian.emplace();
  const int cart_pixel_width = cartesian.cols;
  const auto &cartesian_prev = *qdata.cartesian_prev.emplace();
  const auto &cart_resolution = *qdata.cart_resolution.emplace();
  const auto &t0 = *qdata.t0.emplace();
  const auto &t0_prev = *qdata.t0_prev.emplace();
  
  // this module requires a previous frame to run, skip for frame 0
  if (cartesian.rows != cartesian_prev.rows || pc1orig->size() == 0)
    return;

  if (pc2orig->size() == 0) {
    std::string err{"Empty point cloud."};
    CLOG(ERROR, "radar.mcransac") << err;
    throw std::runtime_error{err};
  }

  CLOG(DEBUG, "radar.mcransac")
      << "initial cloud size: " << pc2orig->size();

  auto pc2ptr =
      std::make_shared<pcl::PointCloud<PointWithInfo>>(*pc2orig);
  auto pc1ptr =
      std::make_shared<pcl::PointCloud<PointWithInfo>>(*pc1orig);

  // Compute (ORB | RSD) descriptors and then match them here.
  // and pointcloud (post filtering1)
  cv::Ptr<cv::ORB> detector = cv::ORB::create();
  detector->setPatchSize(config_->patch_size);
  detector->setEdgeThreshold(config_->patch_size);
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
  cv::Mat desc1, desc2;
  // Convert pointclouds to KeyPoints for OpenCV
  std::vector<cv::KeyPoint> kp1, kp2;
  convert_to_bev(*pc1ptr, cart_resolution, cart_pixel_width, config_->patch_size, kp1);
  convert_to_bev(*pc2ptr, cart_resolution, cart_pixel_width, config_->patch_size, kp2);

  CLOG(DEBUG, "radar.mcransac")
      << "BEV cloud size: " << pc2ptr->size();

  detector->compute(cartesian_prev, kp1, desc1);
  detector->compute(cartesian, kp2, desc2);
  // Match keypoint descriptors
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher->knnMatch(desc1, desc2, knn_matches, 2);
  // Filter matches using nearest neighbor distance ratio (Lowe, Szeliski)
  std::vector<cv::DMatch> good_matches;
  good_matches.reserve(knn_matches.size());
  for (size_t j = 0; j < knn_matches.size(); ++j) {
      if (!knn_matches[j].size())
          continue;
      if (knn_matches[j][0].distance < config_->nndr * knn_matches[j][1].distance) {
          good_matches.emplace_back(knn_matches[j][0]);
      }
  }
  std::vector<int> indices1(good_matches.size());
  std::vector<int> indices2(good_matches.size());
  for (size_t j = 0; j < good_matches.size(); ++j) {
    indices1[j] = good_matches[j].queryIdx;
    indices2[j] = good_matches[j].trainIdx;
  }
  // Downsample and re-order points based on matching and NNDR
  *pc1ptr = pcl::PointCloud<PointWithInfo>(*pc1ptr, indices1);
  *pc2ptr = pcl::PointCloud<PointWithInfo>(*pc2ptr, indices2);

  CLOG(DEBUG, "radar.mcransac")
      << "matching + NNDR point size: " << pc2ptr->size();

  // overwrite processed_point_cloud with the inliers of mcransac
  // initialize ICP with motion computed by mcransac
  // template <class PointWithInfo>
  auto mcransac = std::make_unique<MCRansac<PointWithInfo>>(config_->tolerance, config_->inlier_ratio,
    config_->iterations, config_->gn_iterations,
    config_->epsilon_converge, 2);
  Eigen::VectorXd w_2_1;  // T_1_2 = vec2tran(delta_t * w_2_1)
  std::vector<int> best_inliers;
  mcransac->run(*pc1ptr, *pc2ptr, w_2_1, best_inliers);

  CLOG(DEBUG, "radar.mcransac")
      << "MC-RANSAC inlier point size: " << best_inliers.size();

  if (config_->filter_pc) {
    *pc2ptr = pcl::PointCloud<PointWithInfo>(*pc2ptr, best_inliers);
    qdata.preprocessed_point_cloud = pc2ptr;
  }

  CLOG(DEBUG, "radar.mcransac")
      << "final subsampled point size: " << pc2ptr->size();

  if (config_->init_icp) {
    *qdata.w_m_r_in_r_odo.emplace() = w_2_1;
    double delta_t = t0 - t0_prev;
    *qdata.T_r_pm_odo.emplace() = lgmath::se3::vec2tran(delta_t * w_2_1);
  }

  if (config_->visualize) {
    auto point_cloud_tmp = *pc2ptr;
    const auto ref_time = (double)(*qdata.stamp / 1000) / 1e6;
    std::for_each(point_cloud_tmp.begin(), point_cloud_tmp.end(),
                  [&](PointWithInfo &point) { point.time -= ref_time; });
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(point_cloud_tmp, *pc2_msg);
    pc2_msg->header.frame_id = "radar";
    pc2_msg->header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(*pc2_msg);    
  }
}

}  // namespace radar
}  // namespace vtr