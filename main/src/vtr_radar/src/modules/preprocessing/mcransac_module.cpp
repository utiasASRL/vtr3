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
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"

#include "vtr_radar/features/normal.hpp"
#include "vtr_radar/filters/grid_subsampling.hpp"
#include "vtr_radar/mcransac/mcransac.hpp"
#include "vtr_radar/utils.hpp"
#include "vtr_radar/utils/utils.hpp"

namespace vtr {
namespace radar {

  // Binary Annular Statistics Descriptor (BASD) (Rapp et al., 2016)
// descriptors are binary and can be associated quickly with
// hamming distance
class BASD {
 public:
  BASD() = default;
  BASD(int nbins, int bin_size) : nbins_(nbins), bin_size_(bin_size) {}
  void compute(const cv::Mat& cartesian,
               const std::vector<cv::KeyPoint> &keypoints,
               cv::Mat &descriptors);

 private:
  int nbins_ = 16;
  int bin_size_ = 1;
};

void BASD::compute(const cv::Mat& cartesian,
                   const std::vector<cv::KeyPoint> &keypoints,
                   cv::Mat &descriptors) {
  const int max_range = nbins_ - 1;
  const float max_range_sq = max_range * max_range;
  const int dim = std::ceil(nbins_ * (nbins_ + 1) / 8.0);  // dimension of descriptor in bytes
  const int cols = cartesian.cols;
  const int rows = cartesian.rows;
  descriptors = cv::Mat::zeros(keypoints.size(), dim, CV_8UC1);
  for (size_t kp_idx = 0; kp_idx < keypoints.size(); ++kp_idx) {
    const auto &kp = keypoints[kp_idx];
    std::vector<std::vector<float>> bins(nbins_);
    const int minrow = std::max(int(std::ceil(kp.pt.y - max_range)), 0);
    const int maxrow = std::min(int(std::floor(kp.pt.y + max_range)), rows);
    const int mincol = std::max(int(std::ceil(kp.pt.x - max_range)), 0);
    const int maxcol = std::min(int(std::floor(kp.pt.x + max_range)), cols);
    for (int i = minrow; i < maxrow; ++i) {
      for (int j = mincol; j < maxcol; ++j) {
        float r = pow(i - kp.pt.y, 2) + pow(j - kp.pt.x, 2);
        if (r > max_range_sq)
          continue;
        r = sqrt(r);
        int bin = std::floor(r / bin_size_);
        bins[bin].push_back(cartesian.at<float>(i, j));
      }
    }
    // compute statistics for each bin
    std::vector<float> means(nbins_, 0);
    std::vector<float> stds(nbins_, 0);
    for (int i = 0; i < nbins_; ++i) {
      float mean = 0;
      if (bins[i].size() == 0)
        continue;
      for (size_t j = 0; j < bins[i].size(); ++j) {
        mean += bins[i][j];
      }
      mean /= bins[i].size();
      means[i] = mean;
      float std = 0;
      for (size_t j = 0; j < bins[i].size(); ++j) {
        std += pow(bins[i][j] - mean, 2);
      }
      std /= bins[i].size();
      std = sqrt(std);
      stds[i] = std;
    }
    // compare statistics between rings to create binary descriptor
    int k = 0;
    for (int i = 0; i < nbins_; ++i) {
      for (int j = 0; j < nbins_; ++j) {
        if (i == j)
          continue;
        if (means[i] > means[j]) {
          int byte = std::floor(k / 8.0);
          int bit = k % 8;
          descriptors.at<uchar>(kp_idx, byte) |= (1 << bit);
        }
        k++;
        if (stds[i] > stds[j]) {
          int byte = std::floor(k / 8.0);
          int bit = k % 8;
          descriptors.at<uchar>(kp_idx, byte) |= (1 << bit);
        }
        k++;
      }
    }
  }
}

using namespace tactic;

namespace {

// Converts points in radar frame to 2d keypoint locations in BEV (u,v) wrt TL
// Filters out points that lie outside the square BEV image.
void convertToBEV(const float cart_resolution, const int cart_pixel_width,
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
  config->iterations = node->declare_parameter<int>(param_prefix + ".iterations", config->iterations);
  config->gn_iterations = node->declare_parameter<int>(param_prefix + ".gn_iterations", config->gn_iterations);
  config->epsilon_converge = node->declare_parameter<double>(param_prefix + ".epsilon_converge", config->epsilon_converge);
  config->descriptor = node->declare_parameter<std::string>(param_prefix + ".descriptor", config->descriptor);
  config->patch_size = node->declare_parameter<int>(param_prefix + ".patch_size", config->patch_size);
  config->nndr = node->declare_parameter<float>(param_prefix + ".nndr", config->nndr);
  config->filter_pc = node->declare_parameter<bool>(param_prefix + ".filter_pc", config->filter_pc);
  config->init_icp = node->declare_parameter<bool>(param_prefix + ".init_icp", config->init_icp);
  config->nbins = node->declare_parameter<int>(param_prefix + ".nbins", config->nbins);
  config->bin_size = node->declare_parameter<int>(param_prefix + ".bin_size", config->bin_size);
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
    image_pub_ = qdata.node->create_publisher<ImageMsg>("matching", 5);
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
  const auto query_points_backup = std::make_shared<pcl::PointCloud<PointWithInfo>>(query_points);
  const auto &T_s_r_ = *qdata.T_s_r;
  const Eigen::Matrix4d T_s_r = T_s_r_.matrix().cast<double>();
  const auto &ref_cartesian = *qdata.cartesian_odo;
  const auto &ref_points = *qdata.point_cloud_odo;
  const auto &cart_resolution = *qdata.cart_resolution;

  if (query_points.size() == 0) {
    std::string err{"Empty point cloud."};
    CLOG(ERROR, "radar.mcransac") << err;
    throw std::runtime_error{err};
  }

  CLOG(DEBUG, "radar.mcransac")
      << "initial cloud sizes: " << query_points.size() << " " << ref_points.size();

  // ref == 1, query == 2
  auto filtered_query_points = query_points;
  auto filtered_ref_points = ref_points;

  // Convert pointclouds to KeyPoints for OpenCV
  std::vector<cv::KeyPoint> query_keypoints, ref_keypoints;
  convertToBEV(cart_resolution, query_cartesian.cols, config_->patch_size,
               filtered_query_points, query_keypoints);
  convertToBEV(cart_resolution, query_cartesian.cols, config_->patch_size,
               filtered_ref_points, ref_keypoints);
  CLOG(DEBUG, "radar.mcransac")
      << "BEV cloud sizes: " << filtered_query_points.size() << " " << filtered_ref_points.size();

  /// \todo The following code compiles but have not been tested yet.
  /// remove the if false macro to test them using the odometry test script.
  cv::Mat query_descs, ref_descs;
  if (config_->descriptor == "orb") {
    // Compute (ORB | RSD) descriptors
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->setPatchSize(config_->patch_size);
    detector->setEdgeThreshold(config_->patch_size);
    detector->compute(query_cartesian, query_keypoints, query_descs);
    detector->compute(ref_cartesian, ref_keypoints, ref_descs);
  } else if (config_->descriptor == "basd") {
    auto detector = BASD(config_->nbins, config_->bin_size);
    detector.compute(query_cartesian, query_keypoints, query_descs);
    detector.compute(ref_cartesian, ref_keypoints, ref_descs);
  } else {
    CLOG(ERROR, "radar.mcransac")
        << "Unknown descriptor: " << config_->descriptor;
    throw std::runtime_error("Unknown descriptor: " + config_->descriptor);
  }

  // Match keypoint descriptors
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher->knnMatch(query_descs, ref_descs, knn_matches, 2);
  // Filter matches using nearest neighbor distance ratio (Lowe, Szeliski)
  // and ensure matches are one-to-one
  std::vector<cv::DMatch> matches;
  matches.reserve(knn_matches.size());
  for (size_t j = 0; j < knn_matches.size(); ++j) {
    if (knn_matches[j].size() == 0) continue;
    if (knn_matches[j].size() >= 2) {
      if (knn_matches[j][0].distance >
        config_->nndr * knn_matches[j][1].distance)
        continue;
    }
    auto it = find_if(matches.begin(), matches.end(),
    [&](const cv::DMatch & val) -> bool {
      return val.trainIdx == knn_matches[j][0].trainIdx ||
        val.queryIdx == knn_matches[j][0].queryIdx;
    });
    // trainIdx
    if (it != matches.end()) {
      auto idx = it - matches.begin();
      if (knn_matches[j][0].distance < matches[idx].distance)
        matches[idx] = knn_matches[j][0];
    } else {
      matches.emplace_back(knn_matches[j][0]);
    }
  }

  std::vector<int> query_indices, ref_indices;
  query_indices.reserve(matches.size());
  ref_indices.reserve(matches.size());
  for (size_t j = 0; j < matches.size(); ++j) {
    query_indices.emplace_back(matches[j].queryIdx);
    ref_indices.emplace_back(matches[j].trainIdx);
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
  // T_sensornew_sensorold = vec2tran(dt * w_pm_s_in_s)
  Eigen::VectorXd w_pm_s_in_s;
  std::vector<int> best_inliers;
  mcransac->run(filtered_ref_points, filtered_query_points, w_pm_s_in_s,
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
    // Convert velocity vector from sensor frame to robot frame
    const Eigen::Matrix3d C_s_r = T_s_r.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d r_r_s_in_s = T_s_r.block<3, 1>(0, 3).cast<double>();
    const Eigen::Matrix3d C_r_s = C_s_r.transpose();
    const Eigen::Vector3d vel = w_pm_s_in_s.block<3, 1>(0, 0).cast<double>();
    const Eigen::Vector3d ang = w_pm_s_in_s.block<3, 1>(3, 0).cast<double>();
    w_pm_s_in_s.block<3, 1>(0, 0) = C_r_s * vel - C_r_s * lgmath::so3::hat(r_r_s_in_s) * ang;
    w_pm_s_in_s.block<3, 1>(3, 0) = C_r_s * ang;
    w_pm_s_in_s *= -1;
    *qdata.w_pm_r_in_r_odo = w_pm_s_in_s;
  }

  CLOG(DEBUG, "radar.mcransac")
      << "MC-RANSAC motion estimate: " << w_pm_s_in_s.transpose();

  cv::Mat img_matches;
  if (config_->visualize) {
    cv::drawMatches(query_cartesian,
      query_keypoints, ref_cartesian, ref_keypoints, matches, img_matches, cv::Scalar::all(-1),
      cv::Scalar::all(-1), std::vector<char>(),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  }

  // store this frame's bev image and pointcloud
  qdata.cartesian_odo = qdata.cartesian.ptr();
  qdata.point_cloud_odo = query_points_backup;

  if (config_->visualize) {
    // publish filtered pointcloud
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(*qdata.preprocessed_point_cloud, *pc2_msg);
    pc2_msg->header.frame_id = "radar";
    pc2_msg->header.stamp = rclcpp::Time(*qdata.stamp);
    filtered_pub_->publish(*pc2_msg);
    // publish matching image
    cv_bridge::CvImage image_br;
    image_br.header.frame_id = "radar";
    image_br.encoding = "bgr8";
    // img_matches.convertTo(image_br.image, CV_8UC3, 255);
    image_br.image = img_matches;
    image_pub_->publish(*image_br.toImageMsg());
  }
}

}  // namespace radar
}  // namespace vtr