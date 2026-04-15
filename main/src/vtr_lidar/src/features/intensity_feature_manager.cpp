/**
 * \file intensity_feature_manager.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */


 #include "vtr_lidar/features/intensity_feature_manager.hpp"
#include "vtr_logging/logging.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>

namespace vtr {
namespace lidar {

IntensityFeatureManager::IntensityFeatureManager(
    const IntensityFeatureManagerConfig::ConstPtr& config)
    : config_(config) {
  // Create ORB detector
  orb_detector_ = cv::ORB::create(
      config_->max_features,
      config_->scale_factor,
      config_->nlevels,
      31,                        // edgeThreshold
      0,                         // firstLevel
      2,                         // WTA_K
      cv::ORB::HARRIS_SCORE,
      31,                        // patchSize
      config_->fast_threshold);

  // Create BFMatcher for ORB (Hamming distance)
  bf_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);
}

void IntensityFeatureManager::generateIntensityImage(
    const pcl::PointCloud<PointWithInfo>& cloud,
    cv::Mat& intensity_image,
    cv::Mat& range_image,
    cv::Mat& pixel_to_point_index) {

  const int H = config_->image_height;
  const int W = config_->image_width;
  const float vfov_min = config_->vfov_min_deg * M_PI / 180.0f;
  const float vfov_max = config_->vfov_max_deg * M_PI / 180.0f;

  intensity_image = cv::Mat::zeros(H, W, CV_32F);
  range_image = cv::Mat::zeros(H, W, CV_32F);
  pixel_to_point_index = cv::Mat::ones(H, W, CV_32S) * -1;

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& pt = cloud[i];
    const float x = pt.x, y = pt.y, z = pt.z;
    const float range = std::sqrt(x * x + y * y + z * z);

    if (range < config_->min_range || range > config_->max_range) continue;

    // Azimuth -> column
    const float azimuth = std::atan2(y, x);  // [-pi, pi]
    int col = static_cast<int>((azimuth + M_PI) / (2.0 * M_PI) * W);
    col = std::clamp(col, 0, W - 1);

    // Elevation -> row
    const float elevation = std::asin(z / range);
    int row = static_cast<int>((elevation - vfov_min) / (vfov_max - vfov_min) * H);
    row = std::clamp(row, 0, H - 1);
    row = H - 1 - row;  // flip: top of image = highest beam

    // Keep closest point per pixel
    if (pixel_to_point_index.at<int>(row, col) == -1 ||
        range < range_image.at<float>(row, col)) {
      intensity_image.at<float>(row, col) = pt.intensity;
      range_image.at<float>(row, col) = range;
      pixel_to_point_index.at<int>(row, col) = static_cast<int>(i);
    }
  }

  // Normalize to 0-255
  cv::Mat normalized;
  cv::normalize(intensity_image, normalized, 0, 255, cv::NORM_MINMAX);
  normalized.convertTo(intensity_image, CV_8U);
}

IntensityFeatures IntensityFeatureManager::detectFeatures(
    const cv::Mat& intensity_image,
    const cv::Mat& pixel_to_point_index,
    const pcl::PointCloud<PointWithInfo>& cloud,
    const cv::Mat& mask) {

  IntensityFeatures features;
  features.image_width = intensity_image.cols;
  features.image_height = intensity_image.rows;

  // Detect ORB keypoints and compute descriptors
  std::vector<cv::KeyPoint> all_keypoints;
  cv::Mat all_descriptors;
  orb_detector_->detectAndCompute(intensity_image,
                                   mask.empty() ? cv::noArray() : mask,
                                   all_keypoints, all_descriptors);

  // Filter: keep only keypoints with valid 3D back-projection
  std::vector<cv::KeyPoint> valid_keypoints;
  cv::Mat valid_descriptors;
  std::vector<Eigen::Vector3d> valid_points;
  std::vector<int> valid_cloud_indices;  // indices into cloud for timestamps

  for (size_t i = 0; i < all_keypoints.size(); ++i) {
    const auto& kp = all_keypoints[i];
    const int row = static_cast<int>(std::round(kp.pt.y));
    const int col = static_cast<int>(std::round(kp.pt.x));

    if (row < 0 || row >= pixel_to_point_index.rows ||
        col < 0 || col >= pixel_to_point_index.cols)
      continue;

    const int pt_idx = pixel_to_point_index.at<int>(row, col);
    if (pt_idx < 0 || pt_idx >= static_cast<int>(cloud.size())) continue;

    const auto& pt = cloud[pt_idx];
    const float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (range < config_->min_range) continue;

    valid_keypoints.push_back(kp);
    valid_descriptors.push_back(all_descriptors.row(static_cast<int>(i)));
    valid_points.emplace_back(pt.x, pt.y, pt.z);
    valid_cloud_indices.push_back(pt_idx);
  }

  // Package results
  features.keypoints = std::move(valid_keypoints);
  features.descriptors = valid_descriptors.clone();
  features.points_3d.resize(3, valid_points.size());
  features.timestamps.resize(valid_points.size());
  for (size_t i = 0; i < valid_points.size(); ++i) {
    features.points_3d.col(i) = valid_points[i];
    features.timestamps[i] = cloud[valid_cloud_indices[i]].timestamp;
  }

  return features;
}

std::vector<std::pair<int, int>> IntensityFeatureManager::matchFeatures(
    const IntensityFeatures& query_features,
    const IntensityFeatures& train_features) {

  std::vector<std::pair<int, int>> matches;

  if (query_features.empty() || train_features.empty()) return matches;

  // KNN match with k=2 for ratio test
  std::vector<std::vector<cv::DMatch>> knn_matches;
  bf_matcher_->knnMatch(query_features.descriptors,
                         train_features.descriptors,
                         knn_matches, 2);

  // Apply ratio test and distance threshold
  for (const auto& match_pair : knn_matches) {
    if (match_pair.size() < 2) continue;

    const auto& best = match_pair[0];
    const auto& second = match_pair[1];

    if (best.distance < config_->match_distance_threshold &&
        best.distance < config_->match_ratio_threshold * second.distance) {
      matches.emplace_back(best.queryIdx, best.trainIdx);
    }
  }

  return matches;
}

}  // namespace lidar
}  // namespace vtr