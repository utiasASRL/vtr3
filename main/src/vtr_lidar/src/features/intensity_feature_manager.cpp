/**
 * \file intensity_feature_manager.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */


 #include "vtr_lidar/features/intensity_feature_manager.hpp"
#include "vtr_logging/logging.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <omp.h>

namespace vtr {
namespace lidar {

IntensityFeatureManager::IntensityFeatureManager(
    const IntensityFeatureManagerConfig::ConstPtr& config)
    : config_(config) {
  // The main orb_detector_ is kept for matchFeatures (BFMatcher).
  // For grid detection, each thread creates its own ORB instance.
  orb_detector_ = cv::ORB::create(
      std::max(config_->max_per_cell * 2, 100),
      config_->scale_factor,
      config_->nlevels,
      config_->edge_threshold,
      0,                         // firstLevel
      2,                         // WTA_K
      cv::ORB::HARRIS_SCORE,
      config_->patch_size,
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

  const int rows = intensity_image.rows;
  const int cols = intensity_image.cols;
  const int grid_rows = config_->grid_rows;
  const int grid_cols = config_->grid_cols;
  const int num_cells = grid_rows * grid_cols;
  const int cell_h = rows / grid_rows;
  const int cell_w = cols / grid_cols;

  // Fair per-cell budget: equal share of global cap, clamped to max_per_cell
  const int cell_budget = std::min(config_->max_per_cell,
                                   (config_->max_features + num_cells - 1) / num_cells);

  // Per-cell results (one slot per cell, no data races)
  std::vector<std::vector<cv::KeyPoint>> cell_kps_vec(num_cells);
  std::vector<cv::Mat> cell_desc_vec(num_cells);

  #pragma omp parallel
  {
    // Thread-local ORB detector (cv::ORB is NOT thread-safe)
    auto orb_local = cv::ORB::create(
        std::max(config_->max_per_cell * 2, 100),
        config_->scale_factor,
        config_->nlevels,
        config_->edge_threshold,
        0, 2, cv::ORB::HARRIS_SCORE,
        config_->patch_size,
        config_->fast_threshold);

    #pragma omp for schedule(dynamic)
    for (int ci = 0; ci < num_cells; ++ci) {
      const int gr = ci / grid_cols;
      const int gc = ci % grid_cols;

      // Cell region (last row/col absorbs remainder pixels)
      const int x0 = gc * cell_w;
      const int y0 = gr * cell_h;
      const int x1 = (gc == grid_cols - 1) ? cols : x0 + cell_w;
      const int y1 = (gr == grid_rows - 1) ? rows : y0 + cell_h;
      cv::Rect cell_rect(x0, y0, x1 - x0, y1 - y0);

      cv::Mat cell_img = intensity_image(cell_rect);
      cv::Mat cell_mask = mask.empty() ? cv::Mat() : mask(cell_rect);

      // Detect in this cell
      std::vector<cv::KeyPoint> cell_kps;
      cv::Mat cell_desc;
      orb_local->detectAndCompute(cell_img, cell_mask, cell_kps, cell_desc);

      // Sort by response (strongest first) and keep top cell_budget
      if (static_cast<int>(cell_kps.size()) > cell_budget) {
        std::vector<std::pair<float, int>> scored;
        scored.reserve(cell_kps.size());
        for (int i = 0; i < static_cast<int>(cell_kps.size()); ++i)
          scored.emplace_back(cell_kps[i].response, i);
        std::partial_sort(scored.begin(),
                          scored.begin() + cell_budget,
                          scored.end(),
                          [](const auto& a, const auto& b) {
                            return a.first > b.first;
                          });

        std::vector<cv::KeyPoint> top_kps;
        cv::Mat top_desc;
        top_kps.reserve(cell_budget);
        for (int k = 0; k < cell_budget; ++k) {
          top_kps.push_back(cell_kps[scored[k].second]);
          if (!cell_desc.empty())
            top_desc.push_back(cell_desc.row(scored[k].second));
        }
        cell_kps = std::move(top_kps);
        cell_desc = top_desc;
      }

      // Shift keypoint coordinates from cell-local to full-image
      for (auto& kp : cell_kps) {
        kp.pt.x += static_cast<float>(x0);
        kp.pt.y += static_cast<float>(y0);
      }

      cell_kps_vec[ci] = std::move(cell_kps);
      cell_desc_vec[ci] = cell_desc;
    }
  }  // end #pragma omp parallel

  // Merge per-cell results
  std::vector<cv::KeyPoint> all_keypoints;
  cv::Mat all_descriptors;
  for (int ci = 0; ci < num_cells; ++ci) {
    all_keypoints.insert(all_keypoints.end(),
                         cell_kps_vec[ci].begin(), cell_kps_vec[ci].end());
    if (!cell_desc_vec[ci].empty()) {
      if (all_descriptors.empty())
        all_descriptors = cell_desc_vec[ci];
      else
        cv::vconcat(all_descriptors, cell_desc_vec[ci], all_descriptors);
    }
  }

  // Global cap: keep top max_features by response
  if (static_cast<int>(all_keypoints.size()) > config_->max_features) {
    std::vector<std::pair<float, int>> scored;
    scored.reserve(all_keypoints.size());
    for (int i = 0; i < static_cast<int>(all_keypoints.size()); ++i)
      scored.emplace_back(all_keypoints[i].response, i);
    std::partial_sort(scored.begin(),
                      scored.begin() + config_->max_features,
                      scored.end(),
                      [](const auto& a, const auto& b) {
                        return a.first > b.first;
                      });

    std::vector<cv::KeyPoint> kept_kps;
    cv::Mat kept_desc;
    kept_kps.reserve(config_->max_features);
    for (int k = 0; k < config_->max_features; ++k) {
      kept_kps.push_back(all_keypoints[scored[k].second]);
      if (!all_descriptors.empty())
        kept_desc.push_back(all_descriptors.row(scored[k].second));
    }
    all_keypoints = std::move(kept_kps);
    all_descriptors = kept_desc;
  }

  // Filter: keep only keypoints with valid 3D back-projection
  std::vector<cv::KeyPoint> valid_keypoints;
  cv::Mat valid_descriptors;
  std::vector<Eigen::Vector3d> valid_points;
  std::vector<int> valid_cloud_indices;

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