/**
 * \file intensity_feature_manager.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include "vtr_lidar/data_types/point.hpp"
#include "vtr_lidar/data_types/intensity_features.hpp"

#include "vtr_lidar/features/auto_exposure.hpp"

namespace vtr {
namespace lidar {

/// Configuration for the intensity feature manager
struct IntensityFeatureManagerConfig {
  PTR_TYPEDEFS(IntensityFeatureManagerConfig);

  // Image generation
  int image_width = 1024;
  int image_height = 128;
  float vfov_min_deg = -22.5f;
  float vfov_max_deg = 22.5f;

  // ORB detection
  int max_features = 500;
  int nlevels = 8;
  float scale_factor = 1.2f;
  int fast_threshold = 20;
  float min_range = 0.5f;
  float max_range = 100.0f;

  // Grid-based detection (spatial spread)
  int grid_rows = 2;          ///< vertical grid divisions
  int grid_cols = 8;          ///< horizontal grid divisions
  int max_per_cell = 100;     ///< max features retained per grid cell
  int edge_threshold = 4;     ///< ORB edgeThreshold (small for narrow cells)
  int patch_size = 15;        ///< ORB patchSize (small for narrow cells)

  // Matching
  float match_distance_threshold = 50.0f;  // Hamming distance
  float match_ratio_threshold = 0.8f;      // Lowe's ratio test
};

/// Manages ORB feature detection on lidar intensity images
/// and frame-to-frame matching
class IntensityFeatureManager {
 public:
  PTR_TYPEDEFS(IntensityFeatureManager);

  explicit IntensityFeatureManager(
      const IntensityFeatureManagerConfig::ConstPtr& config);

  /// Generate intensity image from point cloud
  /// Also populates range_image and pixel_to_point_index
  void generateIntensityImage(
      const pcl::PointCloud<PointWithInfo>& cloud,
      cv::Mat& intensity_image,
      cv::Mat& range_image,
      cv::Mat& pixel_to_point_index);

  /// Detect ORB features and back-project to 3D
  /// @param mask  CV_8UC1 mask (255=valid, 0=masked); empty = no masking
  IntensityFeatures detectFeatures(
      const cv::Mat& intensity_image,
      const cv::Mat& pixel_to_point_index,
      const pcl::PointCloud<PointWithInfo>& cloud,
      const cv::Mat& mask = cv::Mat());

  /// Match two sets of features (frame-to-frame or live-to-map)
  /// Returns pairs of (query_idx, train_idx)
  std::vector<std::pair<int, int>> matchFeatures(
      const IntensityFeatures& query_features,
      const IntensityFeatures& train_features);

 private:
  IntensityFeatureManagerConfig::ConstPtr config_;
  cv::Ptr<cv::ORB> orb_detector_;
  cv::Ptr<cv::BFMatcher> bf_matcher_;
  AutoExposure auto_exposure_;
};

}  // namespace lidar
}  // namespace vtr
