
/**
 * \file intensity_features.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Dense>

#include "vtr_common/utils/macros.hpp"
#include "vtr_lidar_msgs/msg/intensity_features.hpp"


namespace vtr {
namespace lidar {

class IntensityFeatures {
 public:
  PTR_TYPEDEFS(IntensityFeatures);

  using IntensityFeaturesMsg = vtr_lidar_msgs::msg::IntensityFeatures;

  /// Construct from ROS2 message (deserialization)
  static Ptr fromStorable(const IntensityFeaturesMsg& storable);
  /// Convert to ROS2 message (serialization)
  IntensityFeaturesMsg toStorable() const;

  IntensityFeatures() = default;

  /// ORB keypoints (2D pixel locations on intensity image)
  std::vector<cv::KeyPoint> keypoints;
  /// ORB descriptors (CV_8U, rows=N, cols=32 for ORB)
  cv::Mat descriptors;
  /// 3D positions of each feature in the sensor frame (3 x N)
  Eigen::Matrix<double, 3, Eigen::Dynamic> points_3d;
  /// Per-feature timestamps (nanoseconds, same clock as PointWithInfo::timestamp)
  std::vector<int64_t> timestamps;
  /// Image dimensions
  int image_width = 0;
  int image_height = 0;

  int size() const { return static_cast<int>(keypoints.size()); }
  bool empty() const { return keypoints.empty(); }
};

}  // namespace lidar
}  // namespace vtr

#include "vtr_lidar/data_types/intensity_features.inl"