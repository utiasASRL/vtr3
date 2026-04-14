
/**
 * \file test_intensity_features.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */

#include <gmock/gmock.h>

#include "vtr_lidar/data_types/intensity_features.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace ::testing;
using namespace vtr::lidar;

TEST(LIDAR, intensity_features_default_construction) {
  IntensityFeatures features;
  EXPECT_TRUE(features.empty());
  EXPECT_EQ(features.size(), 0);
  EXPECT_EQ(features.image_width, 0);
  EXPECT_EQ(features.image_height, 0);
}

TEST(LIDAR, intensity_features_add_keypoints) {
  IntensityFeatures features;

  // Add some fake keypoints
  for (int i = 0; i < 5; i++) {
    features.keypoints.emplace_back(cv::KeyPoint(i * 10.0f, i * 5.0f, 1.0f));
  }
  features.descriptors = cv::Mat::ones(5, 32, CV_8U);
  features.points_3d = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, 5);
  features.points_3d.setRandom();
  features.image_width = 640;
  features.image_height = 480;

  EXPECT_EQ(features.size(), 5);
  EXPECT_FALSE(features.empty());
}

TEST(LIDAR, intensity_features_serialization_roundtrip) {
  // Create and populate features
  IntensityFeatures original;
  for (int i = 0; i < 3; i++) {
    original.keypoints.emplace_back(cv::KeyPoint(i * 10.0f, i * 5.0f, 1.0f));
  }
  original.descriptors = cv::Mat::ones(3, 32, CV_8U) * 42;
  original.points_3d = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, 3);
  original.points_3d.setOnes();
  original.image_width = 640;
  original.image_height = 480;

  // Serialize then deserialize
  auto msg = original.toStorable();
  auto restored = IntensityFeatures::fromStorable(msg);

  // Verify round-trip
  EXPECT_EQ(restored->size(), original.size());
  EXPECT_EQ(restored->image_width, original.image_width);
  EXPECT_EQ(restored->image_height, original.image_height);
  EXPECT_EQ(restored->descriptors.rows, original.descriptors.rows);
  EXPECT_EQ(restored->descriptors.cols, original.descriptors.cols);
  EXPECT_TRUE(restored->points_3d.isApprox(original.points_3d));
}

int main(int argc, char** argv) {
  vtr::logging::configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}