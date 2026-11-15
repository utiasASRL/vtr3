
/**
 * \file test_intensity_features.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 */

#include <gmock/gmock.h>

#include "vtr_lidar/data_types/intensity_feature_map.hpp"
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

TEST(LIDAR, intensity_features_covariance_roundtrip) {
  IntensityFeatures original;
  for (int i = 0; i < 3; i++) {
    original.keypoints.emplace_back(cv::KeyPoint(i * 10.0f, i * 5.0f, 1.0f));
    Eigen::Matrix3d cov = (i + 1) * 0.01 * Eigen::Matrix3d::Identity();
    cov(0, 1) = cov(1, 0) = 0.001 * i;
    original.covs.push_back(cov);
  }
  original.descriptors = cv::Mat::ones(3, 32, CV_8U);
  original.points_3d = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, 3);
  original.points_3d.setOnes();
  ASSERT_TRUE(original.hasCovs());

  const auto restored = IntensityFeatures::fromStorable(original.toStorable());
  ASSERT_TRUE(restored->hasCovs());
  for (int i = 0; i < 3; i++)
    EXPECT_TRUE(restored->covs[i].isApprox(original.covs[i]));

  // features without covariances stay without
  original.covs.clear();
  EXPECT_FALSE(
      IntensityFeatures::fromStorable(original.toStorable())->hasCovs());
}

namespace {

/// One feature for IntensityFeatureMap::update.
struct MapInput {
  Eigen::Matrix<double, 3, Eigen::Dynamic> points;
  std::vector<Eigen::Matrix3d> covs;
  cv::Mat descriptors;
  std::vector<float> responses;
};

MapInput oneFeature(const Eigen::Vector3d& p, double sigma, uchar desc_byte) {
  MapInput in;
  in.points.resize(3, 1);
  in.points.col(0) = p;
  in.covs.push_back(sigma * sigma * Eigen::Matrix3d::Identity());
  in.descriptors = cv::Mat(1, 32, CV_8U, cv::Scalar(desc_byte));
  in.responses.push_back(1.0f);
  return in;
}

void updateMap(IntensityFeatureMap& map, const MapInput& in,
               const std::vector<int>& matched = {}) {
  map.update(in.points, in.covs, in.descriptors, in.responses, matched, 20.0f);
}

}  // namespace

TEST(LIDAR, intensity_feature_map_association) {
  // gate 11.34 (chi^2, 3 dof, 99 %), radius 2 m, Hamming <= 50
  IntensityFeatureMap map(11.34f, 2.0f, 50.0f);
  const Eigen::Vector3d p0(10.0, 0.0, 0.0);

  // first feature: a new landmark
  updateMap(map, oneFeature(p0, 0.1, 0x00));
  ASSERT_EQ(map.size(), 1u);
  EXPECT_EQ(map.last_update().added, 1);

  // same descriptor, 0.1 m away with sigma 0.1 m: the same landmark. It is
  // less accurate than the stored observation, so the position stays.
  updateMap(map, oneFeature(p0 + Eigen::Vector3d(0.1, 0.0, 0.0), 0.2, 0x00));
  ASSERT_EQ(map.size(), 1u);
  EXPECT_EQ(map.last_update().merged, 1);
  EXPECT_EQ(map.landmarks()[0].num_obs, 2);
  EXPECT_TRUE(map.landmarks()[0].p.isApprox(p0));

  // a more accurate observation replaces position and covariance
  const Eigen::Vector3d p_better = p0 + Eigen::Vector3d(0.02, 0.0, 0.0);
  updateMap(map, oneFeature(p_better, 0.05, 0x00));
  ASSERT_EQ(map.size(), 1u);
  EXPECT_TRUE(map.landmarks()[0].p.isApprox(p_better));
  EXPECT_NEAR(map.landmarks()[0].cov(0, 0), 0.05 * 0.05, 1e-12);

  // same place, different descriptor (Hamming 256): another landmark
  updateMap(map, oneFeature(p0, 0.1, 0xFF));
  EXPECT_EQ(map.size(), 2u);

  // same descriptor but outside the Mahalanobis gate (1 m with 5 cm sigmas)
  updateMap(map, oneFeature(p0 + Eigen::Vector3d(1.0, 0.0, 0.0), 0.05, 0x00));
  EXPECT_EQ(map.size(), 3u);

  // a feature that odometry matched updates its landmark whatever the
  // distance and the descriptor (index 1: the 0xFF landmark)
  updateMap(map, oneFeature(p0 + Eigen::Vector3d(0.5, 0.0, 0.0), 0.01, 0x0F),
            {1});
  ASSERT_EQ(map.size(), 3u);
  EXPECT_EQ(map.last_update().matched, 1);
  EXPECT_EQ(map.landmarks()[1].num_obs, 2);
  EXPECT_EQ(map.landmarks()[1].descriptor.at<uchar>(0, 0), 0x0F);
}

TEST(LIDAR, intensity_feature_map_life_time_and_storage) {
  IntensityFeatureMap map(11.34f, 2.0f, 50.0f);
  MapInput in = oneFeature(Eigen::Vector3d(5.0, 1.0, 0.5), 0.1, 0x55);
  in.covs[0](0, 1) = in.covs[0](1, 0) = 0.002;
  map.update(in.points, in.covs, in.descriptors, in.responses, {}, 2.0f);
  ASSERT_EQ(map.size(), 1u);

  // storage round trip keeps the covariance
  const auto restored = IntensityFeatureMap::fromStorable(map.toStorable());
  ASSERT_EQ(restored->size(), 1u);
  EXPECT_TRUE(restored->landmarks()[0].p.isApprox(map.landmarks()[0].p));
  EXPECT_TRUE(restored->landmarks()[0].cov.isApprox(in.covs[0]));

  // not observed again: removed after its life time
  map.age();
  EXPECT_EQ(map.size(), 1u);
  map.age();
  EXPECT_EQ(map.size(), 0u);
}

int main(int argc, char** argv) {
  vtr::logging::configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}