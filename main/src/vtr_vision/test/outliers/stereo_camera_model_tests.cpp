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
 * \file stereo_camera_model_tests.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include <vtr_logging/logging_init.hpp>
#include <vtr_vision/outliers.hpp>
#include <vtr_vision/sensors/stereo_transform_model.hpp>

using namespace vtr::vision;

TEST(Vision, stereoModel1) {
  // Create some intrinsics
  double baseline = 0.1;
  double focal = 1000;
  double cx = 500;
  double cy = 500;
  CameraIntrinsic intrinsics;
  intrinsics << focal, 0, cx, 0, focal, cy, 0, 0, 1;

  // Create the camera matrices
  CameraProjection projection_l = intrinsics * CameraProjection::Identity();
  CameraProjection projection_r = CameraProjection::Identity();
  projection_r(0, 3) = baseline;
  projection_r = intrinsics * projection_r;

  // Weird ref pointer unsafe type conversion (old)
  // Eigen::MatrixXd test (5,5);
  // const Eigen::Matrix<double,3,Eigen::Dynamic>& test_ref = test;
  // const Eigen::Matrix<double,3,Eigen::Dynamic>* test_ref_ptr = &test_ref; //
  // WTF? const Eigen::Matrix<double,3,Eigen::Dynamic>* test_ptr = &test; //
  // Doesn't compile

  // Create the ground truth transformation matrix
  // Will transform points in reference frame to camera frame
  Eigen::Affine3d tf = Eigen::Affine3d::Identity();
  tf.translation() << 0, 0.5, -0.8;

  // Create the two point clouds
  const unsigned int n_inliers = 4;
  const unsigned int n_outliers = 3;
  const unsigned int n_pts = n_inliers + n_outliers;
  Eigen::Matrix<double, 3, Eigen::Dynamic> pts_ref(3, n_pts), pts_cam(3, n_pts);
  pts_ref << 1, 0, 1, 0, 2, 3, -2, 0, 0, 0, -1, 1, 0, -1, 3, 4, 3, 3, 4, 5, 6;
  pts_cam = tf * pts_ref;

  // Corrupt the outlier points
  for (unsigned int i = n_inliers; i < n_pts; ++i) {
    pts_cam(1, i) += (i % 2) ? -1 : 1;
    pts_cam(2, i) += i % 3;
  }

  // Log the point clouds
  //    INFO( "ref:\n" << pts_ref
  //          << "\ncam:\n" << pts_cam
  //          << "\n" );
  EXPECT_EQ(tf * pts_ref.leftCols<n_inliers>(), pts_cam.leftCols<n_inliers>());
  EXPECT_NE(tf * pts_ref.rightCols<n_outliers>(),
            pts_cam.rightCols<n_outliers>());

  // Create the match pairs (just 1-1)
  SimpleMatches matches;
  matches.reserve(n_pts);
  for (unsigned int i = 0; i < n_pts; ++i) matches.push_back(SimpleMatch(i, i));

  // Create the camera model
  StereoTransformModel::Ptr model = std::make_shared<StereoTransformModel>();
  model->setCalibration(projection_l, projection_r);
  model->setCalibration(
      intrinsics, baseline);  // double check that overloaded constructor works
  model->setPoints(&pts_ref, &pts_cam);

  // Set up RANSAC
  auto verifier =
      std::make_shared<VerifySampleIndices>();  // Need 1 close feature
  auto sampler = std::make_shared<BasicSampler>(verifier);
  VanillaRansac<StereoTransformModel::SolutionType> ransac(sampler);
  ransac.setCallback(model);

  // Run RANSAC
  Eigen::Matrix4d tf_found;
  SimpleMatches inliers;
  unsigned int n_inliers_found = ransac.run(matches, &tf_found, &inliers);

  EXPECT_EQ(n_inliers_found, n_inliers);
  LOG(INFO) << "tf:\n" << tf.matrix() << "\nfound:\n" << tf_found << "\n";
  EXPECT_NEAR((tf.matrix() - tf_found).norm(), 0, std::pow(10, -9));
}

TEST(Vision, stereoModel2) {
  std::default_random_engine eng;
  eng.seed(std::chrono::system_clock::now().time_since_epoch().count());
  // Create some intrinsics
  double baseline = 0.24;
  std::uniform_real_distribution<> focal_dis(1000, 3000);
  double focal = focal_dis(eng);
  double width = 1024;
  double height = 768;
  double cx = width / 2.0;
  double cy = height / 2.0;
  CameraIntrinsic intrinsics;
  intrinsics << focal, 0, cx, 0, focal, cy, 0, 0, 1;
  // double hfov = 2*atan2(0.5*width,focal);
  // double vfov = 2*atan2(0.5*height,focal);
  // Create the camera matrices
  CameraProjection projection_l = intrinsics * CameraProjection::Identity();
  CameraProjection projection_r = CameraProjection::Identity();
  projection_r(0, 3) = baseline;
  projection_r = intrinsics * projection_r;

  // Create the ground truth transformation matrix
  // Will transform points in reference frame to camera frame
  Eigen::Affine3d tf = Eigen::Affine3d::Identity();
  std::uniform_real_distribution<> trans_dis(-1.0, 1.0);
  tf.translation() << trans_dis(eng), trans_dis(eng), trans_dis(eng);

  // Create the two point clouds
  const unsigned int n_inliers = 500;
  const unsigned int n_outliers = 500;
  const unsigned int n_pts = n_inliers + n_outliers;
  Eigen::Matrix<double, 3, Eigen::Dynamic> pts_ref(3, n_pts), pts_cam(3, n_pts);
  std::uniform_real_distribution<> x_dis(-10.0, 10.0);
  std::uniform_real_distribution<> y_dis(0.5, 1.0);
  std::uniform_real_distribution<> z_dis(3.0, 50.0);
  tf.translation() << trans_dis(eng), trans_dis(eng), trans_dis(eng);
  for (unsigned ii = 0; ii < n_pts; ii++) {
    pts_ref.col(ii) << x_dis(eng), y_dis(eng), z_dis(eng);
    pts_cam.col(ii) = tf * pts_ref.col(ii);
  }

  // Corrupt the outlier points
  std::normal_distribution<> sample_dis(0.0, 0.5);
  for (unsigned i = n_inliers; i < n_pts; ++i) {
    pts_cam(0, i) += sample_dis(eng);
    pts_cam(1, i) += sample_dis(eng);
    pts_cam(2, i) += sample_dis(eng);
  }
  // Log the point clouds
  //    INFO( "ref:\n" << pts_ref
  //          << "\ncam:\n" << pts_cam
  //          << "\n" );
  EXPECT_EQ(tf * pts_ref.leftCols<n_inliers>(), pts_cam.leftCols<n_inliers>());
  EXPECT_NE(tf * pts_ref.rightCols<n_outliers>(),
            pts_cam.rightCols<n_outliers>());

  // Create the match pairs (just 1-1)
  SimpleMatches matches;
  matches.reserve(n_pts);
  for (unsigned int i = 0; i < n_pts; ++i) matches.push_back(SimpleMatch(i, i));

  // Create the camera model
  StereoTransformModel::Ptr model = std::make_shared<StereoTransformModel>();
  model->setCalibration(projection_l, projection_r);
  model->setCalibration(
      intrinsics, baseline);  // double check that overloaded constructor works
  model->setPoints(&pts_ref, &pts_cam);

  // Set up RANSAC
  auto verifier =
      std::make_shared<VerifySampleIndices>();  // Need 1 close feature
  auto sampler = std::make_shared<BasicSampler>(verifier);
  VanillaRansac<StereoTransformModel::SolutionType> ransac(sampler, 3.5, 5.0,
                                                           100, 0.25, 150);
  ransac.setCallback(model);

  // Run RANSAC
  Eigen::Matrix4d tf_found;
  SimpleMatches inliers;
  int n_inliers_found = ransac.run(matches, &tf_found, &inliers);

  EXPECT_GE(n_inliers_found, 0.9 * n_inliers);
  LOG(INFO) << "tf:\n" << tf.matrix() << "\nfound:\n" << tf_found << "\n";
  EXPECT_NEAR((tf.matrix() - tf_found).norm(), 0, std::pow(10, -9));
}
