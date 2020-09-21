// Internal
#include <asrl/vision/outliers.hpp>
#include <asrl/vision/sensors.hpp>

// External
#include "catch.hpp"
#include <Eigen/Geometry>
#include <random>
#include <chrono>
#include <cmath>

namespace av = asrl::vision;

SCENARIO("Stereo ransac with a simple problem", "[matching][stereo][ransac]" ) {
  GIVEN("Two sets of points") {

    // Create some intrinsics
    double baseline = 0.1;
    double focal = 1000;
    double cx = 500;
    double cy = 500;
    av::CameraIntrinsic intrinsics;
    intrinsics << focal, 0, cx,
            0, focal, cy,
            0, 0, 1;

    // Create the camera matrices
    av::CameraProjection projection_l = intrinsics*av::CameraProjection::Identity();
    av::CameraProjection projection_r = av::CameraProjection::Identity();
    projection_r(0,3) = baseline;
    projection_r = intrinsics*projection_r;

    // Weird ref pointer unsafe type conversion
    //Eigen::MatrixXd test (5,5);
    //const Eigen::Matrix<double,3,Eigen::Dynamic>& test_ref = test;
    //const Eigen::Matrix<double,3,Eigen::Dynamic>* test_ref_ptr = &test_ref; // WTF?
    //const Eigen::Matrix<double,3,Eigen::Dynamic>* test_ptr = &test; // Doesn't compile

    // Create the ground truth transformation matrix
    // Will transform points in reference frame to camera frame
    Eigen::Affine3d tf = Eigen::Affine3d::Identity();
    tf.translation() << 0, 0.5, -0.8;

    // Create the two point clouds
    const unsigned int n_inliers = 4;
    const unsigned int n_outliers = 3;
    const unsigned int n_pts = n_inliers + n_outliers;
    Eigen::Matrix<double,3,Eigen::Dynamic> pts_ref(3,n_pts), pts_cam(3,n_pts);
    pts_ref << 1, 0, 1, 0, 2, 3, -2,
        0, 0, 0, -1, 1, 0, -1,
        3, 4, 3, 3, 4, 5, 6;
    pts_cam = tf * pts_ref;

    // Corrupt the outlier points
    for (unsigned int i = n_inliers; i < n_pts; ++i) {
      pts_cam(1,i) += (i % 2) ? -1 : 1;
      pts_cam(2,i) += i % 3;
    }

    // Log the point clouds
//    INFO( "ref:\n" << pts_ref
//          << "\ncam:\n" << pts_cam
//          << "\n" );
    REQUIRE( tf*pts_ref.leftCols<n_inliers>()
             == pts_cam.leftCols<n_inliers>() );
    REQUIRE( tf*pts_ref.rightCols<n_outliers>()
             != pts_cam.rightCols<n_outliers>() );

    // Create the match pairs (just 1-1)
    av::SimpleMatches matches;
    matches.reserve(n_pts);
    for (unsigned int i = 0; i < n_pts; ++i)
      matches.push_back(av::SimpleMatch(i,i));

    WHEN("We run ransac") {

      // Create the camera model
      av::StereoTransformModel::Ptr model = std::make_shared<av::StereoTransformModel>();
      model->setCalibration(projection_l, projection_r);
      model->setCalibration(intrinsics, baseline); // double check that overloaded constructor works
      model->setPoints(&pts_ref, &pts_cam);

      // Set up RANSAC
      auto verifier = std::make_shared<av::VerifySampleIndices>(); // Need 1 close feature
      auto sampler = std::make_shared<av::BasicSampler>(verifier);
      av::VanillaRansac<av::StereoTransformModel::SolutionType> ransac(sampler);
      ransac.setCallback(model);

      // Run RANSAC
      Eigen::Matrix4d tf_found;
      av::SimpleMatches inliers;
      int n_inliers_found = ransac.run(matches, &tf_found, &inliers);

      THEN("We should find the solution and inliers") {
        REQUIRE( n_inliers_found == n_inliers );
        INFO("tf:\n" << tf.matrix() << "\nfound:\n" << tf_found << "\n");
        REQUIRE( (tf.matrix() - tf_found).norm() == Approx(0.) );
      }
    }
  }
}

SCENARIO("Stereo ransac with a more complicated problem", "[matching][stereo][ransac]" ) {
  GIVEN("Two sets of points") {
     std::default_random_engine eng;
     eng.seed(std::chrono::system_clock::now().time_since_epoch().count());
    // Create some intrinsics
    double baseline = 0.24;
    std::uniform_real_distribution<> focal_dis(1000,3000);
    double focal = focal_dis(eng);
    double width = 1024;
    double height = 768;
    double cx = width/2.0;
    double cy = height/2.0;
    av::CameraIntrinsic intrinsics;
    intrinsics << focal, 0, cx,
            0, focal, cy,
            0, 0, 1;
    //double hfov = 2*atan2(0.5*width,focal);
    //double vfov = 2*atan2(0.5*height,focal);
    // Create the camera matrices
    av::CameraProjection projection_l = intrinsics*av::CameraProjection::Identity();
    av::CameraProjection projection_r = av::CameraProjection::Identity();
    projection_r(0,3) = baseline;
    projection_r = intrinsics*projection_r;

    // Create the ground truth transformation matrix
    // Will transform points in reference frame to camera frame
    Eigen::Affine3d tf = Eigen::Affine3d::Identity();
    std::uniform_real_distribution<> trans_dis(-1.0,1.0);
    tf.translation() << trans_dis(eng), trans_dis(eng), trans_dis(eng);

    // Create the two point clouds
    const unsigned int n_inliers = 500;
    const unsigned int n_outliers = 500;
    const unsigned int n_pts = n_inliers + n_outliers;
    Eigen::Matrix<double,3,Eigen::Dynamic> pts_ref(3,n_pts), pts_cam(3,n_pts);
    std::uniform_real_distribution<> x_dis(-10.0,10.0);
    std::uniform_real_distribution<> y_dis(0.5,1.0);
    std::uniform_real_distribution<> z_dis(3.0,50.0);
    tf.translation() << trans_dis(eng), trans_dis(eng), trans_dis(eng);
    for(unsigned ii = 0; ii < n_pts; ii++) {
        pts_ref.col(ii) << x_dis(eng), y_dis(eng), z_dis(eng);
        pts_cam.col(ii) = tf * pts_ref.col(ii);
    }

    // Corrupt the outlier points
    std::normal_distribution<> sample_dis(0.0,0.5);
    for(unsigned i = n_inliers; i < n_pts; ++i) {
      pts_cam(0,i) += sample_dis(eng);
      pts_cam(1,i) += sample_dis(eng);
      pts_cam(2,i) += sample_dis(eng);
    }
    // Log the point clouds
//    INFO( "ref:\n" << pts_ref
//          << "\ncam:\n" << pts_cam
//          << "\n" );
    REQUIRE( tf*pts_ref.leftCols<n_inliers>()
             == pts_cam.leftCols<n_inliers>() );
    REQUIRE( tf*pts_ref.rightCols<n_outliers>()
             != pts_cam.rightCols<n_outliers>() );

    // Create the match pairs (just 1-1)
    av::SimpleMatches matches;
    matches.reserve(n_pts);
    for (unsigned int i = 0; i < n_pts; ++i)
      matches.push_back(av::SimpleMatch(i,i));

    WHEN("We run ransac") {

      // Create the camera model
      av::StereoTransformModel::Ptr model = std::make_shared<av::StereoTransformModel>();
      model->setCalibration(projection_l, projection_r);
      model->setCalibration(intrinsics, baseline); // double check that overloaded constructor works
      model->setPoints(&pts_ref, &pts_cam);

      // Set up RANSAC
      auto verifier = std::make_shared<av::VerifySampleIndices>(); // Need 1 close feature
      auto sampler = std::make_shared<av::BasicSampler>(verifier);
      av::VanillaRansac<av::StereoTransformModel::SolutionType> ransac(sampler,3.5,5.0,100,0.25,150);
      ransac.setCallback(model);

      // Run RANSAC
      Eigen::Matrix4d tf_found;
      av::SimpleMatches inliers;
      int n_inliers_found = ransac.run(matches, &tf_found, &inliers);

      THEN("We should find the solution and inliers") {
        REQUIRE( n_inliers_found >= 0.9*n_inliers );
        INFO("tf:\n" << tf.matrix() << "\nfound:\n" << tf_found << "\n");
        REQUIRE( (tf.matrix() - tf_found).norm() == Approx(0.).epsilon(0.01) );
      }
    }
  }
}
