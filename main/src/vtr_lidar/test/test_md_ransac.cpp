/**
 * \file test_md_ransac.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Tests of the Ouster measurement Jacobians and of the motion-distorted
 * RANSAC on synthetic matches with a known sensor velocity.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <random>

#include <lgmath.hpp>

#include "vtr_lidar/features/md_ransac.hpp"
#include "vtr_lidar/features/ouster_projector.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace ::testing;
using namespace vtr::lidar;

namespace {

/// An OS1-128-like sensor: 128 x 1024, beams from +22.5 to -22.5 deg.
OusterProjector::Ptr makeProjector() {
  auto config = std::make_shared<OusterProjectorConfig>();
  config->pixels_per_column = 128;
  config->columns_per_frame = 1024;
  config->lidar_origin_to_beam_origin_mm = 15.806;
  config->pixel_shift_by_row.assign(128, 0);
  for (int i = 0; i < 128; ++i)
    config->beam_altitude_angles.push_back(22.5 - 45.0 * i / 127.0);
  return std::make_shared<OusterProjector>(config);
}

/// A point in the field of view (away from the image seam at +-pi).
Eigen::Vector3d randomPoint(std::mt19937& rng) {
  std::uniform_real_distribution<double> az(-3.0, 3.0);
  std::uniform_real_distribution<double> el(-18.0 * M_PI / 180.0,
                                            18.0 * M_PI / 180.0);
  std::uniform_real_distribution<double> range(5.0, 30.0);
  const double a = az(rng), e = el(rng), r = range(rng);
  return Eigen::Vector3d(r * std::cos(e) * std::cos(a),
                         r * std::cos(e) * std::sin(a), r * std::sin(e));
}

}  // namespace

TEST(LIDAR, ouster_measurement_jacobian) {
  const auto projector = makeProjector();
  std::mt19937 rng(1);
  for (int k = 0; k < 20; ++k) {
    const Eigen::Vector3d p = randomPoint(rng);
    const Eigen::Matrix3d F = projector->measurementJacobian(p);

    // numerical Jacobian of z = h(p) = [u, v, rho]
    Eigen::Matrix3d F_num;
    const double eps = 1e-6;
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d dp = Eigen::Vector3d::Zero();
      dp(j) = eps;
      Eigen::Vector3d z_plus, z_minus;
      ASSERT_TRUE(mdRansacMeasure(*projector, p + dp, z_plus));
      ASSERT_TRUE(mdRansacMeasure(*projector, p - dp, z_minus));
      F_num.col(j) = (z_plus - z_minus) / (2.0 * eps);
    }
    // image column and range rows: exact model
    EXPECT_LT((F.row(0) - F_num.row(0)).norm(), 1e-4 * F_num.row(0).norm());
    EXPECT_LT((F.row(2) - F_num.row(2)).norm(), 1e-4);
    // image row: projectPoint interpolates the beam angle table, the
    // Jacobian uses the mean beam spacing (128 / 127 apart for this sensor)
    EXPECT_LT((F.row(1) - F_num.row(1)).norm(), 0.02 * F_num.row(1).norm());

    // the closed-form Jacobian of the inverse projection is the inverse of
    // the measurement Jacobian (inverse function theorem)
    const Eigen::Matrix3d J = projector->backProjectJacobian(p);
    EXPECT_LT((J * F - Eigen::Matrix3d::Identity()).norm(), 1e-9);

    // and it matches numerical differences of the inverse projection
    //   p = h^-1([u, v, rho]) of the linear beam model
    const Eigen::Matrix3d& K = projector->K();
    const double b = projector->beamOffsetM();
    const auto inverse = [&](const Eigen::Vector3d& z) {
      const double phi = (z(0) - K(0, 2)) / K(0, 0);
      const double theta = (z(1) - K(1, 2)) / K(1, 1);
      const double R = std::sqrt(z(2) * z(2) - b * b * std::sin(theta) * std::sin(theta)) -
                       b * std::cos(theta);
      const double rxy = b + R * std::cos(theta);
      return Eigen::Vector3d(rxy * std::cos(phi), rxy * std::sin(phi), R * std::sin(theta));
    };
    const double rxy = p.head<2>().norm();
    const Eigen::Vector3d z0(K(0, 0) * std::atan2(p.y(), p.x()) + K(0, 2),
                             K(1, 1) * std::atan2(p.z(), rxy - b) + K(1, 2), p.norm());
    ASSERT_LT((inverse(z0) - p).norm(), 1e-9);
    Eigen::Matrix3d J_num;
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d dz = Eigen::Vector3d::Zero();
      dz(j) = 1e-5;
      J_num.col(j) = (inverse(z0 + dz) - inverse(z0 - dz)) / 2e-5;
    }
    EXPECT_LT((J - J_num).norm(), 1e-6 * J_num.norm());
  }
}

TEST(LIDAR, md_ransac_recovers_velocity_and_rejects_outliers) {
  const auto projector = makeProjector();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> time(0.0, 0.1);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::normal_distribution<double> noise(0.0, 0.005);

  // sensor velocity [v; omega]: driving and turning
  Eigen::Matrix<double, 6, 1> omega_true;
  omega_true << -1.5, 0.2, 0.05, 0.02, -0.03, 0.5;

  const int num_matches = 200;
  std::vector<MdRansacMatch> matches;
  std::vector<bool> is_inlier;
  for (int i = 0; i < num_matches; ++i) {
    MdRansacMatch m;
    m.p1 = randomPoint(rng);
    m.dt = time(rng);
    if (uniform(rng) < 0.3) {
      m.p2 = randomPoint(rng);  // wrong descriptor match
      is_inlier.push_back(false);
    } else {
      const Eigen::Matrix4d T = lgmath::se3::vec2tran(omega_true * m.dt);
      m.p2 = T.block<3, 3>(0, 0) * m.p1 + T.block<3, 1>(0, 3) +
             Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
      is_inlier.push_back(true);
    }
    matches.push_back(m);
  }
  const int num_true = std::count(is_inlier.begin(), is_inlier.end(), true);

  const auto result = mdRansacSolve(matches, *projector, MdRansacParams());
  ASSERT_TRUE(result.success);

  // full 6-DoF velocity, translation included
  EXPECT_LT((result.omega.head<3>() - omega_true.head<3>()).norm(), 0.1);
  EXPECT_LT((result.omega.tail<3>() - omega_true.tail<3>()).norm(), 0.02);

  int num_correct = 0, num_wrong = 0;
  for (const int i : result.inliers) (is_inlier[i] ? num_correct : num_wrong)++;
  EXPECT_GE(num_correct, static_cast<int>(0.95 * num_true));
  EXPECT_LE(num_wrong, 2);
  // adaptive stop: far fewer hypotheses than the cap
  EXPECT_LT(result.num_hypotheses, MdRansacParams().max_iterations);
}

TEST(LIDAR, md_ransac_fails_on_random_matches) {
  const auto projector = makeProjector();
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> time(0.0, 0.1);
  std::vector<MdRansacMatch> matches;
  for (int i = 0; i < 100; ++i) {
    MdRansacMatch m;
    m.p1 = randomPoint(rng);
    m.p2 = randomPoint(rng);
    m.dt = time(rng);
    matches.push_back(m);
  }
  const auto result = mdRansacSolve(matches, *projector, MdRansacParams());
  EXPECT_FALSE(result.success);

  // too few matches
  matches.resize(5);
  EXPECT_FALSE(mdRansacSolve(matches, *projector, MdRansacParams()).success);
}

int main(int argc, char** argv) {
  vtr::logging::configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
