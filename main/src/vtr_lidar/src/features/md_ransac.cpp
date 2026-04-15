/**
 * \file md_ransac.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Motion-Distorted RANSAC implementation.
 * Reference: LIVO package (livo_ws/src/livo/src/md_ransac.cpp)
 */
#include "vtr_lidar/features/md_ransac.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <lgmath.hpp>

namespace vtr {
namespace lidar {

namespace {

/// Estimate constant body velocity from 3 matches via linearised model:
///   p2 ≈ p1 + dt * [I  -[p1]×] · ω
bool estimateVelocity3(
    const std::vector<VisualMatch>& matches,
    const std::vector<int>& sample,
    const OusterProjector& projector,
    Eigen::Matrix<double, 6, 1>& omega) {
  Eigen::Matrix<double, 9, 6> A;
  Eigen::Matrix<double, 9, 1> b;
  A.setZero();
  b.setZero();

  for (int s = 0; s < 3; ++s) {
    const auto& m = matches[sample[s]];
    const double range1 = m.p1.norm();
    if (range1 < 0.5) return false;

    // Approximate p2 by back-projecting y2 at range of p1
    const Eigen::Vector3d p2 = projector.backProject(m.y2, range1);

    // dp = p2 - p1 ≈ -dt * [I  -[p1]×] · ω
    const Eigen::Vector3d dp = p2 - m.p1;
    const Eigen::Matrix3d p1x = lgmath::so3::hat(m.p1);
    const double dt = m.dt;

    A.block<3, 3>(3 * s, 0) = -dt * Eigen::Matrix3d::Identity();
    A.block<3, 3>(3 * s, 3) = dt * p1x;
    b.segment<3>(3 * s) = dp;
  }

  Eigen::Matrix<double, 6, 6> ATA = A.transpose() * A;
  if (std::abs(ATA.determinant()) < 1e-20) return false;

  omega = ATA.ldlt().solve(A.transpose() * b);
  return true;
}

/// Count MD-RANSAC inliers for a given body velocity
void countInliers(
    const std::vector<VisualMatch>& matches,
    const Eigen::Matrix<double, 6, 1>& omega,
    double threshold,
    const OusterProjector& projector,
    std::vector<int>& inliers) {
  inliers.clear();
  const double thresh2 = threshold * threshold;

  for (size_t i = 0; i < matches.size(); ++i) {
    const auto& m = matches[i];

    // Per-match transform: T_m = Exp(-dt * ω)
    const Eigen::Matrix4d T_m = lgmath::se3::vec2tran(-m.dt * omega);

    // Predicted point in current frame
    const Eigen::Vector3d p2_hat =
        T_m.block<3, 3>(0, 0) * m.p1 + T_m.block<3, 1>(0, 3);

    // Project to pixel
    Eigen::Vector2d y2_hat;
    if (!projector.projectPoint(p2_hat, y2_hat)) continue;

    // Reprojection error
    const double err2 = (m.y2 - y2_hat).squaredNorm();
    if (err2 < thresh2) {
      inliers.push_back(static_cast<int>(i));
    }
  }
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════════
bool mdRansacSolve(
    const std::vector<VisualMatch>& matches,
    const OusterProjector& projector,
    int max_iterations,
    double inlier_threshold,
    int min_inliers,
    std::vector<int>& best_inliers,
    Eigen::Matrix<double, 6, 1>& best_omega) {
  best_inliers.clear();
  best_omega.setZero();

  const int N = static_cast<int>(matches.size());
  if (N < 3) return false;

  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(0, N - 1);

  int best_count = 0;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Sample 3 distinct matches
    std::vector<int> sample(3);
    sample[0] = dist(rng);
    do { sample[1] = dist(rng); } while (sample[1] == sample[0]);
    do { sample[2] = dist(rng); } while (sample[2] == sample[0] || sample[2] == sample[1]);

    // Estimate velocity
    Eigen::Matrix<double, 6, 1> xi_cand;
    if (!estimateVelocity3(matches, sample, projector, xi_cand)) continue;

    // Count inliers
    std::vector<int> cand_inliers;
    countInliers(matches, xi_cand, inlier_threshold, projector, cand_inliers);

    if (static_cast<int>(cand_inliers.size()) > best_count) {
      best_count = static_cast<int>(cand_inliers.size());
      best_omega = xi_cand;
      best_inliers = cand_inliers;
    }
  }

  return best_count >= min_inliers;
}

}  // namespace lidar
}  // namespace vtr
