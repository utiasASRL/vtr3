/**
 * \file md_ransac.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Motion-Distorted RANSAC for lidar intensity feature odometry.
 *
 *   Model: constant body velocity ω = [v; ω] ∈ ℝ⁶
 *   Per-match transform: T_m = Exp(−Δt_m · ω)
 *   Prediction: ŷ₂ = f(T_m · p₁)
 *
 *   1. Sample 3 matches
 *   2. Estimate ω via linearised model (scaled by per-match Δt)
 *   3. Count inliers  ‖y₂ − ŷ₂‖ < α
 *   4. Keep best model
 *
 * Reference: LIVO package (livo_ws/src/livo/src/md_ransac.cpp)
 */
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "vtr_lidar/features/ouster_projector.hpp"

namespace vtr {
namespace lidar {

/// A visual match between previous and current frame features.
/// Used by MD-RANSAC and the reprojection error cost terms.
struct VisualMatch {
  Eigen::Vector3d p1 = Eigen::Vector3d::Zero();  ///< 3-D point in prev sensor frame
  Eigen::Vector2d y1 = Eigen::Vector2d::Zero();  ///< pixel in previous frame
  Eigen::Vector2d y2 = Eigen::Vector2d::Zero();  ///< pixel in current frame
  int64_t timestamp_1 = 0;  ///< timestamp of p1
  int64_t timestamp_2 = 0;  ///< timestamp of y2 (current frame feature)
  double dt = 0.0;          ///< relative time offset (seconds)
  int prev_idx = -1;        ///< index into previous frame features
  int curr_idx = -1;        ///< index into current frame features
};

/**
 * \brief Run motion-distorted RANSAC on a set of visual matches.
 *
 * \param matches       All visual matches between prev and curr frame
 * \param projector     OusterProjector for projection / back-projection
 * \param max_iterations RANSAC iterations
 * \param inlier_threshold  Pixel-space inlier threshold
 * \param min_inliers   Minimum inliers to accept the model
 * \param[out] best_inliers  Indices into `matches` that are inliers
 * \param[out] best_omega    Best estimated body velocity [v; ω] ∈ ℝ⁶
 * \return true if a valid model was found (≥ min_inliers)
 */
bool mdRansacSolve(
    const std::vector<VisualMatch>& matches,
    const OusterProjector& projector,
    int max_iterations,
    double inlier_threshold,
    int min_inliers,
    std::vector<int>& best_inliers,
    Eigen::Matrix<double, 6, 1>& best_omega);

}  // namespace lidar
}  // namespace vtr
