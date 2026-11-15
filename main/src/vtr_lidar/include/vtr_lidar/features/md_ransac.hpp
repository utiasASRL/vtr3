/**
 * \file md_ransac.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Motion-distorted RANSAC (Anderson and Barfoot, 2013) for lidar
 * intensity-image features, full 6-DoF.
 *
 * Rejects wrong descriptor matches between the features of the current scan
 * and landmarks, without using the motion prior of the odometry.
 *
 *   model        constant body velocity ϖ = [v; ω] ∈ ℝ⁶ of the sensor
 *   per match    landmark p₁ in the sensor frame at the start of the scan,
 *                observed Δt_m later:   p̂₂ = exp((ϖ Δt_m)^∧) p₁
 *                (every match has its own Δt_m: its firing time)
 *   measurement  z = h(p) = [u, v, ρ]  (image column, image row, range)
 *   error        e_m = z_m − h(exp((ϖ Δt_m)^∧) p₁ₘ)
 *
 *   1. Sample 3 matches (3 × 3 = 9 equations for the 6 unknowns).
 *   2. Solve ϖ: linear start from p₂ − p₁ = Δt [I  −p₁^∧] ϖ with the measured
 *      point p₂, then Gauss-Newton on the whitened errors with
 *        H = F · [I  −p̂₂^∧] · J_l(ϖ Δt) · Δt,   F = ∂h/∂p
 *   3. Inliers: pixel error < inlier_px and range error < inlier_range.
 *   4. Keep the model with the most inliers (adaptive number of hypotheses),
 *      refine it on its inliers and find the inliers again.
 */
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "vtr_lidar/features/ouster_projector.hpp"

namespace vtr {
namespace lidar {

/// One descriptor match given to MD-RANSAC.
struct MdRansacMatch {
  /// Landmark in the sensor frame at the start of the scan
  Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
  /// Matched feature point, measured in the sensor frame at its own time
  Eigen::Vector3d p2 = Eigen::Vector3d::Zero();
  /// Time of the feature since the start of the scan [s]
  double dt = 0.0;
};

struct MdRansacParams {
  /// Hypotheses at most (adaptive: stops earlier at `confidence`)
  int max_iterations = 1000;
  /// Probability of having drawn at least one all-inlier sample
  double confidence = 0.999;
  /// A match is an inlier below both thresholds
  double inlier_px = 3.0;     ///< [px]
  double inlier_range = 0.5;  ///< [m]
  /// Minimum number of inliers to accept the model
  int min_inliers = 10;
  /// Gauss-Newton steps per hypothesis / refinement
  int gn_steps = 3;
  /// Refinement rounds on the inliers of the best model
  int refine_rounds = 2;
  /// Whitening of the errors in the solver: [px, px, m]
  double sigma_px = 1.2;
  double sigma_range = 0.03;
  /// Hypotheses beyond these are dropped (degenerate samples)
  double max_speed = 30.0;  ///< [m/s]
  double max_rate = 10.0;   ///< [rad/s]
  /// Seed of the sampler (fixed: repeatable runs)
  uint32_t seed = 42;
};

struct MdRansacResult {
  bool success = false;      ///< at least min_inliers inliers
  std::vector<int> inliers;  ///< indices into the matches
  Eigen::Matrix<double, 6, 1> omega =
      Eigen::Matrix<double, 6, 1>::Zero();  ///< sensor velocity [v; ω]
  int num_hypotheses = 0;
};

/**
 * \brief Measurement z = h(p) = [u, v, ρ] of a point in the sensor frame.
 * \return false if the point does not project into the image
 */
bool mdRansacMeasure(const OusterProjector& projector,
                     const Eigen::Vector3d& p, Eigen::Vector3d& z);

/**
 * \brief Run motion-distorted RANSAC on a set of matches.
 * \param matches    descriptor matches (landmark at scan start, feature)
 * \param projector  OusterProjector for the measurement model
 * \param params     see MdRansacParams
 */
MdRansacResult mdRansacSolve(const std::vector<MdRansacMatch>& matches,
                             const OusterProjector& projector,
                             const MdRansacParams& params);

}  // namespace lidar
}  // namespace vtr
