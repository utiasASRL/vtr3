/**
 * \file md_ransac.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Motion-distorted RANSAC implementation (full 6-DoF), see md_ransac.hpp.
 */
#include "vtr_lidar/features/md_ransac.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/Dense>
#include <lgmath.hpp>

namespace vtr {
namespace lidar {

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// A match prepared for the solver.
struct Observation {
  Eigen::Vector3d p1;  ///< landmark in the sensor frame at the scan start
  Eigen::Vector3d p2;  ///< measured feature point
  Eigen::Vector3d z2;  ///< z = h(p2) = [u, v, rho]
  double dt;           ///< time of the feature since the scan start [s]
};

/// e = z2 - h(exp((omega dt)^) p1), with the image column wrapped, and
/// optionally H (3 x 6) such that e(omega + d) = e - H d.
/// Returns false if the predicted point does not project into the image.
bool evalError(const OusterProjector& projector, const Observation& obs,
               const Vector6d& omega, Eigen::Vector3d& e,
               Eigen::Matrix<double, 3, 6>* H = nullptr) {
  const Vector6d xi = omega * obs.dt;
  const Eigen::Matrix4d T = lgmath::se3::vec2tran(xi);
  const Eigen::Vector3d g = T.block<3, 3>(0, 0) * obs.p1 + T.block<3, 1>(0, 3);

  Eigen::Vector3d z_hat;
  if (!mdRansacMeasure(projector, g, z_hat)) return false;

  e = obs.z2 - z_hat;
  // the image column is periodic
  const double width = static_cast<double>(projector.cols());
  e(0) -= width * std::round(e(0) / width);

  if (H != nullptr) {
    // d g / d xi = [I, -g^] J_l(xi),  d xi / d omega = dt
    Eigen::Matrix<double, 3, 6> g_odot;
    g_odot.leftCols<3>() = Eigen::Matrix3d::Identity();
    g_odot.rightCols<3>() = -lgmath::so3::hat(g);
    *H = projector.measurementJacobian(g) * g_odot *
         lgmath::se3::vec2jac(xi) * obs.dt;
  }
  return e.allFinite();
}

/// Linear start: p2 - p1 = dt [I, -p1^] omega (first-order expansion of
/// p2 = exp((omega dt)^) p1), with the measured point p2.
bool linearVelocity(const std::vector<Observation>& obs,
                    const std::vector<int>& idx, Vector6d& omega) {
  Matrix6d AtA = Matrix6d::Zero();
  Vector6d Atb = Vector6d::Zero();
  for (const int i : idx) {
    const auto& o = obs[i];
    Eigen::Matrix<double, 3, 6> A;
    A.leftCols<3>() = o.dt * Eigen::Matrix3d::Identity();
    A.rightCols<3>() = -o.dt * lgmath::so3::hat(o.p1);
    AtA += A.transpose() * A;
    Atb += A.transpose() * (o.p2 - o.p1);
  }
  AtA += (1e-9 * AtA.trace() + 1e-12) * Matrix6d::Identity();
  omega = AtA.ldlt().solve(Atb);
  return omega.allFinite();
}

/// Gauss-Newton on the whitened errors of the given matches.
bool solveVelocity(const OusterProjector& projector,
                   const std::vector<Observation>& obs,
                   const std::vector<int>& idx, const Eigen::Vector3d& sigma,
                   int num_steps, Vector6d& omega) {
  const Eigen::Vector3d inv_sigma = sigma.cwiseInverse();
  for (int step = 0; step < num_steps; ++step) {
    Matrix6d A = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    int used = 0;
    for (const int i : idx) {
      Eigen::Vector3d e;
      Eigen::Matrix<double, 3, 6> H;
      if (!evalError(projector, obs[i], omega, e, &H)) continue;
      e = inv_sigma.asDiagonal() * e;
      H = inv_sigma.asDiagonal() * H;
      A += H.transpose() * H;
      b += H.transpose() * e;
      ++used;
    }
    if (used < 3) return false;
    A += (1e-9 * A.trace() + 1e-12) * Matrix6d::Identity();
    omega += A.ldlt().solve(b);
    if (!omega.allFinite()) return false;
  }
  return true;
}

/// Matches whose pixel and range errors are both below the thresholds.
void findInliers(const OusterProjector& projector,
                 const std::vector<Observation>& obs, const Vector6d& omega,
                 double inlier_px, double inlier_range,
                 std::vector<int>& inliers) {
  inliers.clear();
  const double px2 = inlier_px * inlier_px;
  for (size_t i = 0; i < obs.size(); ++i) {
    Eigen::Vector3d e;
    if (!evalError(projector, obs[i], omega, e)) continue;
    if (e.head<2>().squaredNorm() < px2 && std::abs(e(2)) < inlier_range)
      inliers.push_back(static_cast<int>(i));
  }
}

}  // namespace

bool mdRansacMeasure(const OusterProjector& projector,
                     const Eigen::Vector3d& p, Eigen::Vector3d& z) {
  const double range = p.norm();
  if (!std::isfinite(range) || range < 1e-6) return false;
  Eigen::Vector2d uv;
  if (!projector.projectPoint(p, uv)) return false;
  z << uv.x(), uv.y(), range;
  return z.allFinite();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════════
MdRansacResult mdRansacSolve(const std::vector<MdRansacMatch>& matches,
                             const OusterProjector& projector,
                             const MdRansacParams& params) {
  MdRansacResult result;

  // matches that can be used: the measured point projects into the image
  std::vector<Observation> obs;
  std::vector<int> obs_to_match;
  obs.reserve(matches.size());
  obs_to_match.reserve(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    const auto& m = matches[i];
    Observation o;
    o.p1 = m.p1;
    o.p2 = m.p2;
    o.dt = m.dt;
    if (!o.p1.allFinite() || !std::isfinite(o.dt)) continue;
    if (!mdRansacMeasure(projector, o.p2, o.z2)) continue;
    obs.push_back(o);
    obs_to_match.push_back(static_cast<int>(i));
  }

  const int N = static_cast<int>(obs.size());
  if (N < std::max(3, params.min_inliers)) return result;

  const Eigen::Vector3d sigma(params.sigma_px, params.sigma_px,
                              params.sigma_range);

  std::mt19937 rng(params.seed);
  std::uniform_int_distribution<int> dist(0, N - 1);

  int best_count = 0;
  Vector6d best_omega = Vector6d::Zero();
  std::vector<int> best_inliers;
  std::vector<int> cand_inliers;
  std::vector<int> sample(3);

  int max_hyp = params.max_iterations;
  for (int iter = 0; iter < max_hyp; ++iter) {
    result.num_hypotheses++;

    // 3 distinct matches
    sample[0] = dist(rng);
    do { sample[1] = dist(rng); } while (sample[1] == sample[0]);
    do { sample[2] = dist(rng); } while (sample[2] == sample[0] || sample[2] == sample[1]);

    // velocity of this sample: linear start, then Gauss-Newton
    Vector6d omega;
    if (!linearVelocity(obs, sample, omega)) continue;
    if (!solveVelocity(projector, obs, sample, sigma, params.gn_steps, omega))
      continue;
    if (omega.head<3>().norm() > params.max_speed ||
        omega.tail<3>().norm() > params.max_rate)
      continue;

    findInliers(projector, obs, omega, params.inlier_px, params.inlier_range,
                cand_inliers);
    const int count = static_cast<int>(cand_inliers.size());
    if (count > best_count) {
      best_count = count;
      best_omega = omega;
      best_inliers = cand_inliers;
      // adaptive stop: enough hypotheses to have drawn an all-inlier sample
      // with the given confidence
      const double ratio =
          std::min(static_cast<double>(best_count) / N, 1.0 - 1e-9);
      const double needed = std::log(1.0 - params.confidence) /
                            std::log(1.0 - ratio * ratio * ratio);
      const int needed_i = std::isfinite(needed)
                               ? static_cast<int>(std::ceil(needed))
                               : params.max_iterations;
      max_hyp = std::min(params.max_iterations, std::max(needed_i, 20));
    }
  }

  if (best_count < 3) return result;

  // refine on the inliers, then find the inliers again
  for (int round = 0; round < params.refine_rounds; ++round) {
    if (best_inliers.size() < 3) break;
    Vector6d refined = best_omega;
    if (!solveVelocity(projector, obs, best_inliers, sigma, params.gn_steps,
                       refined))
      break;
    std::vector<int> refined_inliers;
    findInliers(projector, obs, refined, params.inlier_px,
                params.inlier_range, refined_inliers);
    if (refined_inliers.size() < best_inliers.size()) break;
    best_omega = refined;
    best_inliers = std::move(refined_inliers);
  }

  result.omega = best_omega;
  result.inliers.reserve(best_inliers.size());
  for (const int i : best_inliers) result.inliers.push_back(obs_to_match[i]);
  result.success =
      static_cast<int>(result.inliers.size()) >= params.min_inliers;
  return result;
}

}  // namespace lidar
}  // namespace vtr
