#pragma once

// General DA-ICP library: functions that are identical across all DA-ICP
// methods (HybridQP, Hybrid, P2Plane). Method-specific functions live in
// da_hybridqp_lib.hpp, da_hybrid_lib.hpp and da_p2plane_lib.hpp, in nested
// namespaces da_lib::hybridqp, da_lib::hybrid and da_lib::p2plane.

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "steam.hpp"
#include "vtr_logging/logging.hpp"
#include <iostream>
#include <vector>

namespace vtr {
namespace lidar {

class LocalizationDAICPModule;

namespace da_lib {

// =================== Print Functions ===================
inline void printEigenvalues(const Eigen::VectorXd& eigenvalues,
                             const std::string& label = "Eigenvalues") {
  CLOG(DEBUG, "lidar.localization_daicp") << label << ": [" << eigenvalues.transpose() << "]";
}

inline void printWellConditionedDirections(const Eigen::VectorXd& eigenvalues,
                                           double threshold) {
  std::string directions_str = "Well-conditioned directions: [";
  for (int i = 0; i < eigenvalues.size(); ++i) {
    directions_str += (eigenvalues(i) > threshold) ? "True" : "False";
    if (i < eigenvalues.size() - 1) directions_str += ", ";
  }
  directions_str += "]";
  CLOG(DEBUG, "lidar.localization_daicp") << directions_str;
}

// =================== Range/Bearing Noise Model Utilities ===================
inline Eigen::Vector3d omegaFromAzEl(double az, double el) {
  const double c_az = std::cos(az), s_az = std::sin(az);
  const double c_el = std::cos(el), s_el = std::sin(el);
  return Eigen::Vector3d(c_el * c_az, c_el * s_az, s_el);
}

inline Eigen::Matrix<double, 3, 2> NFromAzEl(double az, double el) {
  const double c_az = std::cos(az), s_az = std::sin(az);
  const double c_el = std::cos(el), s_el = std::sin(el);

  Eigen::Matrix<double, 3, 2> N;
  N << -c_el * s_az, -s_el * c_az,
        c_el * c_az, -s_el * s_az,
        0.0,          c_el;
  return N;
}

inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
  Eigen::Matrix3d skew;
  skew <<     0, -v(2),  v(1),
           v(2),     0, -v(0),
          -v(1),  v(0),     0;
  return skew;
}

inline Eigen::Matrix3d computeRangeBearingCovariance(double d, double az, double el,
                                                    const Eigen::Matrix3d& Sigma_du) {
  const Eigen::Vector3d w = omegaFromAzEl(az, el);
  const Eigen::Matrix<double, 3, 2> N = NFromAzEl(az, el);

  // JA = [w, -d * [w]× * N] where [w]× is skew-symmetric matrix of w
  Eigen::Matrix3d JA;
  JA.col(0) = w;
  JA.block<3, 2>(0, 1) = -d * skewSymmetric(w) * N;

  return JA * Sigma_du * JA.transpose();
}

// =================== Covariance Computation ===================
// Prior-proportional covariance: degenerate directions get sigma_i^2 = alpha * v_i^T Sigma_prior v_i.
// `prior_cov` MUST be expressed in the same coordinate frame (ordering AND scaling)
// as Vf/Vd. Caller is responsible.
inline Eigen::Matrix<double, 6, 6> computeDaicpCovariance(
                                              const Eigen::Matrix<double, 6, 6>& Vf,
                                              const Eigen::VectorXd& eigen_vf,
                                              const Eigen::Matrix<double, 6, Eigen::Dynamic>& Vd,
                                              const Eigen::Matrix<double, 6, 6>& prior_cov,
                                              double degenerate_cov_alpha) {
  // Apply solution remapping + regularization in covariance matrix

  // Find non-zero columns in Vf
  std::vector<int> valid_cols;
  for (int i = 0; i < Vf.cols(); ++i) {
    if (Vf.col(i).norm() > 1e-12) {
      valid_cols.push_back(i);
    }
  }

  // Extract non-zero parts
  Eigen::MatrixXd Vf_reduced(Vf.rows(), valid_cols.size());
  Eigen::VectorXd eigen_vf_reduced(valid_cols.size());
  for (size_t i = 0; i < valid_cols.size(); ++i) {
    Vf_reduced.col(i) = Vf.col(valid_cols[i]);
    eigen_vf_reduced(i) = eigen_vf(valid_cols[i]);
  }

  Eigen::Matrix<double, 6, 6> daicpCov;
  if ((Vf_reduced.cols() == 6) && (Vd.cols() == 0)) {
    // No degenerate directions
    daicpCov = Vf_reduced * eigen_vf_reduced.cwiseInverse().asDiagonal() * Vf_reduced.transpose();
  } else {
    // Covariance in degenerate directions is set proportional to the prior:
    //   sigma_i^2 = alpha * v_i^T * Sigma_prior * v_i        (alpha >> 1)
    // so STEAM's joint posterior gives the lidar weight 1/(1+alpha) along v_i.
    // This is scale-invariant in the prior (large or small) and converges to
    // the rank-deficient (Bayesian-correct) solution as alpha -> infinity.
    //
    // Floor on the projected prior variance: prevents a degenerate direction
    // collapsing if the prior happens to be tiny in that direction (which
    // would re-introduce the old "fictitious lidar information" failure mode).
    constexpr double kMinPriorVar = 1e3;
    daicpCov = Vf_reduced * eigen_vf_reduced.cwiseInverse().asDiagonal() * Vf_reduced.transpose();
    for (int i = 0; i < Vd.cols(); ++i) {
      const Eigen::Matrix<double, 6, 1> v = Vd.col(i);
      double prior_var = v.dot(prior_cov * v);
      if (!std::isfinite(prior_var) || prior_var < kMinPriorVar) prior_var = kMinPriorVar;
      const double sigma2_i = degenerate_cov_alpha * prior_var;
      daicpCov.noalias() += sigma2_i * (v * v.transpose());
    }
  }

  return daicpCov;
}

inline Eigen::Matrix<double, 6, 6> computeDaicpCovarianceDefault(
                                  const Eigen::Matrix<double, 6, 6>& Vf,
                                  const Eigen::VectorXd& eigen_vf,
                                  const Eigen::Matrix<double, 6, Eigen::Dynamic>& Vd) {
  // Apply solution remapping + regularization in covariance matrix

  // Find non-zero columns in Vf
  std::vector<int> valid_cols;
  for (int i = 0; i < Vf.cols(); ++i) {
    if (Vf.col(i).norm() > 1e-12) {
      valid_cols.push_back(i);
    }
  }

  // Extract non-zero parts
  Eigen::MatrixXd Vf_reduced(Vf.rows(), valid_cols.size());
  Eigen::VectorXd eigen_vf_reduced(valid_cols.size());
  for (size_t i = 0; i < valid_cols.size(); ++i) {
    Vf_reduced.col(i) = Vf.col(valid_cols[i]);
    eigen_vf_reduced(i) = eigen_vf(valid_cols[i]);
  }

  Eigen::Matrix<double, 6, 6> daicpCov;
  if ((Vf_reduced.cols() == 6) && (Vd.cols() == 0)) {
    // No degenerate directions
    daicpCov = Vf_reduced * eigen_vf_reduced.cwiseInverse().asDiagonal() * Vf_reduced.transpose();
  } else {
    CLOG(DEBUG, "lidar.localization_daicp") << Vf_reduced.cols() << " non-degenerate directions and "
                                           << Vd.cols() << " degenerate directions.";
    // With degenerate directions
    // [NOTE] a small epsilon, i.e. 1e-6, will lead to very large values in degenerate directions,
    // we set epsilon to be 1e-1 or 1e-2 for covariance inflation.
    // Consider to use the prior covariance in degenerated directions.
    const double epsilon = 1e-3;
    daicpCov = Vf_reduced * eigen_vf_reduced.cwiseInverse().asDiagonal() * Vf_reduced.transpose() +
              (1.0/epsilon) * (Vd *Vd.transpose());
  }

  return daicpCov;
}

// =================== Degeneracy Analysis ===================
inline void constructWellConditionedDirections(
    const Eigen::VectorXd& eigenvalues,
    const Eigen::Matrix<double, 6, 6>& eigenvectors,
    double eigenvalue_threshold,
    Eigen::Matrix<double, 6, 6>& V,
    Eigen::Matrix<double, 6, 6>& Vf,
    Eigen::VectorXd& eigen_vf,
    Eigen::Matrix<double, 6, Eigen::Dynamic>& Vd) {

  const int n_dims = eigenvalues.size();

  // V is the full eigenvector matrix (each column is an eigenvector)
  V = eigenvectors;

  // Initialize Vf, Vd, eigen_vf as zeros
  Vf.setZero();
  Vd = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_dims);
  eigen_vf.setZero(n_dims);


  // // Debug logging for eigenvalues and threshold
  // CLOG(DEBUG, "lidar.localization_daicp") << "Eigenvalues: [" << eigenvalues.transpose() << "]";
  // CLOG(DEBUG, "lidar.localization_daicp") << "Eigenvalue threshold: " << eigenvalue_threshold;

  // Find well-conditioned directions using the threshold
  std::vector<bool> well_conditioned_mask(n_dims);
  // int num_well_conditioned = 0;
  int deg_count = 0;
  for (int i = 0; i < n_dims; ++i) {
    well_conditioned_mask[i] = eigenvalues[i] > eigenvalue_threshold;
    if (well_conditioned_mask[i]) {
      Vf.col(i) = V.col(i);
      eigen_vf[i] = eigenvalues[i];
      // num_well_conditioned++;
    }
    else {
      Vd.col(deg_count) = V.col(i);
      deg_count++;
    }
  }
  Vd.conservativeResize(6, deg_count);

  // Print well-conditioned directions with color coding
  printWellConditionedDirections(eigenvalues, eigenvalue_threshold);
}

inline bool computeEigenvalueDecomposition(
    const Eigen::Matrix<double, 6, 6>& H,
    Eigen::VectorXd& eigenvalues,
    Eigen::Matrix<double, 6, 6>& eigenvectors) {

  // Add regularization
  const double reg_val = 1e-12;
  Eigen::Matrix<double, 6, 6> H_reg = H + reg_val * Eigen::Matrix<double, 6, 6>::Identity(H.rows(), H.cols());

  try {
    // Primary method: SelfAdjointEigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(H_reg);

    if (eigen_solver.info() != Eigen::Success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Eigenvalue decomposition failed, trying SVD fallback";

      // Fallback to SVD
      Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(H_reg, Eigen::ComputeFullU | Eigen::ComputeFullV);
      eigenvalues = svd.singularValues();
      eigenvectors = svd.matrixU();

      // Threshold tiny singular values
      const double eigenvalue_threshold = 1e-10;
      for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < eigenvalue_threshold) {
          eigenvalues(i) = 0.0;
        }
      }

      // SVD fallback successful
      return true;
    }

    eigenvalues = eigen_solver.eigenvalues();
    eigenvectors = eigen_solver.eigenvectors();

    // Sort eigenvalues in descending order
    std::vector<std::pair<double, int>> eigen_pairs;
    for (int i = 0; i < eigenvalues.size(); ++i) {
      eigen_pairs.push_back(std::make_pair(eigenvalues(i), i));
    }
    std::sort(eigen_pairs.begin(), eigen_pairs.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    Eigen::VectorXd sorted_eigenvalues(eigenvalues.size());
    Eigen::Matrix<double, 6, 6> sorted_eigenvectors(eigenvectors.rows(), eigenvectors.cols());

    for (int i = 0; i < eigenvalues.size(); ++i) {
      sorted_eigenvalues(i) = eigen_pairs[i].first;
      sorted_eigenvectors.col(i) = eigenvectors.col(eigen_pairs[i].second);
    }

    eigenvalues = sorted_eigenvalues;
    eigenvectors = sorted_eigenvectors;

    // Threshold tiny eigenvalues
    const double eigenvalue_threshold = 1e-10;
    for (int i = 0; i < eigenvalues.size(); ++i) {
      if (eigenvalues(i) < eigenvalue_threshold) {
        eigenvalues(i) = 0.0;
      }
    }

    // Debug logging to verify eigenvalues
    // CLOG(DEBUG, "lidar.localization_daicp") << "Eigenvalues (descending): [" << eigenvalues.transpose() << "]";

    return true;

  } catch (const std::exception& e) {
    CLOG(ERROR, "lidar.localization_daicp") << "Exception in eigenvalue decomposition: " << e.what();
    return false;
  }
}

inline double computeThreshold(const Eigen::VectorXd& eigenvalues,
                               const double cond_num_thresh_ratio) {

  const double max_eigenval = eigenvalues.maxCoeff();

  // ----- Compute threshold based on condition number ratio
  // A direction is well-conditioned if: max_eigenval / eigenval < cond_num_thresh_ratio
  // Rearranging: eigenval > max_eigenval / cond_num_thresh_ratio
  const double eigenvalue_threshold = max_eigenval / cond_num_thresh_ratio;

  CLOG(DEBUG, "lidar.localization_daicp") << "Relative Condition Number Threshold: " << cond_num_thresh_ratio;

  // const double eigenvalue_threshold = -1000.0;       // [DEBUG] default back to point-to-plane icp

  // Print relative condition numbers
  for (int i = 0; i < eigenvalues.size(); ++i) {
    double cond_num = (eigenvalues(i) > 1e-15) ? (max_eigenval / eigenvalues(i)) : std::numeric_limits<double>::infinity();
    CLOG(DEBUG, "lidar.localization_daicp") << "Condition number [" << i << "]: " << cond_num;
  }

  return eigenvalue_threshold;
}

}  // namespace da_lib
}  // namespace lidar
}  // namespace vtr
