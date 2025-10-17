#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "steam.hpp"
#include "vtr_logging/logging.hpp"
#include <iostream>
#include <vector>

namespace vtr {

namespace lidar {

namespace daicp_lib_new {

inline int daicp_num_thread = 8;


// =================== Print Functions ===================
inline void printEigenvalues(const Eigen::VectorXd& eigenvalues, const std::string& label = "Eigenvalues") {
  CLOG(DEBUG, "lidar.localization_daicp") << label << ": [" << eigenvalues.transpose() << "]";
}

inline void printWellConditionedDirections(const Eigen::VectorXd& eigenvalues, double threshold) {
  std::string directions_str = "Well-conditioned directions: [";
  for (int i = 0; i < eigenvalues.size(); ++i) {
    directions_str += (eigenvalues(i) > threshold) ? "True" : "False";
    if (i < eigenvalues.size() - 1) directions_str += ", ";
  }
  directions_str += "]";
  CLOG(DEBUG, "lidar.localization_daicp") << directions_str;
}

inline void printCovarianceInfo(const Eigen::MatrixXd& daicp_cov) {
  const Eigen::VectorXd diagonal = daicp_cov.diagonal();
  const Eigen::VectorXd std_dev = diagonal.cwiseSqrt();
  
  CLOG(DEBUG, "lidar.localization_daicp") << "Final covariance P diagonal (roll, pitch, yaw, x, y, z): [" << diagonal.transpose() << "]";
  CLOG(DEBUG, "lidar.localization_daicp") << "Final std (roll, pitch, yaw, x, y, z): [" << std_dev.transpose() << "]";
}

// se3 math
inline lgmath::se3::Transformation 
paramsToTransformationMatrix(const Eigen::VectorXd& params) {
  if (params.size() != 6) {
    CLOG(ERROR, "lidar.localization_daicp") << "Invalid parameter size: " << params.size();
    return lgmath::se3::Transformation();
  }
  
  const double rx = params(0), ry = params(1), rz = params(2);
  const Eigen::Vector3d translation = params.tail<3>();
  
  // For very small angles, use linearized form for numerical stability
  if (std::max({std::abs(rx), std::abs(ry), std::abs(rz)}) < 0.1) {
    // Small angle approximation
    Eigen::Matrix3d R;
    R << 1, -rz, ry,
         rz, 1, -rx,
         -ry, rx, 1;
    
    // Ensure orthogonality using SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    
    // Build 4x4 transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = translation;
    
    return lgmath::se3::Transformation(T);
  } else {
    // Full rotation matrix computation for larger angles
    // Individual rotation matrices
    Eigen::Matrix3d Rx, Ry, Rz;
    Rx << 1, 0, 0,
          0, std::cos(rx), -std::sin(rx),
          0, std::sin(rx), std::cos(rx);
    
    Ry << std::cos(ry), 0, std::sin(ry),
          0, 1, 0,
          -std::sin(ry), 0, std::cos(ry);
    
    Rz << std::cos(rz), -std::sin(rz), 0,
          std::sin(rz), std::cos(rz), 0,
          0, 0, 1;
    
    // Combined rotation (ZYX order)
    Eigen::Matrix3d R = Rz * Ry * Rx;
    
    // Build 4x4 transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = translation;
    
    return lgmath::se3::Transformation(T);
  }
}

inline Eigen::VectorXd computeP2PlaneJacobian(
    const Eigen::Vector3d& source_point, 
    const Eigen::Vector3d& target_normal) {
  
  // Ensure normal is unit vector
  const Eigen::Vector3d n = target_normal.normalized();
  const Eigen::Vector3d ps = source_point;
  
  // Jacobian with respect to rotation parameters 
  const Eigen::Vector3d cross_x(0, -ps[2], ps[1]);   // ∂(R*p)/∂rx
  const Eigen::Vector3d cross_y(ps[2], 0, -ps[0]);   // ∂(R*p)/∂ry  
  const Eigen::Vector3d cross_z(-ps[1], ps[0], 0);   // ∂(R*p)/∂rz
  
  // Rotation Jacobian: dot product with normal
  Eigen::Vector3d rotation_jacobian;
  rotation_jacobian[0] = cross_x.dot(n);  // ∂e/∂rx
  rotation_jacobian[1] = cross_y.dot(n);  // ∂e/∂ry
  rotation_jacobian[2] = cross_z.dot(n);  // ∂e/∂rz
  
  // Translation Jacobian: normal vector
  const Eigen::Vector3d translation_jacobian = n;
  
  // Combine into 6D vector [rotation_jacobian, translation_jacobian]
  Eigen::VectorXd jacobian(6);
  jacobian.head<3>() = rotation_jacobian;
  jacobian.tail<3>() = translation_jacobian;
  
  return jacobian;
}

inline void constructWellConditionedDirections(
    const Eigen::VectorXd& eigenvalues,
    const Eigen::MatrixXd& eigenvectors,
    double eigenvalue_threshold,
    Eigen::MatrixXd& V,
    Eigen::MatrixXd& Vf,
    Eigen::VectorXd& eigen_vf,
    Eigen::MatrixXd& Vd) {
  
  const int n_dims = eigenvalues.size();
  
  // V is the full eigenvector matrix (each column is an eigenvector)
  V = eigenvectors;

  // Initialize Vf, Vd, eigen_vf as zeros
  Vf = Eigen::MatrixXd::Zero(n_dims, n_dims);
  Vd = Eigen::MatrixXd::Zero(n_dims, n_dims);
  eigen_vf = Eigen::VectorXd::Zero(6);

  // Debug logging for eigenvalues and threshold
  CLOG(DEBUG, "lidar.localization_daicp") << "Eigenvalues: [" << eigenvalues.transpose() << "]";
  CLOG(DEBUG, "lidar.localization_daicp") << "Eigenvalue threshold: " << eigenvalue_threshold;  

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

inline Eigen::VectorXd computeUpdateStep(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXd& Vf) {
  
  // Compute update in well-conditioned space: Δxf ← (A^T A)^{-1} A^T b
  const Eigen::MatrixXd AtA = A.transpose() * A;
  
  Eigen::VectorXd delta_x_f;
  try {
    // Use pseudoinverse for robustness
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(AtA, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Compute pseudoinverse manually with tolerance
    const double tolerance = 1e-14;
    Eigen::VectorXd singular_values = svd.singularValues();
    
    // Create diagonal matrix with inverse of non-zero singular values
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(singular_values.size(), singular_values.size());
    for (int i = 0; i < singular_values.size(); ++i) {
      if (singular_values(i) > tolerance) {
        S_inv(i, i) = 1.0 / singular_values(i);
      }
    }
    
    // Reconstruct pseudoinverse: AtA_pinv = V * S_inv * U^T
    Eigen::MatrixXd AtA_pinv = svd.matrixV() * S_inv * svd.matrixU().transpose();
    delta_x_f = AtA_pinv * A.transpose() * b;
    
  } catch (const std::exception& e) {
    // Fallback to least squares 
    // SVD failed, using least squares fallback
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    delta_x_f = svd.solve(b);
  }
  
  // Apply degeneracy-aware projection
  // delta_x_projected = V @ (Vf.T @ delta_x_f)
  const Eigen::VectorXd delta_x_projected = V * (Vf.transpose() * delta_x_f);
  
  return delta_x_projected;
}

inline bool computeEigenvalueDecomposition(
    const Eigen::MatrixXd& H,
    Eigen::VectorXd& eigenvalues,
    Eigen::MatrixXd& eigenvectors) {
  
  // Add regularization 
  const double reg_val = 1e-12;
  Eigen::MatrixXd H_reg = H + reg_val * Eigen::MatrixXd::Identity(H.rows(), H.cols());

  try {
    // Primary method: SelfAdjointEigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(H_reg);

    if (eigen_solver.info() != Eigen::Success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Eigenvalue decomposition failed, trying SVD fallback";

      // Fallback to SVD
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(H_reg, Eigen::ComputeFullU | Eigen::ComputeFullV);
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
    Eigen::MatrixXd sorted_eigenvectors(eigenvectors.rows(), eigenvectors.cols());
    
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

// NEW FUNCTION!
inline double computeThresholdVLG(const Eigen::VectorXd& eigenvalues) {
  // Apply natural log (with base e) - matching Python implementation
  Eigen::VectorXd log_eigenvalues = eigenvalues.array().max(1e-30).log();
  
  const double max_eigenval_full = log_eigenvalues.maxCoeff();
  const double eigenvalue_threshold = max_eigenval_full * 1e-3;  // Conservative threshold
  
  double decode_threshold = std::exp(eigenvalue_threshold);

  // [DEBUG]: magic number
  // decode_threshold = 350.0;

  // [DEBUG] ensure exactly which directions are well-conditioned
  decode_threshold = eigenvalues(1) - 0.0001; 

  CLOG(DEBUG, "lidar.localization_daicp") << "Log Threshold: " << eigenvalue_threshold;
  CLOG(DEBUG, "lidar.localization_daicp") << "Decode Threshold: " << decode_threshold;
  
  // return decode_threshold * (-1.0);  // disable daicp, should fall back to standard icp
  return decode_threshold; 
}

// =================== Block Scaling Functions ===================
inline Eigen::MatrixXd makePSD(const Eigen::MatrixXd& matrix, double min_eigenvalue = 1e-12) {
  // Make a matrix positive semidefinite by clamping negative eigenvalues
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(matrix);
  Eigen::VectorXd eigenvals = eigen_solver.eigenvalues();
  Eigen::MatrixXd eigenvecs = eigen_solver.eigenvectors();
  
  // Clamp negative eigenvalues
  eigenvals = eigenvals.cwiseMax(min_eigenvalue);
  
  return eigenvecs * eigenvals.asDiagonal() * eigenvecs.transpose();
}

inline std::pair<Eigen::Matrix3d, Eigen::Matrix3d> schurComplementMarginalization(const Eigen::MatrixXd& H) {
  // Apply Schur complement marginalization to obtain marginalized information matrices
  // H is 6x6: [H_theta_theta, H_theta_t; H_t_theta, H_tt]
  
  // Extract blocks
  Eigen::Matrix3d H_theta_theta = H.block<3, 3>(0, 0);  // rotation block
  Eigen::Matrix3d H_theta_t = H.block<3, 3>(0, 3);      // rotation-translation block
  Eigen::Matrix3d H_t_theta = H.block<3, 3>(3, 0);      // translation-rotation block
  Eigen::Matrix3d H_tt = H.block<3, 3>(3, 3);           // translation block
  
  const double reg_val = 1e-12;
  
  // Marginalized rotation information: H_marg_theta = H_theta_theta - H_theta_t * H_tt^{-1} * H_t_theta
  Eigen::Matrix3d H_marg_theta;
  try {
    Eigen::Matrix3d H_tt_inv = (H_tt + reg_val * Eigen::Matrix3d::Identity()).inverse();
    H_marg_theta = H_theta_theta - H_theta_t * H_tt_inv * H_t_theta;
  } catch (const std::exception& e) {
    // Use pseudo-inverse if singular
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_tt + reg_val * Eigen::Matrix3d::Identity(), 
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d H_tt_pinv = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
    H_marg_theta = H_theta_theta - H_theta_t * H_tt_pinv * H_t_theta;
  }
  
  // Marginalized translation information: H_marg_t = H_tt - H_t_theta * H_theta_theta^{-1} * H_theta_t
  Eigen::Matrix3d H_marg_t;
  try {
    Eigen::Matrix3d H_theta_theta_inv = (H_theta_theta + reg_val * Eigen::Matrix3d::Identity()).inverse();
    H_marg_t = H_tt - H_t_theta * H_theta_theta_inv * H_theta_t;
  } catch (const std::exception& e) {
    // Use pseudo-inverse if singular
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_theta_theta + reg_val * Eigen::Matrix3d::Identity(), 
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d H_theta_theta_pinv = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
    H_marg_t = H_tt - H_t_theta * H_theta_theta_pinv * H_theta_t;
  }
  
  // Ensure positive semidefinite
  H_marg_theta = makePSD(H_marg_theta);
  H_marg_t = makePSD(H_marg_t);
  
  return std::make_pair(H_marg_theta, H_marg_t);
}

inline double computeScalingFactorTrace(const Eigen::Matrix3d& H_marg_theta, const Eigen::Matrix3d& H_marg_t) {
  // Compute scaling factor using the trace method
  double tr_theta = H_marg_theta.trace();
  double tr_t = H_marg_t.trace();
  
  if (tr_t < 1e-12) {
    CLOG(WARNING, "lidar.localization_daicp") << "Translation trace is very small (" << tr_t << "), using default scaling";
    return 1.0;
  }
  
  return std::sqrt(tr_theta / tr_t);
}

inline double computeScalingFactorMax(const Eigen::Matrix3d& H_marg_theta, const Eigen::Matrix3d& H_marg_t) {

  // Compute scaling factor using the max eigenvalue method
  // regularization step
  const double reg_val = 1e-12;
  Eigen::Matrix3d H_marg_theta_reg = H_marg_theta + reg_val * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d H_marg_t_reg = H_marg_t + reg_val * Eigen::Matrix3d::Identity();
  
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_theta(H_marg_theta_reg);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_t(H_marg_t_reg);
  
  // Sort eigenvalues in descending order for consistent output
  Eigen::Vector3d eigvals_theta = solver_theta.eigenvalues();
  Eigen::Vector3d eigvals_t = solver_t.eigenvalues();
  
  std::sort(eigvals_theta.data(), eigvals_theta.data() + 3, std::greater<double>());
  std::sort(eigvals_t.data(), eigvals_t.data() + 3, std::greater<double>());
  
  double max_theta = eigvals_theta(0);
  double max_t = eigvals_t(0);
  
  // Debug print eigenvalues with same format as Python
  CLOG(DEBUG, "lidar.localization_daicp") << "Eigenvalues theta: [" << eigvals_theta.transpose() << "]";
  CLOG(DEBUG, "lidar.localization_daicp") << "Eigenvalues t: [" << eigvals_t.transpose() << "]";
  
  if (max_t < 1e-12) {
    CLOG(WARNING, "lidar.localization_daicp") << "Translation max eigenvalue is very small (" << max_t << "), using default scaling";
    return 1.0;
  }
  
  return std::sqrt(max_theta / max_t);
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
inline Eigen::MatrixXd computeDaicpCovariance(const Eigen::MatrixXd& H, const Eigen::MatrixXd& Vf, 
                                              const Eigen::VectorXd& eigen_vf, const Eigen::MatrixXd& Vd) {
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
  
  Eigen::MatrixXd daicpCov;
  if (Vd.cols() == 0) {
    // No degenerate directions
    daicpCov = Vf_reduced * eigen_vf_reduced.cwiseInverse().asDiagonal() * Vf_reduced.transpose();
  } else {
    // With degenerate directions
    // [NOTE] a small epsilon, i.e. 1e-6, will lead to very large values in degenerate directions,
    // we set epsilon to be 1e-1 or 1e-2 for covariance inflation.
    // Consider to use the prior covariance in degenerated directions. 
    const double epsilon = 0.1;  
    daicpCov = Vf_reduced * eigen_vf_reduced.cwiseInverse().asDiagonal() * Vf_reduced.transpose() +
              (1.0/epsilon) * (Vd *Vd.transpose());
  }
  
  return daicpCov;
}

// =================== Unified Computation Function ===================
inline void computeJacobianResidualInformation(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    const Eigen::Matrix4d& T_combined,
    const Eigen::MatrixXd& T_ms_prior,
    const Eigen::MatrixXd& Sigma_x,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b,
    Eigen::MatrixXd& W_inv,
    double& ell_mr) {

  const int n_points = static_cast<int>(sample_inds.size());
  
  // Initialize outputs
  A.resize(n_points, 6);
  b.resize(n_points);
  W_inv = Eigen::MatrixXd::Zero(n_points, n_points);
  
  // Lidar range and bearing noise model parameters (HARDCODED, move to config later)
  const double sigma_d = 0.02;                    // range noise std (2cm)
  const double sigma_az = M_PI / 180.0 * 0.03;    // azimuth noise std (0.03 degrees)
  const double sigma_el = M_PI / 180.0 * 0.03;    // elevation noise std (0.03 degrees)
  
  const Eigen::Matrix3d Sigma_du = Eigen::Vector3d(sigma_d * sigma_d, 
                                                   sigma_az * sigma_az, 
                                                   sigma_el * sigma_el).asDiagonal();
  
  // --- parallel accumulation for mean range ---
  double sum_range = 0.0;
  int valid_count  = 0;
  // using OpenMP reduction to safely accumulate sum_range and valid_count
  #pragma omp parallel for reduction(+:sum_range,valid_count) schedule(dynamic, 10) num_threads(daicp_num_thread)
  for (int i = 0; i < n_points; ++i) {
    const auto& ind = sample_inds[i];
    
    // Get original source point and transform it with combined transformation
    const Eigen::Vector3d source_pt_original = query_mat.block<3, 1>(0, ind.first).cast<double>();
    Eigen::Vector4d source_pt_hom;
    source_pt_hom << source_pt_original, 1.0;
    const Eigen::Vector3d source_pt_transformed = (T_combined * source_pt_hom).head<3>();

    
    // Get target point and normal
    const Eigen::Vector3d target_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
    const Eigen::Vector3d target_normal = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
    const Eigen::Vector3d n_t = target_normal.normalized();
    
    // ===== 1. Compute Jacobian and Residual =====
    const Eigen::VectorXd jacobian = computeP2PlaneJacobian(source_pt_transformed, n_t);
    A.row(i) = jacobian.transpose();
    
    // Compute residual (point-to-plane distance)
    b(i) = n_t.dot(target_pt - source_pt_transformed);
    
    // ===== 2. Compute Sigma_pL_s for this point =====
    // Get lidar scan point in lidar frame for noise modeling
    const Eigen::Vector3d p_lidar = (T_ms_prior.inverse() * source_pt_hom).head<3>();
    
    // Compute range, azimuth, elevation in lidar frame
    const double range = p_lidar.norm();
    const double azimuth = std::atan2(p_lidar(1), p_lidar(0));
    const double elevation = std::atan2(p_lidar(2), std::sqrt(p_lidar(0)*p_lidar(0) + p_lidar(1)*p_lidar(1)));
    
    // Compute covariance for this measurement
    const Eigen::Matrix3d Sigma_pL_i = computeRangeBearingCovariance(range, azimuth, elevation, Sigma_du);
    
    // accumulate range for ell_mr
    if (std::isfinite(range)) { sum_range += range; valid_count += 1; }

    // ===== 3. Compute Information Weight for this point =====
    // Compute Jacobians following the Python implementation
    // J_theta = -n_t^T @ skew_symmetric(source_pt_transformed)
    const Eigen::Matrix3d skew_p = skewSymmetric(source_pt_transformed);
    const Eigen::RowVector3d J_theta = -n_t.transpose() * skew_p;
    
    // J_t = n_t^T
    const Eigen::RowVector3d J_t = n_t.transpose();
    
    // J_pose = [J_theta, J_t] (1x6)
    Eigen::RowVectorXd J_pose(6);
    J_pose << J_theta, J_t;
    
    // J_p = n_t^T (1x3)
    const Eigen::RowVector3d J_p = n_t.transpose();
    

    // CLOG(DEBUG, "lidar.localization_daicp") << " ================ n_t: [" << n_t.transpose() << "] ================ ";
    // CLOG(DEBUG, "lidar.localization_daicp") << " ================ J_pose: [" << J_pose << "] ================ ";
    // CLOG(DEBUG, "lidar.localization_daicp") << " ================ J_p: [" << J_p << "] ================ ";
    // CLOG(DEBUG, "lidar.localization_daicp") << " ================ Sigma_pL_i: " << Sigma_pL_i.diagonal().transpose() << " ================ ";
    // CLOG(DEBUG, "lidar.localization_daicp") << " ================ Sigma_x: " << Sigma_x.diagonal().transpose() << " ================ ";

    // cov_r = J_pose @ Sigma_x @ J_pose^T + J_p @ Sigma_pL_i @ J_p^T
    const double cov_r = (J_pose * Sigma_x * J_pose.transpose())(0,0) + 
                        (J_p * Sigma_pL_i * J_p.transpose())(0,0);

    // Set information weight (inverse of covariance)
    if ((1.0 / cov_r) > 10000.0) {
      W_inv(i, i) = 10000.0;  // Cap the maximum weight
      CLOG(DEBUG, "lidar.localization_daicp") << " ================ W_inv is set to be 10000.0, error out  ================ ";
    } else {
      W_inv(i, i) = 1.0 / cov_r;
    }
  }

    // mean range -> scaling parameter
  const double mean_range = (valid_count > 0) ? (sum_range / valid_count) : 0.0;
  ell_mr = mean_range;  // simplest: 1 rad ~ ell_mr meters
}

// =================== DA-ICP with Block Scaling ===================
inline bool daGaussNewtonScaleP2Plane(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    steam::se3::SE3StateVar::Ptr T_var,
    const Eigen::MatrixXd& T_ms_prior,
    const Eigen::MatrixXd& Sigma_x,
    int max_gn_iter,
    double inner_tolerance,
    Eigen::MatrixXd& daicp_cov) {

  if (sample_inds.size() < 6) {
    CLOG(WARNING, "lidar.localization_daicp") << "Insufficient correspondences for Gauss-Newton";
    return false;
  }
  
  // Convergence parameters
  const double absolute_cost_threshold = 1e-12;         
  const double absolute_cost_change_threshold = 1e-8;   
  const double relative_cost_change_threshold = 1e-8;    
  const double zero_gradient_threshold = 1e-8;          
  const double parameter_change_threshold = inner_tolerance; 

  // Get initial transformation from T_var
  const lgmath::se3::Transformation initial_T_var = T_var->value();
  lgmath::se3::Transformation current_transformation = lgmath::se3::Transformation(); // Identity
  Eigen::VectorXd accumulated_params = Eigen::VectorXd::Zero(6);
  double curr_cost = 0.0;
  double prev_cost = std::numeric_limits<double>::max();
  bool converged = false;
  std::string termination_reason = "";

  // Gauss-Newton iterations with block scaling
  for (int gn_iter = 0; gn_iter < max_gn_iter && !converged; ++gn_iter) {

    // Compose with initial transformation: final_T = current_T * initial_T
    const Eigen::Matrix4d T_combined = current_transformation.matrix() * initial_T_var.matrix();
    
    // ---- Unified computation: 
    // ---- Jacobian, residual, Sigma_pL_s, and information matrix 
    Eigen::MatrixXd A, W_inv;
    Eigen::VectorXd b;
    double ell_mr = 0.0;  // scaling factor using mean point range \bar{r}

    computeJacobianResidualInformation(sample_inds, query_mat, map_mat, map_normals_mat,
                                       T_combined, T_ms_prior, Sigma_x, A, b, W_inv, ell_mr);

    // --------- [DEBUG]: disable weighting
    W_inv = Eigen::MatrixXd::Identity(W_inv.rows(), W_inv.cols());  // DEBUG: set to identity to disable weighting

    // STEAM-style convergence checking
    // Compute current weighted cost (0.5 * b^T * W_inv * b) and weighted gradient norm
    curr_cost = 0.5 * b.transpose() * W_inv * b;
    const Eigen::VectorXd weighted_gradient = A.transpose() * W_inv * b;
    const double grad_norm = weighted_gradient.norm();

    // 1. Check absolute cost threshold
    if (curr_cost <= absolute_cost_threshold) {
      converged = true;
      termination_reason = "CONVERGED_ABSOLUTE_COST";
    }
    // 2. Check absolute cost change (after first iteration)  
    else if (gn_iter > 0 && std::abs(prev_cost - curr_cost) <= absolute_cost_change_threshold) {
      converged = true;
      termination_reason = "CONVERGED_ABSOLUTE_COST_CHANGE";
    }
    // 3. Check relative cost change (after first iteration)
    else if (gn_iter > 0 && prev_cost > 0 && 
              std::abs(prev_cost - curr_cost) / prev_cost <= relative_cost_change_threshold) {
      converged = true;
      termination_reason = "CONVERGED_RELATIVE_COST_CHANGE";
    }
    // 4. Check zero gradient
    else if (grad_norm < zero_gradient_threshold) {
      converged = true;
      termination_reason = "CONVERGED_ZERO_GRADIENT";
    }
    
    if (converged) {
      CLOG(DEBUG, "lidar.localization_daicp") << "Converged after " << (gn_iter + 1) << " iterations: " << termination_reason;
      break;
    }

    // ---------- perform scaled GN update, solving for the same problem.
    // compute original Hessian
    Eigen::MatrixXd H_original = A.transpose() * W_inv * A;
    
    // Apply Schur complement marginalization
    auto [H_marg_theta, H_marg_t] = schurComplementMarginalization(H_original);
    
    // Compute scaling factor
    double ell = computeScalingFactorMax(H_marg_theta, H_marg_t);

    CLOG(DEBUG, "lidar.localization_daicp") << "-------------- Scaling factor ell: " << ell << " --------------";
    CLOG(DEBUG, "lidar.localization_daicp") << "-------------- Scaling factor ell_mr: " << ell_mr << " --------------";
    
    // --- [DEBUG]
    // ell_mr = 1.0;  // Disable scaling for debugging

    // Construct inverse block scaling matrix: D_inv
    Eigen::MatrixXd D_inv = Eigen::MatrixXd::Identity(6, 6);
    D_inv.block<3, 3>(0, 0) *= (1.0 / ell_mr);  // rotation scaling, use the mean range distance instead. 
    // translation scaling remains 1.0
    
    // Scale the jacobian
    Eigen::MatrixXd A_scaled = A * D_inv;
    
    // Degeneracy analysis in eigenspace
    Eigen::MatrixXd H_scaled = A_scaled.transpose() * W_inv * A_scaled;
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenvectors;
    bool eigen_success = computeEigenvalueDecomposition(H_scaled, eigenvalues, eigenvectors);
    
    if (!eigen_success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Insufficient correspondences for Gauss-Newton";
      return false;
    }
    
    // Compute threshold using VLG method and construct well-conditioned directions
    const double eigenvalue_threshold = computeThresholdVLG(eigenvalues);
    Eigen::MatrixXd V, Vf, Vd;
    Eigen::VectorXd eigen_vf;
    constructWellConditionedDirections(eigenvalues, eigenvectors, eigenvalue_threshold, 
                                       V, Vf, eigen_vf, Vd);
    
    // Compute update step using eigenspace projection
    Eigen::VectorXd delta_params_scaled = computeUpdateStep(A_scaled, b, V, Vf);
    // Compute the scaled covariance matrix
    Eigen::MatrixXd daicp_cov_scaled = computeDaicpCovariance(H_scaled, Vf, eigen_vf, Vd);
    
    // Unscale the parameters and covariance
    Eigen::VectorXd delta_params = D_inv * delta_params_scaled;
    daicp_cov = D_inv * daicp_cov_scaled * D_inv.transpose();
    // --- Print covariance information
    printCovarianceInfo(daicp_cov);
    
    // Accumulate parameters and update transformation
    accumulated_params += delta_params;
    current_transformation = paramsToTransformationMatrix(accumulated_params);
    // Check convergence (but allow at least one iteration to see progress)
    if (gn_iter > 0 && delta_params.norm() < parameter_change_threshold) {
      converged = true;
      termination_reason = "CONVERGED_PARAMETER_CHANGE";
      break;
    }
    // Update cost for next iteration
    prev_cost = curr_cost;
  }
  
  // Check if terminated due to max iterations
  if (!converged) {
    // Maximum Gauss-Newton iterations reached
    termination_reason = "MAX_ITERATIONS";
  }
    
  // Update T_var with the final transformation
  const lgmath::se3::Transformation final_transformation = lgmath::se3::Transformation(
      static_cast<Eigen::Matrix4d>(current_transformation.matrix() * initial_T_var.matrix())
  );
  
  // Calculate the actual delta for STEAM update
  Eigen::Matrix4d delta_for_steam = final_transformation.matrix() * initial_T_var.matrix().inverse();
  Eigen::Matrix<double, 6, 1> delta_vec_steam = lgmath::se3::tran2vec(delta_for_steam);
  
  // Apply the update to T_var using STEAM's update mechanism
  T_var->update(delta_vec_steam);
  
  return true;
}

}  // daicp_lib_new

}  // namespace lidar
}  // namespace vtr