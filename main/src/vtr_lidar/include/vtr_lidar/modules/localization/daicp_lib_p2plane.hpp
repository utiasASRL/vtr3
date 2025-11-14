#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "steam.hpp"
#include "vtr_logging/logging.hpp"
#include <iostream>
#include <vector>

namespace vtr {
namespace lidar {

class LocalizationDAICPModule;

namespace daicp_lib_p2plane {

inline int daicp_num_thread = 8;

// =================== Print Functions ===================

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

inline Eigen::Matrix<double, 6, 6> computeP2PlaneCovariance(const Eigen::Matrix<double, 6, 6>& Hessian) {
  constexpr double kMinEigenvalue = 1e-8;
  Eigen::Matrix<double, 6, 6> H_sym = 0.5 * (Hessian + Hessian.transpose());  // guard symmetry

  Eigen::Matrix<double, 6, 6> p2plane_cov;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(H_sym);
  if (ldlt.info() == Eigen::Success) {
    Eigen::Matrix<double, 6, 1> diag = ldlt.vectorD();
    for (int i = 0; i < diag.size(); ++i) {
      diag(i) = std::max(diag(i), kMinEigenvalue);
    }
    Eigen::Matrix<double, 6, 6> P = ldlt.transpositionsP() * Eigen::Matrix<double, 6, 6>::Identity();
    p2plane_cov = P * diag.cwiseInverse().asDiagonal() * P.transpose();
  } else {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(H_sym);
    if (eig.info() == Eigen::Success) {
      Eigen::VectorXd eigenvalues = eig.eigenvalues();
      for (int i = 0; i < eigenvalues.size(); ++i) {
        eigenvalues(i) = std::max(eigenvalues(i), kMinEigenvalue);
      }
      p2plane_cov =
          eig.eigenvectors() * eigenvalues.cwiseInverse().asDiagonal() * eig.eigenvectors().transpose();
    } else {
      p2plane_cov = Eigen::Matrix<double, 6, 6>::Identity() / kMinEigenvalue;
      CLOG(WARNING, "lidar.localization_daicp") << "Fell back to inflated identity covariance";
    }
  }
  return p2plane_cov;
}

// =================== point-to-plane Jacobian Computation ===================
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

// =================== Unified Computation Function ===================
inline void computeJacobianResidualInformation(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    const Eigen::Matrix4d& T_combined,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b,
    Eigen::MatrixXd& W_inv,
    double& ell_mr,
    const std::shared_ptr<const vtr::lidar::LocalizationDAICPModule::Config>& config_) {

  const int n_points = static_cast<int>(sample_inds.size());
  
  // Initialize outputs
  A.resize(n_points, 6);
  b.resize(n_points);
  W_inv = Eigen::MatrixXd::Zero(n_points, n_points);
  
  // Lidar range and bearing noise model parameters
  const double sigma_d = config_->sigma_d;          // range noise std (2cm)
  const double sigma_az = config_->sigma_az;        // azimuth noise std (0.03 degrees)
  const double sigma_el = config_->sigma_el;        // elevation noise std (0.03 degrees)

  const Eigen::Matrix3d Sigma_du = Eigen::Vector3d(sigma_d * sigma_d,
                                                   sigma_az * sigma_az,
                                                   sigma_el * sigma_el).asDiagonal();
  
  // --- parallel accumulation for mean range ---
  double sum_range = 0.0;
  int valid_count  = 0;
  // using OpenMP reduction to safely accumulate sum_range and valid_count
  #pragma omp parallel for reduction(+:sum_range,valid_count) schedule(static) num_threads(daicp_num_thread)
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
    const Eigen::Vector3d p_lidar = source_pt_original;
    
    // Compute range, azimuth, elevation in lidar frame
    const double range = p_lidar.norm();
    // const double flat_range = std::sqrt(p_lidar(0)*p_lidar(0) + p_lidar(1)*p_lidar(1));
    const double azimuth = std::atan2(p_lidar(1), p_lidar(0));
    const double elevation = std::atan2(p_lidar(2), std::sqrt(p_lidar(0)*p_lidar(0) + p_lidar(1)*p_lidar(1)));
    
    // Compute covariance for this measurement
    const Eigen::Matrix3d Sigma_pL_i = computeRangeBearingCovariance(range, azimuth, elevation, Sigma_du);
    
    // accumulate range for ell_mr
    if (std::isfinite(range)) { sum_range += range; valid_count += 1; }

    // ===== 3. Compute Information Weight for this point =====
    // Jacobians for variance terms:
    const Eigen::Matrix3d R_combined = T_combined.block<3,3>(0,0); // Rotation part of T_combined
    // J_p = n^T R   (since residual uses R p + t)
    const Eigen::RowVector3d J_p = (n_t.transpose() * R_combined);      // 1x3
    // J_q = -n^T
    const Eigen::RowVector3d J_q = (-n_t.transpose());                   // 1x3
    // J_n = (R p + t - q)^T
    const Eigen::Vector3d d = source_pt_transformed - target_pt;
    const Eigen::RowVector3d J_n = d.transpose();                        // 1x3

    // --- Map-point covariance  ---
    const double sigma_map = 0.01;  // meters
    const Eigen::Matrix3d Sigma_q_i = (sigma_map * sigma_map) * Eigen::Matrix3d::Identity();

    // --- Normal covariance (tangent-plane, PSD) ---
    // Uncertainty only in directions orthogonal to n_t.
    const double sigma_n = 0.03; // e.g., a small angle in radians mapped to length via ||d||
                                // typical: normal_sigma ~ 0.02 to 0.05 (tuned)
    const Eigen::Matrix3d Pn = Eigen::Matrix3d::Identity() - n_t * n_t.transpose();
    const Eigen::Matrix3d Sigma_n_i = (sigma_n * sigma_n) * Pn;

    // --- Transform LiDAR point covariance to the target/map frame ---
    // Sigma_p (world) = R * Sigma_p (lidar) * R^T   (R: lidar->world here equals R_combined if p_lidar was in lidar frame)
    const Eigen::Matrix3d Sigma_p_world = R_combined * Sigma_pL_i * R_combined.transpose();

    // --- Scalar residual variance ---
    double cov_r = 0.0;
    cov_r += (J_p * Sigma_p_world * J_p.transpose())(0,0); // source point noise
    cov_r += (J_q * Sigma_q_i    * J_q.transpose())(0,0);  // map point noise
    cov_r += (J_n * Sigma_n_i    * J_n.transpose())(0,0);  // normal noise

    // --- Physical variance floor (prevents ill-conditioning / overconfidence) ---
    // Use sensor floor and model floor; pick something tied to your setup.
    // const double sigma_floor = std::max({ 1.0e-3, 0.25 * voxel_size }); // meters
    const double sigma_floor = 0.01;  // meters
    const double var_floor   = sigma_floor * sigma_floor;
    cov_r = std::max(cov_r, var_floor);

    double w = 1.0 / cov_r;

    // CLOG(DEBUG, "lidar.localization_daicp") << "---------------------- Cov_r: " << cov_r;
    // CLOG(DEBUG, "lidar.localization_daicp") << "---------------------- Weight: " << w;

    const double w_cap = 1.0e6;  // weight cap
    if (w > w_cap) {
      w = w_cap;
      CLOG(DEBUG, "lidar.localization_daicp") << "W_inv capped at " << w_cap;
    }

    // Debug
    // CLOG(DEBUG, "lidar.localization_daicp") << "cov_r=" << cov_r
    //     << "  | JpΣpJp^T=" << (J_p * Sigma_p_world * J_p.transpose())(0,0)
    //     << "  | JqΣqJq^T=" << (J_q * Sigma_q_i    * J_q.transpose())(0,0)
    //     << "  | JnΣnJn^T=" << (J_n * Sigma_n_i    * J_n.transpose())(0,0);

    // Store weight (diagonal of W_inv)
    W_inv(i, i) = w;
  }

  // mean flat range -> scaling parameter
  const double mean_range = (valid_count > 0) ? (sum_range / valid_count) : 0.0;
  ell_mr = mean_range;  // simplest: 1 rad ~ ell_mr meters
}

inline Eigen::VectorXd computeSimpleUpdateStep(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b) {
  
  // Compute update in well-conditioned space: Δxf ← (A^T A)^{-1} A^T b
  // [Note] b does not need to be scaled. 
  // Since A is the scaled A, we are solving the (A^T A) dx = A^T b.
  // The right-hand side is scaled by A directly.
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
  
  return delta_x_f;
}

inline bool GaussNewtonP2Plane(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    steam::se3::SE3StateVar::Ptr T_var,
    const std::shared_ptr<const vtr::lidar::LocalizationDAICPModule::Config>& config_,
    Eigen::Matrix<double, 6, 6>& daicp_cov) {
  
  if (sample_inds.size() < 6) {
    CLOG(WARNING, "lidar.localization_daicp") << "Insufficient correspondences for Gauss-Newton";
    return false;
  }

  // Start with identity transformation for the Gauss-Newton process
  lgmath::se3::Transformation current_transformation = lgmath::se3::Transformation(); // Identity
  Eigen::VectorXd accumulated_params = Eigen::VectorXd::Zero(6);
  
  // Get initial transformation from T_var to apply later
  const lgmath::se3::Transformation initial_T_var = T_var->value();
  
  // Variables for convergence tracking
  double prev_cost = std::numeric_limits<double>::max();
  double curr_cost = 0.0;
  bool converged = false;
  std::string termination_reason = "";
  
  // Inner loop Gauss-Newton iterations with DEGENERACY-AWARE updates
  for (int gn_iter = 0; gn_iter < config_->max_gn_iter && !converged; ++gn_iter) {
    // Build jacobian and residual for current transformation
    Eigen::MatrixXd A(sample_inds.size(), 6);
    Eigen::MatrixXd W_inv(sample_inds.size(), sample_inds.size());
    Eigen::VectorXd b(sample_inds.size());
    
    // Compose with initial transformation: final_T = current_T * initial_T
    const Eigen::Matrix4d T_combined = current_transformation.matrix() * initial_T_var.matrix();
    
    double ell_mr = 0.0;

    // Compute Jacobian, residuals, and information matrix
    // Note: Measurement noise is computed from the original sensor measurements (query_mat),
    // which are independent of the state estimate, as it should be.
    computeJacobianResidualInformation(sample_inds, query_mat, map_mat, map_normals_mat,
                                       T_combined, A, b, W_inv, ell_mr, config_);

    // --------- [DEBUG]: disable weighting
    // W_inv = Eigen::MatrixXd::Identity(W_inv.rows(), W_inv.cols());
    // --- [DEBUG]
    // ell_mr = 1.0;  // Disable scaling for debugging

    // CLOG(DEBUG, "lidar.localization_daicp") << "---------------------- ell_mr:   " << ell_mr;

    // Compute current weighted cost (0.5 * b^T * W_inv * b) and weighted gradient norm
    curr_cost = 0.5 * b.transpose() * W_inv * b;
    const Eigen::VectorXd weighted_gradient = A.transpose() * W_inv * b;
    const double grad_norm = weighted_gradient.norm();
    
    // STEAM-style convergence checking
    // 1. Check absolute cost threshold
    if (curr_cost <= config_->abs_cost_thresh) {
      converged = true;
      termination_reason = "CONVERGED_ABSOLUTE_COST";
    }
    // 2. Check absolute cost change (after first iteration)
    else if (gn_iter > 0 && std::abs(prev_cost - curr_cost) <= config_->abs_cost_change_thresh) {
      converged = true;
      termination_reason = "CONVERGED_ABSOLUTE_COST_CHANGE";
    }
    // 3. Check relative cost change (after first iteration)
    else if (gn_iter > 0 && prev_cost > 0 && 
             std::abs(prev_cost - curr_cost) / prev_cost <= config_->rel_cost_change_thresh) {
      converged = true;
      termination_reason = "CONVERGED_RELATIVE_COST_CHANGE";
    }
    // 4. Check zero gradient
    else if (grad_norm < config_->zero_gradient_thresh) {
      converged = true;
      termination_reason = "CONVERGED_ZERO_GRADIENT";
    }

    // DEGENERACY-AWARE EIGENSPACE PROJECTION
    // Construct inverse block scaling matrix: D_inv
    Eigen::Matrix<double, 6, 6> D_inv = Eigen::Matrix<double, 6, 6>::Identity();
    D_inv.block<3, 3>(0, 0) *= (1.0 / ell_mr);  // rotation scaling, use the mean range distance instead.
    // translation scaling remains 1.0
    // Scale the jacobian
    Eigen::MatrixXd A_scaled = A * D_inv;
    // Degeneracy analysis in eigenspace
    Eigen::Matrix<double, 6, 6> H_scaled = A_scaled.transpose() * W_inv * A_scaled;
    
    // Compute update step using eigenspace projection
    Eigen::VectorXd delta_params_scaled = computeSimpleUpdateStep(A_scaled, b);
    // Compute the scaled covariance matrix
    Eigen::Matrix<double, 6, 6> daicp_cov_scaled = computeP2PlaneCovariance(H_scaled);

    // Unscale the parameters and covariance
    Eigen::VectorXd delta_params = D_inv * delta_params_scaled;
    daicp_cov = D_inv * daicp_cov_scaled * D_inv.transpose();
    
    // Accumulate parameters 
    accumulated_params += delta_params;
    // Convert accumulated parameters to transformation
    current_transformation = paramsToTransformationMatrix(accumulated_params);
    // Check parameter change convergence AFTER applying the update
    double param_change = delta_params.norm();
    
    // Check parameter change convergence (after at least one iteration)
    if (gn_iter > 0 && param_change < config_->inner_tolerance) {
      converged = true;
      termination_reason = "CONVERGED_PARAMETER_CHANGE";
      break;
    }
    // check convergence
    if (converged) {
      CLOG(DEBUG, "lidar.localization_daicp") << "Converged after " << gn_iter + 1 << " iterations: " 
                                              << termination_reason;
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
  // Compose the delta transformation with the initial transformation: T_final = delta_T * T_initial
  const lgmath::se3::Transformation final_transformation = lgmath::se3::Transformation(
      static_cast<Eigen::Matrix4d>(current_transformation.matrix() * initial_T_var.matrix())
  );
  
  // CLOG(DEBUG, "lidar.localization_daicp") << " current_transformation:\n" << current_transformation.matrix();
  // CLOG(DEBUG, "lidar.localization_daicp") << " initial_T_var:\n" << initial_T_var.matrix();  
  // CLOG(DEBUG, "lidar.localization_daicp") << " final_transformation:\n" << final_transformation.matrix();


  // Calculate the actual delta from initial to final for STEAM update
  Eigen::Matrix4d delta_for_steam = final_transformation.matrix() * initial_T_var.matrix().inverse();
  Eigen::Matrix<double, 6, 1> delta_vec_steam = lgmath::se3::tran2vec(delta_for_steam);
  
  // Apply the update to T_var using STEAM's update mechanism
  T_var->update(delta_vec_steam);

  // CLOG(DEBUG, "lidar.localization_daicp") << " updated T_m_s_var:\n" << T_var->value().matrix();


  return true;
}

}  // daicp_lib
}  // namespace lidar
}  // namespace vtr