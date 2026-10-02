#pragma once

// P2Plane: point-to-plane-only DA-ICP.
// Method-specific DA-ICP functions, reproduced verbatim from branch aeva_warthog_speed.
// Functions shared by all methods live in da_lib.hpp (namespace da_lib); they
// are visible here through the enclosing namespace.
// NOTE: internal parameter ordering is [rotation, translation] ([rx,ry,rz,tx,ty,tz]);
// the caller permutes the prior covariance in and the DA-ICP covariance out.

#include "vtr_lidar/modules/localization/da_lib.hpp"

namespace vtr {
namespace lidar {

class LocalizationDAICPModule;

namespace da_lib {
namespace p2plane {

// =================== Print Functions ===================
inline void printCovarianceInfo(const Eigen::MatrixXd& daicp_cov) {
  const Eigen::VectorXd diagonal = daicp_cov.diagonal();
  const Eigen::VectorXd std_dev = diagonal.cwiseSqrt();
  
  CLOG(DEBUG, "lidar.localization_daicp") << "Final covariance P diagonal (roll, pitch, yaw, x, y, z): [" << diagonal.transpose() << "]";
  CLOG(DEBUG, "lidar.localization_daicp") << "Final std (roll, pitch, yaw, x, y, z): [" << std_dev.transpose() << "]";
}

// =================== Block Scaling Functions ===================
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
  // H_marg_theta = makePSD(H_marg_theta);
  // H_marg_t = makePSD(H_marg_t);
  
  return std::make_pair(H_marg_theta, H_marg_t);
}

inline double computeScalingFactorTrace(const Eigen::Matrix3d& H_marg_theta, const Eigen::Matrix3d& H_marg_t) {
  // Compute scaling factor using the trace method
  double tr_theta = H_marg_theta.trace();
  double tr_t = H_marg_t.trace();
  
  if (tr_t < 1e-12) {
    std::cout << "[WARNING] Translation trace is very small (" << tr_t << "), using default scaling" << std::endl;
    return 20.0; // mid-range for 40 meter lidar
  }
  
  return std::sqrt(tr_theta / tr_t);
}

inline double computeScalingFactorMax(const Eigen::Matrix3d& H_marg_theta,
                                      const Eigen::Matrix3d& H_marg_t)
{
    constexpr double reg_val = 1e-12;

    // Add tiny diagonal regularization (fast: avoids a new matrix allocation)
    Eigen::Matrix3d H_theta = H_marg_theta;
    H_theta.diagonal().array() += reg_val;

    Eigen::Matrix3d H_t = H_marg_t;
    H_t.diagonal().array() += reg_val;

    // Fastest 3x3 symmetric eigen decomposition in Eigen
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_theta(H_theta, Eigen::EigenvaluesOnly);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_t(H_t, Eigen::EigenvaluesOnly);

    // Eigenvalues are sorted in ASCENDING order: [λ_min, λ_mid, λ_max]
    const double max_theta = solver_theta.eigenvalues()(2);
    const double max_t     = solver_t.eigenvalues()(2);

    if (max_t < 1e-12) {
        std::cout << "[WARNING] Translation max eigenvalue very small (" << max_t << "), using default scaling\n";
        return 20.0; // mid-range for 40 meter lidar
    }

    return std::sqrt(max_theta / max_t);
}


// =================== Covariance Computation ===================
inline Eigen::Matrix<double, 6, 6> makePD(const Eigen::Matrix<double, 6, 6>& cov, 
                                          double min_eigenvalue = 1e-6) {
    // Check if matrix is already positive definite using Cholesky decomposition
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(cov);
    if (llt.info() == Eigen::Success) {
        return cov;  // Already positive definite
    }

    // Try LDLT decomposition (handles semi-definite cases)
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(cov);
    if (ldlt.info() == Eigen::Success) {
        // Regularize by clamping very small or negative diagonal elements
        Eigen::Matrix<double, 6, 1> D = ldlt.vectorD();
        bool modified = false;
        for (int i = 0; i < D.size(); ++i) {
            if (D(i) < min_eigenvalue) {
                D(i) = min_eigenvalue;
                modified = true;
            }
        }
        if (modified)
            CLOG(DEBUG, "lidar.localization_daicp") << "Regularized LDLT diagonal values";

        // Reconstruct: cov_PD = P * D * Pᵀ
        Eigen::Matrix<double, 6, 6> P = ldlt.transpositionsP() * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 6> cov_pd = P * D.asDiagonal() * P.transpose();
        return cov_pd;
    } 

    // Use eigenvalue decomposition to fix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(cov);
    if (eigensolver.info() != Eigen::Success) {
        CLOG(WARNING, "lidar.localization_daicp") << "Eigenvalue decomposition failed, using diagonal regularization";
        return cov + min_eigenvalue * Eigen::Matrix<double, 6, 6>::Identity();
    }
    
    Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
    Eigen::Matrix<double, 6, 6> eigenvectors = eigensolver.eigenvectors();
    
    // Clamp negative eigenvalues
    bool modified = false;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < min_eigenvalue) {
            eigenvalues(i) = min_eigenvalue;
            modified = true;
        }
    }
    
    if (modified) {
        CLOG(DEBUG, "lidar.localization_daicp") << "Fixed " << eigenvalues.size() << " negative eigenvalues";
    }
    
    // Reconstruct the matrix
    return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
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
    Eigen::VectorXd& W_inv,
    const std::shared_ptr<const vtr::lidar::LocalizationDAICPModule::Config>& config_) {
  
  // Lidar range and bearing noise model parameters
  const double sigma_d = config_->sigma_d;          // range noise std (2cm)
  const double sigma_az = config_->sigma_az;        // azimuth noise std (0.03 degrees)
  const double sigma_el = config_->sigma_el;        // elevation noise std (0.03 degrees)

  const Eigen::Matrix3d Sigma_du = Eigen::Vector3d(sigma_d * sigma_d,
                                                   sigma_az * sigma_az,
                                                   sigma_el * sigma_el).asDiagonal();
  
  // using OpenMP reduction to safely accumulate sum_range and valid_count
#pragma omp parallel for schedule(static) num_threads(config_->num_threads)
  for (uint i = 0; i < sample_inds.size(); ++i) {
    const auto& ind = sample_inds[i];
      
    Eigen::VectorXd jacobian(6);
    Eigen::Vector3d target_pt, target_normal;
    Eigen::Matrix3d Sigma_q_i, Sigma_n_i, Sigma_pL_i;
    Eigen::Vector4d source_pt_hom;
    Eigen::Matrix3d R_combined = T_combined.block<3,3>(0,0);

    // Get original source point and transform it with combined transformation
    const Eigen::Vector3d source_pt_original = query_mat.block<3, 1>(0, ind.first).cast<double>();
    source_pt_hom = Eigen::Vector4d(source_pt_original(0), source_pt_original(1), source_pt_original(2), 1.0);
    const Eigen::Vector3d source_pt_transformed = (T_combined * source_pt_hom).head<3>();
    
    // Get target point and normal
    target_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
    target_normal = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
    const Eigen::Vector3d n_t = target_normal.normalized();
    
    // ===== 1. Compute Jacobian and Residual =====
    jacobian = computeP2PlaneJacobian(source_pt_transformed, n_t);
    A.row(i) = jacobian.transpose();
    
    // Compute residual (point-to-plane distance)
    b(i) = n_t.dot(target_pt - source_pt_transformed);
    
    // ===== 2. Compute Sigma_pL_s for this point =====
    // Compute range, azimuth, elevation in lidar frame
    const double range = source_pt_original.norm();
    // const double flat_range = std::sqrt(p_lidar(0)*p_lidar(0) + p_lidar(1)*p_lidar(1));
    const double azimuth = std::atan2(source_pt_original(1), source_pt_original(0));
    const double elevation = std::atan2(source_pt_original(2), std::sqrt(source_pt_original(0)*source_pt_original(0) + source_pt_original(1)*source_pt_original(1)));

    // Compute covariance for this measurement
    Sigma_pL_i = computeRangeBearingCovariance(range, azimuth, elevation, Sigma_du);
    
    // ===== 3. Compute Information Weight for this point =====
    // Jacobians for variance terms:
    // J_p = n^T R   (since residual uses R p + t)
    const Eigen::RowVector3d J_p = (n_t.transpose() * R_combined);      // 1x3
    // J_q = -n^T
    const Eigen::RowVector3d J_q = (-n_t.transpose());                   // 1x3
    // J_n = (R p + t - q)^T
    const Eigen::Vector3d d = source_pt_transformed - target_pt;
    const Eigen::RowVector3d J_n = d.transpose();                        // 1x3

    // --- Map-point covariance  ---
    const double sigma_map = 0.01;  // meters
    Sigma_q_i = (sigma_map * sigma_map) * Eigen::Matrix3d::Identity();

    // --- Normal covariance (tangent-plane, PSD) ---
    // Uncertainty only in directions orthogonal to n_t.
    const double sigma_n = 0.03; // e.g., a small angle in radians mapped to length via ||d||
                                // typical: normal_sigma ~ 0.02 to 0.05 (tuned)
    const Eigen::Matrix3d Pn = Eigen::Matrix3d::Identity() - n_t * n_t.transpose();
    Sigma_n_i = (sigma_n * sigma_n) * Pn;

    // --- Scalar residual variance ---
    double cov_r = 0.0;
    cov_r += (J_p * Sigma_pL_i * J_p.transpose())(0,0); // source point noise
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
    //     << "  | JpΣpJp^T=" << (J_p * Sigma_pL_i * J_p.transpose())(0,0)
    //     << "  | JqΣqJq^T=" << (J_q * Sigma_q_i    * J_q.transpose())(0,0)
    //     << "  | JnΣnJn^T=" << (J_n * Sigma_n_i    * J_n.transpose())(0,0);

    // Store weight (diagonal of W_inv)
    W_inv(i) = w;
  }
}

inline Eigen::VectorXd computeUpdateStep(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::Matrix<double, 6, 6>& V,
    const Eigen::Matrix<double, 6, 6>& Vf)
{
    // Compute RHS once
    const Eigen::VectorXd Atb = A.transpose() * b;

    // Solve (AᵀA) Δx = Aᵀ b using the normal-equation Cholesky decomposition.
    Eigen::VectorXd delta_x_f;
    Eigen::LLT<Eigen::MatrixXd> llt(A.transpose() * A);

    if (llt.info() == Eigen::Success) {
        delta_x_f = llt.solve(Atb);
    } else {
        // LAST-RESORT fallback: SVD(A)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        delta_x_f = svd.solve(b);
    }

    // Project degeneracies
    return V * (Vf.transpose() * delta_x_f);
}

inline bool daGaussNewtonP2Plane(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    steam::se3::SE3StateVar::Ptr T_var,
    const std::shared_ptr<const vtr::lidar::LocalizationDAICPModule::Config>& config_,
    const Eigen::Matrix<double, 6, 6>& prior_cov,  // in internal [rx,ry,rz,tx,ty,tz] order
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
    Eigen::VectorXd W_inv;
    W_inv.resize(sample_inds.size());
    Eigen::VectorXd b(sample_inds.size());
    
    // Compose with initial transformation: final_T = current_T * initial_T
    const Eigen::Matrix4d T_combined = current_transformation.matrix() * initial_T_var.matrix();

    // Compute Jacobian, residuals, and information matrix
    // Note: Measurement noise is computed from the original sensor measurements (query_mat),
    // which are independent of the state estimate, as it should be.
    computeJacobianResidualInformation(sample_inds, query_mat, map_mat, map_normals_mat,
                                       T_combined, A, b, W_inv, config_);

    // Compute original Hessian
    Eigen::MatrixXd H_original = A.transpose() * W_inv.asDiagonal() * A;
    // Apply Schur complement marginalization
    auto [H_marg_theta, H_marg_t] = schurComplementMarginalization(H_original);
    // Compute scaling factor
    // ell_mr = computeScalingFactorTrace(H_marg_theta, H_marg_t);
    double ell_mr = computeScalingFactorMax(H_marg_theta, H_marg_t);

    CLOG(DEBUG, "lidar.localization_daicp") << "----------------------use H_the/H_t, ell_mr:   " << ell_mr;

    // Compute current weighted cost (0.5 * b^T * W_inv * b) and weighted gradient norm
    curr_cost = 0.5 * b.transpose() * W_inv.asDiagonal() * b;
    const Eigen::VectorXd weighted_gradient = A.transpose() * W_inv.asDiagonal() * b;
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
    // --- compute original Hessian
    // Eigen::Matrix<double, 6, 6>  H_original = A.transpose() * W_inv * A;
    // Construct inverse block scaling matrix: D_inv
    Eigen::Matrix<double, 6, 6> D_inv = Eigen::Matrix<double, 6, 6>::Identity();
    D_inv.block<3, 3>(0, 0) *= (1.0 / ell_mr);  // rotation scaling, use the mean range distance instead.
    // translation scaling remains 1.0
    // Scale the jacobian
    Eigen::MatrixXd A_scaled = A * D_inv;
    // Degeneracy analysis in eigenspace
    Eigen::Matrix<double, 6, 6> H_scaled = A_scaled.transpose() * W_inv.asDiagonal() * A_scaled;

    Eigen::VectorXd eigenvalues;
    Eigen::Matrix<double, 6, 6> eigenvectors;
    bool eigen_success = computeEigenvalueDecomposition(H_scaled, eigenvalues, eigenvectors);

    if (!eigen_success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Gauss-Newton eigenvalue decomposition failed";
      return false;
    }
    
    // Compute unified threshold
    const double eigenvalue_threshold = computeThreshold(eigenvalues, config_->degeneracy_thresh);
    
    // Construct well-conditioned directions matrix 
    Eigen::Matrix<double, 6, 6> V, Vf;
    Eigen::Matrix<double, 6, Eigen::Dynamic> Vd;
    Eigen::VectorXd eigen_vf;
    constructWellConditionedDirections(eigenvalues, eigenvectors, eigenvalue_threshold, 
                                       V, Vf, eigen_vf, Vd);

    // Compute update step using eigenspace projection
    Eigen::VectorXd delta_params_scaled = computeUpdateStep(A_scaled, b, V, Vf);
    // Compute the scaled covariance matrix.
    // Vf/Vd live in the scaled frame: rotation entries multiplied by ell_mr (since
    // D_inv scales the rotation block at index (0,0) on this branch). The prior must
    // be expressed in the same scaled frame:
    //   Sigma_scaled = D * Sigma * D^T,  D = diag(ell_mr, ell_mr, ell_mr, 1, 1, 1)
    // (rotation first because this branch uses [rx,ry,rz,tx,ty,tz] internal order.)
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();
    D.block<3, 3>(0, 0) *= ell_mr;
    const Eigen::Matrix<double, 6, 6> prior_cov_scaled = D * prior_cov * D.transpose();
    Eigen::Matrix<double, 6, 6> daicp_cov_scaled = config_->use_prior_prop_cov
        ? computeDaicpCovariance(Vf, eigen_vf, Vd, prior_cov_scaled, config_->degenerate_cov_alpha)
        : computeDaicpCovarianceDefault(Vf, eigen_vf, Vd);
    
    // Unscale the parameters and covariance
    Eigen::VectorXd delta_params = D_inv * delta_params_scaled;
    daicp_cov = D_inv * daicp_cov_scaled * D_inv.transpose();
    // --- Debug-print covariance information
    // printCovarianceInfo(daicp_cov);
    
    // Accumulate parameters 
    accumulated_params += delta_params;
    // Convert accumulated parameters to transformation

    // Lgmath uses [tx, ty, tz, rx, ry, rz] ordering.
    Eigen::Matrix<double, 6, 1> xi;
    xi << accumulated_params[3], accumulated_params[4], accumulated_params[5],
          accumulated_params[0], accumulated_params[1], accumulated_params[2];
    current_transformation = lgmath::se3::Transformation(lgmath::se3::vec2tran(xi));


    // Check parameter change convergence AFTER applying the update
    double param_change = delta_params.norm();
    
    // Check convergence (but allow at least one iteration to see progress)
    if (gn_iter > 0 && param_change < config_->inner_tolerance) {
      converged = true;
      termination_reason = "CONVERGED_PARAMETER_CHANGE";
      break;
    }
    // check convergence
    if (converged) {
      CLOG(DEBUG, "lidar.localization_daicp") << "Converged after " << (gn_iter) << " iterations: " << termination_reason;
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
  
  // Calculate the actual delta from initial to final for STEAM update
  Eigen::Matrix4d delta_for_steam = final_transformation.matrix() * initial_T_var.matrix().inverse();
  Eigen::Matrix<double, 6, 1> delta_vec_steam = lgmath::se3::tran2vec(delta_for_steam);
  
  // Apply the update to T_var using STEAM's update mechanism
  T_var->update(delta_vec_steam);

  return true;
}

}  // namespace p2plane
}  // namespace da_lib
}  // namespace lidar
}  // namespace vtr
