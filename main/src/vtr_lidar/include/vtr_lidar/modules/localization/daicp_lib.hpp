#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "steam.hpp"
#include <iostream>
#include <vector>


namespace vtr {

namespace lidar {

namespace daicp_lib {

inline int daicp_num_thread = 8;

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
  
  // Jacobian with respect to rotation parameters (same as Python reference)
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
    Eigen::MatrixXd& Vf) {
  
  const int n_dims = eigenvalues.size();
  
  // V is the full eigenvector matrix (each column is an eigenvector)
  V = eigenvectors;
  
  // Initialize Vf as zeros
  Vf = Eigen::MatrixXd::Zero(n_dims, n_dims);
  
  // Find well-conditioned directions using the threshold
  std::vector<bool> well_conditioned_mask(n_dims);
  int num_well_conditioned = 0;
  
  for (int i = 0; i < n_dims; ++i) {
    well_conditioned_mask[i] = eigenvalues[i] > eigenvalue_threshold;
    if (well_conditioned_mask[i]) {
      num_well_conditioned++;
    }
  }
  
  // Keep the good directions as-is, zero out the others
  for (int i = 0; i < n_dims; ++i) {
    if (well_conditioned_mask[i]) {
      Vf.col(i) = V.col(i);
    }
    // else: Vf.col(i) remains zero
  }
  
  CLOG(INFO, "lidar.localization_daicp") 
      << "Well-conditioned directions: " << num_well_conditioned << "/6"
      << " (threshold: " << eigenvalue_threshold << ")";
  
  // Log which directions are well-conditioned
  std::string mask_str = "[";
  for (int i = 0; i < n_dims; ++i) {
    mask_str += well_conditioned_mask[i] ? "True" : "False";
    if (i < n_dims - 1) mask_str += ", ";
  }
  mask_str += "]";
  
  CLOG(DEBUG, "lidar.localization_daicp") 
      << "Well-conditioned mask: " << mask_str;
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
    // Use pseudoinverse for robustness (equivalent to pinv in Python)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(AtA, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd AtA_pinv = svd.solve(Eigen::MatrixXd::Identity(AtA.rows(), AtA.cols()));
    delta_x_f = AtA_pinv * A.transpose() * b;
  } catch (const std::exception& e) {
    // Fallback to least squares (equivalent to lstsq in Python)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    delta_x_f = svd.solve(b);
  }
  
  // Only use well-conditioned directions for the update
  // Since V is orthogonal matrix with V^T V = I, we have V^{-1} = V^T
  // The equation from the paper: V^{-1} @ (Vf @ delta_x_f) = V^T @ (Vf @ delta_x_f)
  // But since each column of Vf is either an eigenvector or zero, we can use:
  // delta_x_projected = V @ (Vf^T @ delta_x_f)
  const Eigen::VectorXd delta_x_projected = V * (Vf.transpose() * delta_x_f);
  
  return delta_x_projected;
}

inline bool computeEigenvalueDecomposition(
    const Eigen::MatrixXd& AtA,
    Eigen::VectorXd& eigenvalues,
    Eigen::MatrixXd& eigenvectors) {
  
  // Add regularization 
  const double reg_val = 1e-12;
  Eigen::MatrixXd AtA_reg = AtA + reg_val * Eigen::MatrixXd::Identity(AtA.rows(), AtA.cols());

  try {
    // Primary method: SelfAdjointEigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(AtA_reg);

    if (eigen_solver.info() != Eigen::Success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Eigenvalue decomposition failed, trying SVD fallback";
      
      // Fallback to SVD 
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(AtA_reg, Eigen::ComputeFullU | Eigen::ComputeFullV);
      eigenvalues = svd.singularValues();
      eigenvectors = svd.matrixU();
      
      // Threshold tiny singular values
      const double eigenvalue_threshold = 1e-10;
      for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < eigenvalue_threshold) {
          eigenvalues(i) = 0.0;
        }
      }
      
      CLOG(INFO, "lidar.localization_daicp") << "SVD fallback successful";
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
    
    // Debug logging to verify eigenvalues match Python
    CLOG(DEBUG, "lidar.localization_daicp") 
        << "Eigenvalues (descending): [" << eigenvalues.transpose() << "]";
    
    return true;
    
  } catch (const std::exception& e) {
    CLOG(ERROR, "lidar.localization_daicp") 
        << "Exception in eigenvalue decomposition: " << e.what();
    return false;
  }
}

inline double computeThreshold(const Eigen::VectorXd& eigenvalues) {
  const double max_eigenval = eigenvalues.maxCoeff();
  const double eigenvalue_threshold = max_eigenval * 1e-6;  // Conservative threshold
  return eigenvalue_threshold;
}

inline bool linearizeOptimizationProblem(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    steam::se3::SE3StateVar::Ptr T_var,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b) {
  
  const int num_constraints = sample_inds.size();
  A.resize(num_constraints, 6);
  b.resize(num_constraints);
  
  // Get current transformation evaluation
  const Eigen::Matrix4d T_current = T_var->value().matrix();
  
#pragma omp parallel for schedule(dynamic, 10) num_threads(daicp_num_thread)
  for (size_t i = 0; i < sample_inds.size(); ++i) {
    const auto& ind = sample_inds[i];
    
    // Get points and normal
    const Eigen::Vector3d source_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
    const Eigen::Vector3d target_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
    const Eigen::Vector3d target_normal = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
    
    // Transform source point using current transformation
    const Eigen::Vector4d source_pt_hom = (Eigen::Vector4d() << source_pt, 1.0).finished();
    const Eigen::Vector3d transformed_source = (T_current * source_pt_hom).head<3>();
    
    // Compute Jacobian using the helper function
    const Eigen::VectorXd jacobian = computeP2PlaneJacobian(transformed_source, target_normal);
    A.row(i) = jacobian.transpose();
    
    // Compute residual (point-to-plane distance): n^T * (target - transformed_source)
    const Eigen::Vector3d n = target_normal.normalized();
    b(i) = n.dot(target_pt - transformed_source);
  }
  
  return true;
}

inline bool daGaussNewtonP2Plane(
    const std::vector<std::pair<size_t, size_t>>& sample_inds,
    const Eigen::Matrix4Xf& query_mat,
    const Eigen::Matrix4Xf& map_mat,
    const Eigen::Matrix4Xf& map_normals_mat,
    steam::se3::SE3StateVar::Ptr T_var,
    int max_gn_iter,
    double inner_tolerance) {
  
  if (sample_inds.size() < 6) {
    CLOG(WARNING, "lidar.localization_daicp") << "Insufficient correspondences for Gauss-Newton";
    return false;
  }
  
  // STEAM-style convergence parameters
  const double absolute_cost_threshold = 1e-8;
  const double absolute_cost_change_threshold = 1e-4;
  const double relative_cost_change_threshold = 1e-4;
  const double zero_gradient_threshold = 1e-6;
  const double parameter_change_threshold = inner_tolerance;
  
  // Store initial transformation
  const auto T_initial = T_var->value();
  
  // Initialize transformation parameters (following Python reference)
  Eigen::VectorXd params = Eigen::VectorXd::Zero(6);
  lgmath::se3::Transformation current_transformation; // Identity by default
  
  // Variables for convergence tracking
  double prev_cost = std::numeric_limits<double>::max();
  double curr_cost = 0.0;
  bool converged = false;
  std::string termination_reason = "";
  
  // Gauss-Newton iterations with DEGENERACY-AWARE updates
  for (int gn_iter = 0; gn_iter < max_gn_iter && !converged; ++gn_iter) {
    CLOG(DEBUG, "lidar.localization_daicp") 
        << "  Gauss-Newton iteration " << gn_iter;
    
    // Build jacobian and residual for current transformation
    // Following Python: transform source points with current estimate first
    Eigen::MatrixXd A(sample_inds.size(), 6);
    Eigen::VectorXd b(sample_inds.size());
    
    const Eigen::Matrix4d T_current_mat = current_transformation.matrix();
    
#pragma omp parallel for schedule(dynamic, 10) num_threads(daicp_num_thread)
    for (size_t i = 0; i < sample_inds.size(); ++i) {
      const auto& ind = sample_inds[i];
      
      // Get original source point and transform it
      const Eigen::Vector3d source_pt_original = query_mat.block<3, 1>(0, ind.first).cast<double>();
      Eigen::Vector4d source_pt_hom;
      source_pt_hom << source_pt_original, 1.0;
      const Eigen::Vector3d source_pt_transformed = (T_current_mat * source_pt_hom).head<3>();
      
      // Get target point and normal
      const Eigen::Vector3d target_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
      const Eigen::Vector3d target_normal = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      
      // Compute Jacobian using the helper function
      const Eigen::VectorXd jacobian = computeP2PlaneJacobian(source_pt_transformed, target_normal);
      A.row(i) = jacobian.transpose();
      
      // Compute residual (point-to-plane distance): n^T * (target - transformed_source)
      const Eigen::Vector3d n = target_normal.normalized();
      b(i) = n.dot(target_pt - source_pt_transformed);
    }
    
    // Compute current cost (0.5 * ||b||^2) and gradient norm
    curr_cost = 0.5 * b.squaredNorm();
    const double grad_norm = (A.transpose() * b).norm();
    
    CLOG(DEBUG, "lidar.localization_daicp") 
        << "  Iter " << gn_iter << " - Cost: " << curr_cost << ", Grad norm: " << grad_norm;
    
    // STEAM-style convergence checking
    
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
      CLOG(DEBUG, "lidar.localization_daicp") 
          << "  Gauss-Newton converged after " << gn_iter << " iterations. Reason: " << termination_reason;
      break;
    }
    
    // DEGENERACY-AWARE EIGENSPACE PROJECTION
    // Compute eigenvalue decomposition of A^T A
    const Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenvectors;
    bool eigen_success = computeEigenvalueDecomposition(AtA, eigenvalues, eigenvectors);
    
    if (!eigen_success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Gauss-Newton eigenvalue decomposition failed";
      return false;
    }
    
    // Compute unified threshold
    // const double eigenvalue_threshold = computeThreshold(eigenvalues);

    const double eigenvalue_threshold = 50.0;
    
    // Construct well-conditioned directions matrix Vf
    Eigen::MatrixXd V, Vf;
    constructWellConditionedDirections(eigenvalues, eigenvectors, eigenvalue_threshold, V, Vf);
    
    // Compute update step using eigenspace projection
    Eigen::VectorXd delta_params = computeUpdateStep(A, b, V, Vf);
    
    // 5. Check parameter change convergence
    const double param_change = delta_params.norm();
    if (param_change < parameter_change_threshold) {
      converged = true;
      termination_reason = "CONVERGED_PARAMETER_CHANGE";
      CLOG(DEBUG, "lidar.localization_daicp") 
          << "  Gauss-Newton converged after " << gn_iter << " iterations. Reason: " << termination_reason
          << " (param change: " << param_change << ")";
      break;
    }

    // Update parameters 
    params += delta_params;
    
    // Convert parameters to transformation matrix 
    current_transformation = paramsToTransformationMatrix(params);
    
    CLOG(DEBUG, "lidar.localization_daicp") 
        << "  Accumulated params [rx,ry,rz,tx,ty,tz]: [" 
        << params(0) << ", " << params(1) << ", " << params(2) << ", "
        << params(3) << ", " << params(4) << ", " << params(5) << "]";
    
    // Update cost for next iteration
    prev_cost = curr_cost;
  }
  
  // Check if terminated due to max iterations
  if (!converged) {
    CLOG(WARNING, "lidar.localization_daicp") 
        << "  Gauss-Newton reached maximum iterations (" << max_gn_iter << ") without convergence";
    termination_reason = "MAX_ITERATIONS";
  }
  
  // Apply the final transformation to T_var
  // current_transformation represents the total delta from identity
  // Convert to SE(3) vector and apply as update
  const Eigen::Matrix<double, 6, 1> delta_vec = lgmath::se3::tran2vec(current_transformation.matrix()); 
  CLOG(DEBUG, "lidar.localization_daicp") 
        << "  delta_vec before T_var->update [rx,ry,rz,tx,ty,tz]: [" 
        << delta_vec(0) << ", " << delta_vec(1) << ", " << delta_vec(2) << ", "
        << delta_vec(3) << ", " << delta_vec(4) << ", " << delta_vec(5) << "]";


  // [consistent with STEAM]
  //   Eigen::Matrix4d T_var_update = current_transformation.matrix() * T_var->value().matrix();
  //   CLOG(DEBUG, "lidar.localization_daicp") 
  //       << " T_var matrix (T_curr * T_var):\n" << T_var_update;

  //   T_var->update(delta_vec);
  
  // steam update
  CLOG(DEBUG, "lidar.localization_daicp") 
      << " T_var matrix (steam):\n" << T_var->value().matrix().cast<float>();

  return true;
}

}  // daicp_lib

}  // namespace lidar
}  // namespace vtr