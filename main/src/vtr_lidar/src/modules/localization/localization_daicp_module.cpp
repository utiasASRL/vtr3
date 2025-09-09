#include "vtr_lidar/modules/localization/localization_daicp_module.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;
using namespace steam;
using namespace steam::se3;

auto LocalizationDAICPModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config->use_pose_prior);

  // icp params
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  config->first_num_steps = node->declare_parameter<int>(param_prefix + ".first_num_steps", config->first_num_steps);
  config->initial_max_iter = node->declare_parameter<int>(param_prefix + ".initial_max_iter", config->initial_max_iter);
  config->initial_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".initial_max_pairing_dist", config->initial_max_pairing_dist);
  config->initial_max_planar_dist = node->declare_parameter<float>(param_prefix + ".initial_max_planar_dist", config->initial_max_planar_dist);
  config->refined_max_iter = node->declare_parameter<int>(param_prefix + ".refined_max_iter", config->refined_max_iter);
  config->refined_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".refined_max_pairing_dist", config->refined_max_pairing_dist);
  config->refined_max_planar_dist = node->declare_parameter<float>(param_prefix + ".refined_max_planar_dist", config->refined_max_planar_dist);
  config->averaging_num_steps = node->declare_parameter<int>(param_prefix + ".averaging_num_steps", config->averaging_num_steps);
  config->rot_diff_thresh = node->declare_parameter<float>(param_prefix + ".rot_diff_thresh", config->rot_diff_thresh);
  config->trans_diff_thresh = node->declare_parameter<float>(param_prefix + ".trans_diff_thresh", config->trans_diff_thresh);
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", false);
  config->max_iterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", 1);
  config->target_loc_time = node->declare_parameter<float>(param_prefix + ".target_loc_time", config->target_loc_time);

  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);
  // clang-format on
  return config;
}

/////// Helper functions
Eigen::VectorXd LocalizationDAICPModule::computeP2PlaneJacobian(
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

void LocalizationDAICPModule::constructWellConditionedDirections(
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

Eigen::VectorXd LocalizationDAICPModule::computeUpdateStep(
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

bool LocalizationDAICPModule::computeEigenvalueDecomposition(
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

double LocalizationDAICPModule::computeThreshold(const Eigen::VectorXd& eigenvalues) {
  const double max_eigenval = eigenvalues.maxCoeff();
  const double eigenvalue_threshold = max_eigenval * 1e-6;  // Conservative threshold
  return eigenvalue_threshold;
}

bool LocalizationDAICPModule::linearizeOptimizationProblem(
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
  
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
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


bool LocalizationDAICPModule::daGaussNewtonP2Plane(
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
  
  // Get corresponding points and normals (like Python)
  std::vector<Eigen::Vector3d> p_s_original, p_t, n_t;
  for (const auto& ind : sample_inds) {
    p_s_original.push_back(query_mat.block<3, 1>(0, ind.first).cast<double>());
    p_t.push_back(map_mat.block<3, 1>(0, ind.second).cast<double>());
    n_t.push_back(map_normals_mat.block<3, 1>(0, ind.second).cast<double>());
  }
  
  // Initialize transformation parameters (like Python)
  Eigen::VectorXd params = Eigen::VectorXd::Zero(6);
  lgmath::se3::Transformation current_transformation; // Identity
  
  // Store initial T_var value to apply final delta
  const auto T_var_initial = T_var->value();
  
  // Gauss-Newton iterations with DEGENERACY-AWARE updates (like Python)
  for (int gn_iter = 0; gn_iter < max_gn_iter; ++gn_iter) {
    CLOG(DEBUG, "lidar.localization_daicp") 
        << "  Gauss-Newton iteration " << gn_iter;
    
    // Transform source points with current estimate (like Python)
    std::vector<Eigen::Vector3d> p_s_current;
    const Eigen::Matrix4d T_current = current_transformation.matrix();
    for (const auto& pt : p_s_original) {
      Eigen::Vector4d pt_hom = (Eigen::Vector4d() << pt, 1.0).finished();
      p_s_current.push_back((T_current * pt_hom).head<3>());
    }
    
    // Build jacobian and residual (like Python)
    const int num_constraints = sample_inds.size();
    Eigen::MatrixXd A(num_constraints, 6);
    Eigen::VectorXd b(num_constraints);
    
    for (size_t i = 0; i < sample_inds.size(); ++i) {
      const Eigen::Vector3d& ps = p_s_current[i];
      const Eigen::Vector3d& pt = p_t[i];
      const Eigen::Vector3d& nt = n_t[i];
      
      // Compute Jacobian for this correspondence
      const Eigen::VectorXd jacobian = computeP2PlaneJacobian(ps, nt);
      A.row(i) = jacobian.transpose();
      
      // Compute residual (point-to-plane distance)
      const Eigen::Vector3d n = nt.normalized();
      b(i) = n.dot(pt - ps);
    }
    
    // DEGENERACY-AWARE EIGENSPACE PROJECTION (like Python)
    // Compute eigenvalue decomposition of A^T A
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenvectors;
    bool eigen_success = computeEigenvalueDecomposition(A.transpose() * A, eigenvalues, eigenvectors);
    
    if (!eigen_success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Eigenvalue decomposition failed";
      return false;
    }
    
    // Compute unified thresholds (like Python)
    // const double eigenvalue_threshold = computeThreshold(eigenvalues);

    const double eigenvalue_threshold = 50; 
    
    // Construct well-conditioned directions matrix Vf (like Python)
    Eigen::MatrixXd V, Vf;
    constructWellConditionedDirections(eigenvalues, eigenvectors, eigenvalue_threshold, V, Vf);
    
    // Compute update step using eigenspace projection (like Python)
    Eigen::VectorXd delta_params = computeUpdateStep(A, b, V, Vf);
    
    // Update parameters (like Python)
    params += delta_params;
    
    // Convert parameters to transformation matrix (like Python)
    current_transformation = paramsToTransformationMatrix(params);
    
    // Check convergence (like Python)
    double param_change = delta_params.norm();
    CLOG(DEBUG, "lidar.localization_daicp") 
        << "  Parameter change: " << param_change;
    
    if (param_change < inner_tolerance) {
      CLOG(DEBUG, "lidar.localization_daicp") 
          << "  Gauss-Newton converged (parameter change: " << param_change << ")";
      break;
    }
  }
  
  // Apply the final transformation to T_var (convert from parameter space to SE(3))
  const Eigen::Matrix<double, 6, 1> final_delta_vec = lgmath::se3::tran2vec(current_transformation.matrix());
  T_var->update(final_delta_vec);
  
  return true;
}


// bool LocalizationDAICPModule::daGaussNewtonP2Plane(
//     const std::vector<std::pair<size_t, size_t>>& sample_inds,
//     const Eigen::Matrix4Xf& query_mat,
//     const Eigen::Matrix4Xf& map_mat,
//     const Eigen::Matrix4Xf& map_normals_mat,
//     steam::se3::SE3StateVar::Ptr T_var,
//     int max_gn_iter,
//     double inner_tolerance) {
  
//   if (sample_inds.size() < 6) {
//     CLOG(WARNING, "lidar.localization_daicp") << "Insufficient correspondences for Gauss-Newton";
//     return false;
//   }
  
//   // STEAM-style convergence parameters
//   const double absolute_cost_threshold = 1e-8;
//   const double absolute_cost_change_threshold = 1e-4;
//   const double relative_cost_change_threshold = 1e-4;
//   const double zero_gradient_threshold = 1e-6;
//   const double parameter_change_threshold = inner_tolerance;
  
//   // Store initial transformation
//   const auto T_initial = T_var->value();
  
//   // Initialize transformation parameters (following Python reference)
//   Eigen::VectorXd params = Eigen::VectorXd::Zero(6);
//   lgmath::se3::Transformation current_transformation; // Identity by default
  
//   // Variables for convergence tracking
//   double prev_cost = std::numeric_limits<double>::max();
//   double curr_cost = 0.0;
//   bool converged = false;
//   std::string termination_reason = "";
  
//   // Gauss-Newton iterations with DEGENERACY-AWARE updates
//   for (int gn_iter = 0; gn_iter < max_gn_iter && !converged; ++gn_iter) {
//     CLOG(DEBUG, "lidar.localization_daicp") 
//         << "  Gauss-Newton iteration " << gn_iter;
    
//     // Build jacobian and residual for current transformation
//     // Following Python: transform source points with current estimate first
//     Eigen::MatrixXd A(sample_inds.size(), 6);
//     Eigen::VectorXd b(sample_inds.size());
    
//     const Eigen::Matrix4d T_current_mat = current_transformation.matrix();
    
// #pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
//     for (size_t i = 0; i < sample_inds.size(); ++i) {
//       const auto& ind = sample_inds[i];
      
//       // Get original source point and transform it
//       const Eigen::Vector3d source_pt_original = query_mat.block<3, 1>(0, ind.first).cast<double>();
//       Eigen::Vector4d source_pt_hom;
//       source_pt_hom << source_pt_original, 1.0;
//       const Eigen::Vector3d source_pt_transformed = (T_current_mat * source_pt_hom).head<3>();
      
//       // Get target point and normal
//       const Eigen::Vector3d target_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
//       const Eigen::Vector3d target_normal = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      
//       // Compute Jacobian using the helper function
//       const Eigen::VectorXd jacobian = computeP2PlaneJacobian(source_pt_transformed, target_normal);
//       A.row(i) = jacobian.transpose();
      
//       // Compute residual (point-to-plane distance): n^T * (target - transformed_source)
//       const Eigen::Vector3d n = target_normal.normalized();
//       b(i) = n.dot(target_pt - source_pt_transformed);
//     }
    
//     // Compute current cost (0.5 * ||b||^2) and gradient norm
//     curr_cost = 0.5 * b.squaredNorm();
//     const double grad_norm = (A.transpose() * b).norm();
    
//     CLOG(DEBUG, "lidar.localization_daicp") 
//         << "  Iter " << gn_iter << " - Cost: " << curr_cost << ", Grad norm: " << grad_norm;
    
//     // STEAM-style convergence checking
    
//     // 1. Check absolute cost threshold
//     if (curr_cost <= absolute_cost_threshold) {
//       converged = true;
//       termination_reason = "CONVERGED_ABSOLUTE_COST";
//     }
//     // 2. Check absolute cost change (after first iteration)
//     else if (gn_iter > 0 && std::abs(prev_cost - curr_cost) <= absolute_cost_change_threshold) {
//       converged = true;
//       termination_reason = "CONVERGED_ABSOLUTE_COST_CHANGE";
//     }
//     // 3. Check relative cost change (after first iteration)
//     else if (gn_iter > 0 && prev_cost > 0 && 
//              std::abs(prev_cost - curr_cost) / prev_cost <= relative_cost_change_threshold) {
//       converged = true;
//       termination_reason = "CONVERGED_RELATIVE_COST_CHANGE";
//     }
//     // 4. Check zero gradient
//     else if (grad_norm < zero_gradient_threshold) {
//       converged = true;
//       termination_reason = "CONVERGED_ZERO_GRADIENT";
//     }
    
//     if (converged) {
//       CLOG(DEBUG, "lidar.localization_daicp") 
//           << "  Gauss-Newton converged after " << gn_iter << " iterations. Reason: " << termination_reason;
//       break;
//     }
    
//     // DEGENERACY-AWARE EIGENSPACE PROJECTION
//     // Compute eigenvalue decomposition of A^T A
//     const Eigen::MatrixXd AtA = A.transpose() * A;
//     Eigen::VectorXd eigenvalues;
//     Eigen::MatrixXd eigenvectors;
//     bool eigen_success = computeEigenvalueDecomposition(AtA, eigenvalues, eigenvectors);
    
//     if (!eigen_success) {
//       CLOG(WARNING, "lidar.localization_daicp") << "Gauss-Newton eigenvalue decomposition failed";
//       return false;
//     }
    
//     // Compute unified threshold
//     // const double eigenvalue_threshold = computeThreshold(eigenvalues);

//     const double eigenvalue_threshold = 50.0;
    
//     // Construct well-conditioned directions matrix Vf
//     Eigen::MatrixXd V, Vf;
//     constructWellConditionedDirections(eigenvalues, eigenvectors, eigenvalue_threshold, V, Vf);
    
//     // Compute update step using eigenspace projection
//     Eigen::VectorXd delta_params = computeUpdateStep(A, b, V, Vf);
    
//     // 5. Check parameter change convergence
//     const double param_change = delta_params.norm();
//     if (param_change < parameter_change_threshold) {
//       converged = true;
//       termination_reason = "CONVERGED_PARAMETER_CHANGE";
//       CLOG(DEBUG, "lidar.localization_daicp") 
//           << "  Gauss-Newton converged after " << gn_iter << " iterations. Reason: " << termination_reason
//           << " (param change: " << param_change << ")";
//       break;
//     }

//     // Update parameters 
//     params += delta_params;
    
//     // Convert parameters to transformation matrix 
//     current_transformation = paramsToTransformationMatrix(params);
    
//     CLOG(DEBUG, "lidar.localization_daicp") 
//         << "  Accumulated params [rx,ry,rz,tx,ty,tz]: [" 
//         << params(0) << ", " << params(1) << ", " << params(2) << ", "
//         << params(3) << ", " << params(4) << ", " << params(5) << "]";
    
//     // Update cost for next iteration
//     prev_cost = curr_cost;
//   }
  
//   // Check if terminated due to max iterations
//   if (!converged) {
//     CLOG(WARNING, "lidar.localization_daicp") 
//         << "  Gauss-Newton reached maximum iterations (" << max_gn_iter << ") without convergence";
//     termination_reason = "MAX_ITERATIONS";
//   }
  
//   // Apply the final transformation to T_var
//   // current_transformation represents the total delta from identity
//   // Convert to SE(3) vector and apply as update
//   const Eigen::Matrix<double, 6, 1> delta_vec = lgmath::se3::tran2vec(current_transformation.matrix()); 
//   CLOG(DEBUG, "lidar.localization_daicp") 
//         << "  delta_vec before T_var->update [rx,ry,rz,tx,ty,tz]: [" 
//         << delta_vec(0) << ", " << delta_vec(1) << ", " << delta_vec(2) << ", "
//         << delta_vec(3) << ", " << delta_vec(4) << ", " << delta_vec(5) << "]";


//   // [consistent with STEAM]
//   //   Eigen::Matrix4d T_var_update = current_transformation.matrix() * T_var->value().matrix();
//   //   CLOG(DEBUG, "lidar.localization_daicp") 
//   //       << " T_var matrix (T_curr * T_var):\n" << T_var_update;

//   T_var->update(delta_vec);
  
//   // steam update
//   CLOG(DEBUG, "lidar.localization_daicp") 
//       << " T_var matrix (steam):\n" << T_var->value().matrix().cast<float>();

//   return true;
// }

lgmath::se3::Transformation LocalizationDAICPModule::paramsToTransformationMatrix(const Eigen::VectorXd& params) {
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

///////// End helper functions

void LocalizationDAICPModule::run_(QueryCache &qdata0, OutputCache &output,
                                 const Graph::Ptr &,
                                 const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (output.chain->isLocalized() && *qdata.loc_time > config_->target_loc_time && *qdata.pipeline_mode == tactic::PipelineMode::RepeatFollow) {
    CLOG(WARNING, "lidar.localization_icp") << "Skipping localization to save on compute. EMA val=" << *qdata.loc_time;
    return;
  }

  // Inputs
  // const auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.undistorted_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_v = *qdata.T_r_v_loc;  // used as prior
  const auto &T_v_m = *qdata.T_v_m_loc;
  // const auto &map_version = qdata.submap_loc->version();
  auto &point_map = qdata.submap_loc->point_cloud();

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;

  // clang-format off
  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r); T_s_r_var->locked() = true;
  const auto T_v_m_var = SE3StateVar::MakeShared(T_v_m); T_v_m_var->locked() = true;

  /// Create and add the T_robot_map variable, here m = vertex frame.
  const auto T_r_v_var = SE3StateVar::MakeShared(T_r_v);

  /// compound transform for alignment (sensor to point map transform)
  const auto T_m_s_eval = inverse(compose(T_s_r_var, compose(T_r_v_var, T_v_m_var)));

  /// Create T_m_s_var for direct point cloud alignment optimization
  const auto initial_T_m_s = T_m_s_eval->evaluate();
  auto T_m_s_var = SE3StateVar::MakeShared(initial_T_m_s);

  /// Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto map_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// the aligned_mat and aligned_norms_mat store the aligned points and normals which
  /// will be updated during the optimization
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// create kd-tree of the map
  CLOG(DEBUG, "lidar.localization_icp") << "Start building a kd-tree of the map.";
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(/* max leaf */ 10);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  /// perform initial alignment
  {
    const auto T_m_s = T_m_s_var->value().matrix().cast<float>();
    aligned_mat = T_m_s * query_mat;
    aligned_norms_mat = T_m_s * query_norms_mat;
  }

  using Stopwatch = common::timing::Stopwatch<>;
  std::vector<std::unique_ptr<Stopwatch>> timer;
  std::vector<std::string> clock_str;
  clock_str.push_back("Random Sample ...... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("KNN Search ......... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Point Filtering .... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Optimization ....... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Alignment .......... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Check Convergence .. ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Compute Covariance . ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));

  // ICP results
  EdgeTransform T_r_v_icp;
  float matched_points_ratio = 0.0;

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(4, 4);
  bool refinement_stage = false;
  int refinement_step = 0;

  CLOG(DEBUG, "lidar.localization_daicp") << "Start the Degeneracy-Aware ICP optimization loop.";
  // outer loop
  for (int step = 0;; step++) {
    // index of the corresponding points
    timer[0]->start();
    std::vector<std::pair<size_t, size_t>> sample_inds;
    sample_inds.resize(query_points.size());
    for (size_t i = 0; i < query_points.size(); i++) sample_inds[i].first = i; 
    timer[0]->stop();
    // find nearest neigbors and distances
    timer[1]->start();
    std::vector<float> nn_dists(sample_inds.size());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (size_t i = 0; i < sample_inds.size(); i++) {
      KDTreeResultSet result_set(1);
      result_set.init(&sample_inds[i].second, &nn_dists[i]);
      kdtree->findNeighbors(result_set, aligned_points[sample_inds[i].first].data, search_params);
    }
    timer[1]->stop();

    /// Filter point pairs based on distances metrics
    timer[2]->start();
    std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
    filtered_sample_inds.reserve(sample_inds.size());
    for (size_t i = 0; i < sample_inds.size(); i++) {
      if (nn_dists[i] < max_pair_d2) {
        // Check planar distance (only after a few steps for initial alignment)
        auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
                    point_map[sample_inds[i].second].getVector3fMap();
        float planar_dist = std::abs(
            diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap())
        );
        // check (1) first a few steps (2) planar distance is smaller than threshold (outlier rejection)
        if (step < first_steps || planar_dist < max_planar_d) {
          filtered_sample_inds.push_back(sample_inds[i]);
        }
      }
    }
    timer[2]->stop();

    /// Degeneracy-Aware ICP point-to-plane optimization
    timer[3]->start();
    /// ########################### Gauss-Newton solver ########################### ///
    int max_gn_iter = 3;
    double inner_tolerance = 1e-4;
    bool optimization_success = daGaussNewtonP2Plane(filtered_sample_inds, 
                                                     query_mat,
                                                     map_mat,
                                                     map_normals_mat,
                                                     T_m_s_var,
                                                     max_gn_iter,
                                                     inner_tolerance);

    if (!optimization_success) {
      CLOG(WARNING, "lidar.localization_daicp") << "Gauss-Newton optimization failed at step " << step;
      break;
    }
    /// ########################################################################### ///
    timer[3]->stop();

    /// Alignment, update aligned_mat and aligned_norms_mat
    timer[4]->start();
    {
      const auto T_m_s = T_m_s_var->value().matrix().cast<float>();
      aligned_mat = T_m_s * query_mat;
      aligned_norms_mat = T_m_s * query_norms_mat;
    }
    /// save the transformation results
    const auto T_m_s = T_m_s_var->value().matrix();
    if (step == 0)
      all_tfs = Eigen::MatrixXd(T_m_s);
    else {
      Eigen::MatrixXd temp(all_tfs.rows() + 4, 4);
      temp.topRows(all_tfs.rows()) = all_tfs;
      temp.bottomRows(4) = Eigen::MatrixXd(T_m_s);
      all_tfs = temp;
    }
    timer[4]->stop();

    /// Check convergence
    timer[5]->start();
    // Update variations
    if (step > 0) {
      float avg_tot = step == 1 ? 1.0 : (float)config_->averaging_num_steps;

      // Get last transformation variations
      Eigen::Matrix4d T2 = all_tfs.block<4, 4>(all_tfs.rows() - 4, 0);
      Eigen::Matrix4d T1 = all_tfs.block<4, 4>(all_tfs.rows() - 8, 0);
      Eigen::Matrix4d diffT = T2 * T1.inverse();
      Eigen::Matrix<double, 6, 1> diffT_vec = lgmath::se3::tran2vec(diffT);
      float dT_b = diffT_vec.block<3, 1>(0, 0).norm();
      float dR_b = diffT_vec.block<3, 1>(3, 0).norm();

      mean_dT += (dT_b - mean_dT) / avg_tot;
      mean_dR += (dR_b - mean_dR) / avg_tot;
    }

    // Refinement incremental
    if (refinement_stage) refinement_step++;

    // Stop condition
    if (!refinement_stage && step >= first_steps) {
      if ((step >= max_it - 1) || (mean_dT < config_->trans_diff_thresh &&
                                   mean_dR < config_->rot_diff_thresh)) {
        CLOG(DEBUG, "lidar.localization_daicp") << "Initial alignment takes " << step << " steps.";

        // enter the second refine stage
        refinement_stage = true;
        max_it = step + config_->refined_max_iter;
        // reduce the max distance
        max_pair_d = config_->refined_max_pairing_dist;
        max_pair_d2 = max_pair_d * max_pair_d;
        max_planar_d = config_->refined_max_planar_dist;
      }
    }
    timer[5]->stop();

    /// Last step
    timer[6]->start();
    if ((refinement_stage && step >= max_it - 1) ||
        (refinement_step > config_->averaging_num_steps &&
         mean_dT < config_->trans_diff_thresh &&
         mean_dR < config_->rot_diff_thresh)) {
      // Decode T_r_v from optimized T_m_s_var
      // T_m_s = T_m_v * T_v_r * T_r_s, so T_r_v = T_r_s * T_s_m * T_m_v
      const auto T_m_s_optimized = T_m_s_var->value();
      const auto T_s_m_optimized = T_m_s_optimized.inverse();
      const auto T_r_v_decoded = T_s_r.inverse() * T_s_m_optimized * T_v_m.inverse();
      
      // Create T_r_v_icp with dummy covariance
      Eigen::Matrix<double, 6, 6> dummy_cov = Eigen::Matrix<double, 6, 6>::Identity();
      dummy_cov.diagonal() << 0.001, 0.001, 0.001, 1e-6, 1e-6, 1e-6;  // [x,y,z,rx,ry,rz]
      T_r_v_icp = EdgeTransform(T_r_v_decoded, dummy_cov);
      matched_points_ratio = (float)filtered_sample_inds.size() / (float)sample_inds.size();
      //
      CLOG(DEBUG, "lidar.localization_daicp") << "Total number of steps: " << step << ", with matched ratio " << matched_points_ratio;
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "lidar.localization_daicp") << "DA-ICP did not converge to the specified threshold.";
        if (!refinement_stage) {
          CLOG(WARNING, "lidar.localization_daicp") << "DA-ICP did not enter refinement stage at all.";
        }
      }
      break;
    }
    timer[6]->stop();
  }

  /// Dump timing info
  CLOG(DEBUG, "lidar.localization_daicp") << "Dump timing info inside loop: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "lidar.localization_daicp") << clock_str[i] << timer[i]->count();
  }

  /// Outputs
  if (matched_points_ratio > config_->min_matched_ratio) {
    // update map to robot transform
    *qdata.T_r_v_loc = T_r_v_icp;
    // set success
    *qdata.loc_success = true;
  } else {
    CLOG(WARNING, "lidar.localization_daicp")
        << "Matched points ratio " << matched_points_ratio
        << " is below the threshold. DA-ICP is considered failed.";
    // no update to map to robot transform
    // set success
    *qdata.loc_success = false;
  }

}

}  // namespace lidar
}  // namespace vtr