/**
 * \file localization_liv_module.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * LIV localization: ICP point-to-plane + visual reprojection error,
 * combined in a single STEAM Gauss-Newton optimisation.
 *
 * This module is the localization counterpart of odometry_liv_module.
 * Key differences from odometry:
 *   - Uses OptimizationProblem with a single pose variable T_r_v
 *     (no trajectory estimation / interpolation / marginalization)
 *   - Visual matching is live_intensity_features vs map_intensity_features
 *     (recalled from teach vertex) rather than consecutive-frame matching
 *   - Map feature 3D points are in the vertex frame, so the transform
 *     for reprojection is T_s_v = compose(T_s_r, T_r_v)
 *   - No MD-RANSAC (single-pose model has no motion distortion);
 *     outlier rejection uses ratio-test + Cauchy robust loss
 */
#include "vtr_lidar/modules/localization/localization_liv_module.hpp"

#include "vtr_lidar/data_types/intensity_features.hpp"
#include "vtr_lidar/features/reproj_error_eval.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;
using namespace steam;
using namespace steam::se3;

// ═══════════════════════════════════════════════════════════════════════════
//  Config::fromROS
// ═══════════════════════════════════════════════════════════════════════════
auto LocalizationLIVModule::Config::fromROS(
    const rclcpp::Node::SharedPtr& node,
    const std::string& param_prefix) -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  // ── Prior ──
  config->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config->use_pose_prior);

  // ── ICP ──
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

  config->calc_gy_bias = node->declare_parameter<bool>(param_prefix + ".calc_gy_bias", config->calc_gy_bias);
  config->calc_gy_bias_thresh = node->declare_parameter<float>(param_prefix + ".calc_gy_bias_thresh", config->calc_gy_bias_thresh);

  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);

  // ── Visual (LIV) ──
  config->sigma_pixel = node->declare_parameter<double>(param_prefix + ".sigma_pixel", config->sigma_pixel);
  config->reproj_loss_sigma = node->declare_parameter<double>(param_prefix + ".reproj_loss_sigma", config->reproj_loss_sigma);
  config->use_visual = node->declare_parameter<bool>(param_prefix + ".use_visual", config->use_visual);
  config->use_lidar = node->declare_parameter<bool>(param_prefix + ".use_lidar", config->use_lidar);

  // ── OusterProjector sensor metadata ──
  config->pixels_per_column = node->declare_parameter<int>(param_prefix + ".pixels_per_column", config->pixels_per_column);
  config->columns_per_frame = node->declare_parameter<int>(param_prefix + ".columns_per_frame", config->columns_per_frame);
  config->lidar_origin_to_beam_origin_mm = node->declare_parameter<double>(param_prefix + ".lidar_origin_to_beam_origin_mm", config->lidar_origin_to_beam_origin_mm);

  // Helper: declare-or-get for shared global params (may already be declared).
  auto declare_or_get = [&](const std::string& name, auto default_val) {
    using T = decltype(default_val);
    if (node->has_parameter(name))
      return node->get_parameter(name).get_value<T>();
    return node->declare_parameter<T>(name, default_val);
  };

  // beam_altitude_angles
  config->beam_altitude_angles = declare_or_get(
      "beam_intrinsics.beam_altitude_angles", std::vector<double>{});
  if (config->beam_altitude_angles.empty())
    config->beam_altitude_angles = declare_or_get(
        "beam_altitude_angles", std::vector<double>{});

  // pixels_per_column / columns_per_frame from JSON
  {
    auto ppc = declare_or_get("lidar_data_format.pixels_per_column", int64_t(0));
    if (ppc == 0) ppc = declare_or_get("data_format.pixels_per_column", int64_t(0));
    if (ppc > 0) config->pixels_per_column = static_cast<int>(ppc);
    auto cpf = declare_or_get("lidar_data_format.columns_per_frame", int64_t(0));
    if (cpf == 0) cpf = declare_or_get("data_format.columns_per_frame", int64_t(0));
    if (cpf > 0) config->columns_per_frame = static_cast<int>(cpf);
  }

  // lidar_origin_to_beam_origin_mm from JSON
  {
    auto bo = declare_or_get("beam_intrinsics.lidar_origin_to_beam_origin_mm", 0.0);
    if (bo == 0.0) bo = declare_or_get("lidar_origin_to_beam_origin_mm", 0.0);
    if (bo != 0.0) config->lidar_origin_to_beam_origin_mm = bo;
  }

  // pixel_shift_by_row
  {
    auto ps = declare_or_get(
        "lidar_data_format.pixel_shift_by_row", std::vector<int64_t>{});
    if (ps.empty())
      ps = declare_or_get(
          "data_format.pixel_shift_by_row", std::vector<int64_t>{});
    config->pixel_shift_by_row.resize(ps.size());
    for (size_t i = 0; i < ps.size(); ++i)
      config->pixel_shift_by_row[i] = static_cast<int>(ps[i]);
  }

  config->u_shift = node->declare_parameter<int>(param_prefix + ".u_shift", config->u_shift);
  config->destagger = node->declare_parameter<bool>(param_prefix + ".destagger", config->destagger);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

// ═══════════════════════════════════════════════════════════════════════════
//  run_
// ═══════════════════════════════════════════════════════════════════════════
void LocalizationLIVModule::run_(QueryCache& qdata0, OutputCache& output,
                                 const Graph::Ptr&,
                                 const TaskExecutor::Ptr&) {
  auto& qdata = dynamic_cast<LidarQueryCache&>(qdata0);

  // ── Skip if we are already localized and taking too long ──
  if (output.chain->isLocalized() &&
      *qdata.loc_time > config_->target_loc_time &&
      *qdata.pipeline_mode == tactic::PipelineMode::RepeatFollow) {
    CLOG(WARNING, "lidar.localization_liv")
        << "Skipping localization to save compute. EMA val="
        << *qdata.loc_time;
    return;
  }

  // ── Lazy-initialize OusterProjector ──
  if (!projector_initialized_ && config_->use_visual) {
    auto proj_config = std::make_shared<OusterProjectorConfig>();
    proj_config->pixels_per_column = config_->pixels_per_column;
    proj_config->columns_per_frame = config_->columns_per_frame;
    proj_config->lidar_origin_to_beam_origin_mm =
        config_->lidar_origin_to_beam_origin_mm;
    proj_config->pixel_shift_by_row = config_->pixel_shift_by_row;
    proj_config->beam_altitude_angles = config_->beam_altitude_angles;
    proj_config->u_shift = config_->u_shift;
    proj_config->destagger = config_->destagger;
    projector_ = std::make_shared<OusterProjector>(proj_config);
    projector_initialized_ = true;
    CLOG(INFO, "lidar.localization_liv")
        << "OusterProjector initialized for localization LIV.";
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Inputs
  // ════════════════════════════════════════════════════════════════════════
  const auto& query_points = *qdata.undistorted_point_cloud;
  const auto& T_s_r = *qdata.T_s_r;
  const auto& T_r_v = *qdata.T_r_v_loc;  // prior from odometry
  const auto& T_v_m = *qdata.T_v_m_loc;
  auto& point_map = qdata.submap_loc->point_cloud();

  CLOG(DEBUG, "lidar.localization_liv")
      << "Localization LIV input: T_r_v (prior):\n" << T_r_v.matrix();

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;

  // clang-format off
  /// Create robot-to-sensor transform variable (fixed)
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r); T_s_r_var->locked() = true;
  /// Create vertex-to-map transform variable (fixed)
  const auto T_v_m_var = SE3StateVar::MakeShared(T_v_m); T_v_m_var->locked() = true;
  /// The single state variable: robot-to-vertex
  const auto T_r_v_var = SE3StateVar::MakeShared(T_r_v);

  /// Optional pose prior from odometry
  WeightedLeastSqCostTerm<6>::Ptr prior_cost_term = nullptr;
  if (config_->use_pose_prior && *qdata.odo_success && output.chain->isLocalized()) {
    auto loss_func = L2LossFunc::MakeShared();
    // Guard: ensure covariance is positive definite before creating noise model
    Eigen::Matrix<double, 6, 6> prior_cov = T_r_v.cov();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigsolver(prior_cov, Eigen::EigenvaluesOnly);
    if (eigsolver.eigenvalues().minCoeff() <= 0) {
      CLOG(WARNING, "lidar.localization_liv")
          << "T_r_v covariance is not positive definite (min eigenvalue: "
          << eigsolver.eigenvalues().minCoeff() << "). Using diagonal only.";
      prior_cov = prior_cov.diagonal().asDiagonal();
      // If diagonal is still not PD, add a small regularizer
      if (prior_cov.diagonal().minCoeff() <= 0) {
        prior_cov += 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
      }
    }
    auto noise_model = StaticNoiseModel<6>::MakeShared(prior_cov);
    auto T_r_v_meas = SE3StateVar::MakeShared(T_r_v); T_r_v_meas->locked() = true;
    auto error_func = tran2vec(compose(T_r_v_meas, inverse(T_r_v_var)));
    prior_cost_term = WeightedLeastSqCostTerm<6>::MakeShared(error_func, noise_model, loss_func);
  }

  /// Compound transform: sensor → point map  (T_m_s = T_m_v · T_v_r · T_r_s)
  const auto T_m_s_eval = inverse(compose(T_s_r_var, compose(T_r_v_var, T_v_m_var)));

  /// Compound transform: vertex → current sensor  (T_s_v = T_s_r · T_r_v)
  /// Used for reprojection: maps vertex-frame 3D points into current sensor frame
  const auto T_s_v_eval = compose(T_s_r_var, T_r_v_var);
  // clang-format on

  /// Initialize aligned points for matching (deep copy)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix views (shallow)
  const auto map_mat = point_map.getMatrixXfMap(
      4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(
      4, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(
      4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(
      4, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(
      4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(
      4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// Build KD-tree of the map
  CLOG(DEBUG, "lidar.localization_liv") << "Building kd-tree of the map.";
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(/* max leaf */ 10);
  auto kdtree =
      std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  /// Perform initial alignment
  {
    const auto T_m_s = T_m_s_eval->evaluate().matrix().cast<float>();
    aligned_mat = T_m_s * query_mat;
    aligned_norms_mat = T_m_s * query_norms_mat;
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Visual feature matching: live vs map (teach vertex)
  // ════════════════════════════════════════════════════════════════════════
  // Matched features: map 3D point (vertex frame) + live 2D observation
  struct VisualLocMatch {
    Eigen::Vector3d p_vertex;  ///< 3D point in vertex frame
    Eigen::Vector2d y_live;    ///< observed 2D projection in live sensor frame
  };
  std::vector<VisualLocMatch> visual_loc_matches;

  if (config_->use_visual && projector_ &&
      qdata.live_intensity_features.valid() &&
      qdata.map_intensity_features.valid()) {
    const auto& live_feat = *qdata.live_intensity_features;
    const auto& map_feat_vec = *qdata.map_intensity_features;

    if (!map_feat_vec.empty() && !live_feat.empty()) {
      // Concatenate all map features (typically one vertex worth)
      // We match LIVE (query) → MAP (train)
      const auto& map_feat = map_feat_vec[0];  // single vertex

      if (!map_feat.empty()) {
        auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        bf_matcher->knnMatch(live_feat.descriptors, map_feat.descriptors,
                             knn_matches, 2);

        for (const auto& match_pair : knn_matches) {
          if (match_pair.size() < 2) continue;
          const auto& best = match_pair[0];
          const auto& second = match_pair[1];

          // Ratio test + absolute distance threshold
          if (best.distance < 50.0f &&
              best.distance < 0.8f * second.distance) {
            const int live_idx = best.queryIdx;
            const int map_idx = best.trainIdx;

            // Validate 3D point
            const Eigen::Vector3d p_v = map_feat.points_3d.col(map_idx);
            if (p_v.norm() < 0.5) continue;

            // Analytically project the live 3D point to get y_live
            // in the same coordinate space that projectPoint uses.
            const Eigen::Vector3d p_live = live_feat.points_3d.col(live_idx);
            if (p_live.norm() < 0.5) continue;
            Eigen::Vector2d y_live_proj;
            if (!projector_->projectPoint(p_live, y_live_proj)) continue;

            VisualLocMatch vm;
            vm.p_vertex = p_v;
            vm.y_live = y_live_proj;
            visual_loc_matches.push_back(vm);
          }
        }

        CLOG(DEBUG, "lidar.localization_liv")
            << "Visual loc matches: " << visual_loc_matches.size()
            << " (live: " << live_feat.size()
            << ", map: " << map_feat.size() << ")";
      }
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  ICP + Visual optimisation loop
  // ════════════════════════════════════════════════════════════════════════
  EdgeTransform T_r_v_icp;
  float matched_points_ratio = 0.0;

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(4, 4);
  bool refinement_stage = false;
  int refinement_step = 0;

  CLOG(DEBUG, "lidar.localization_liv") << "Start the ICP+LIV optimization.";

  for (int step = 0;; step++) {
    /// Sample points
    std::vector<std::pair<size_t, size_t>> sample_inds;
    sample_inds.resize(query_points.size());
    for (size_t i = 0; i < query_points.size(); i++)
      sample_inds[i].first = i;

    /// Find nearest neighbours
    std::vector<float> nn_dists(sample_inds.size());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (size_t i = 0; i < sample_inds.size(); i++) {
      KDTreeResultSet result_set(1);
      result_set.init(&sample_inds[i].second, &nn_dists[i]);
      kdtree->findNeighbors(
          result_set, aligned_points[sample_inds[i].first].data,
          search_params);
    }

    /// Filter based on distance metrics
    std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
    filtered_sample_inds.reserve(sample_inds.size());
    for (size_t i = 0; i < sample_inds.size(); i++) {
      if (nn_dists[i] < max_pair_d2) {
        auto diff =
            aligned_points[sample_inds[i].first].getVector3fMap() -
            point_map[sample_inds[i].second].getVector3fMap();
        float planar_dist = std::abs(diff.dot(
            point_map[sample_inds[i].second].getNormalVector3fMap()));
        if (step < first_steps || planar_dist < max_planar_d) {
          filtered_sample_inds.push_back(sample_inds[i]);
        }
      }
    }

    /// Build STEAM problem (single-pose optimisation)
    OptimizationProblem problem(config_->num_threads);
    problem.addStateVariable(T_r_v_var);

    // Add prior cost term
    if (prior_cost_term != nullptr && *qdata.odo_success)
      problem.addCostTerm(prior_cost_term);

    // ── ICP point-to-plane cost terms ──
    if (config_->use_lidar) {
      auto loss_func_icp = L2LossFunc::MakeShared();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (const auto& ind : filtered_sample_inds) {
        if (point_map[ind.second].normal_score <= 0.0) continue;
        Eigen::Vector3d nrm =
            map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
        Eigen::Matrix3d W(
            point_map[ind.second].normal_score * (nrm * nrm.transpose()) +
            1e-5 * Eigen::Matrix3d::Identity());
        auto noise_model =
            StaticNoiseModel<3>::MakeShared(W, NoiseType::INFORMATION);

        const auto qry_pt =
            query_mat.block<3, 1>(0, ind.first).cast<double>();
        const auto ref_pt =
            map_mat.block<3, 1>(0, ind.second).cast<double>();

        const auto error_func = p2p::p2pError(T_m_s_eval, ref_pt, qry_pt);

        auto cost = WeightedLeastSqCostTerm<3>::MakeShared(
            error_func, noise_model, loss_func_icp);

#pragma omp critical(loc_liv_add_p2p_error_cost)
        problem.addCostTerm(cost);
      }
    }  // end if (use_lidar)

    // ── Visual reprojection error cost terms ──
    if (config_->use_visual && !visual_loc_matches.empty() && projector_) {
      const Eigen::Matrix2d meas_cov =
          config_->sigma_pixel * config_->sigma_pixel *
          Eigen::Matrix2d::Identity();
      const auto noise_model_vis =
          StaticNoiseModel<2>::MakeShared(meas_cov);
      const auto loss_func_vis =
          CauchyLossFunc::MakeShared(config_->reproj_loss_sigma);

      for (const auto& m : visual_loc_matches) {
        // ReprojErrorEval: e = y_live - f(T_s_v · p_vertex)
        // T_s_v_eval maps vertex-frame points into current sensor frame
        auto error_func = ReprojErrorEval::MakeShared(
            T_s_v_eval, m.p_vertex, m.y_live, projector_);

        auto cost = WeightedLeastSqCostTerm<2>::MakeShared(
            error_func, noise_model_vis, loss_func_vis);
        problem.addCostTerm(cost);
      }

      CLOG(DEBUG, "lidar.localization_liv")
          << "Added " << visual_loc_matches.size()
          << " visual reprojection cost terms (step " << step << ")";
    }

    // ── Solve ──
    GaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.max_iterations = config_->max_iterations;
    GaussNewtonSolver solver(problem, params);

    // [NEW] check for the constructed Hessian matrix and gradient vector for degeneracy. 
    Eigen::SparseMatrix<double> loc_hessian;
    Eigen::VectorXd loc_gradient_v;
    problem.buildGaussNewtonTerms(loc_hessian, loc_gradient_v);
    CLOG(INFO, "lidar.localization_liv") << "loc_hessian: " << loc_hessian.rows() << " x " << loc_hessian.cols()
                                         << " loc_gradient_v: size=" << loc_gradient_v.size();
    // converted to normal dense matrix for operation. 
    const Eigen::MatrixXd H_dense(loc_hessian);

    try {
      solver.optimize();
    } catch (std::runtime_error& e) {
      CLOG(WARNING, "lidar.localization_liv")
          << "Steam failed.\n e.what(): " << e.what();
      break;
    }
    Covariance covariance(solver);

    /// Alignment update
    {
      const auto T_m_s = T_m_s_eval->evaluate().matrix().cast<float>();
      aligned_mat = T_m_s * query_mat;
      aligned_norms_mat = T_m_s * query_norms_mat;
    }

    // Update all_tfs
    const auto T_m_s = T_m_s_eval->evaluate().matrix();
    if (step == 0)
      all_tfs = Eigen::MatrixXd(T_m_s);
    else {
      Eigen::MatrixXd temp(all_tfs.rows() + 4, 4);
      temp.topRows(all_tfs.rows()) = all_tfs;
      temp.bottomRows(4) = Eigen::MatrixXd(T_m_s);
      all_tfs = temp;
    }

    /// Check convergence
    if (step > 0) {
      float avg_tot =
          step == 1 ? 1.0f : static_cast<float>(config_->averaging_num_steps);
      Eigen::Matrix4d T2 = all_tfs.block<4, 4>(all_tfs.rows() - 4, 0);
      Eigen::Matrix4d T1 = all_tfs.block<4, 4>(all_tfs.rows() - 8, 0);
      Eigen::Matrix4d diffT = T2 * T1.inverse();
      Eigen::Matrix<double, 6, 1> diffT_vec =
          lgmath::se3::tran2vec(diffT);
      float dT_b = diffT_vec.block<3, 1>(0, 0).norm();
      float dR_b = diffT_vec.block<3, 1>(3, 0).norm();
      mean_dT += (dT_b - mean_dT) / avg_tot;
      mean_dR += (dR_b - mean_dR) / avg_tot;
    }

    if (refinement_stage) refinement_step++;

    // Stop condition: transition to refinement
    if (!refinement_stage && step >= first_steps) {
      if ((step >= max_it - 1) ||
          (mean_dT < config_->trans_diff_thresh &&
           mean_dR < config_->rot_diff_thresh)) {
        CLOG(DEBUG, "lidar.localization_liv")
            << "Initial alignment takes " << step << " steps.";
        refinement_stage = true;
        max_it = step + config_->refined_max_iter;
        max_pair_d = config_->refined_max_pairing_dist;
        max_pair_d2 = max_pair_d * max_pair_d;
        max_planar_d = config_->refined_max_planar_dist;
      }
    }

    /// Last step
    if ((refinement_stage && step >= max_it - 1) ||
        (refinement_step > config_->averaging_num_steps &&
         mean_dT < config_->trans_diff_thresh &&
         mean_dR < config_->rot_diff_thresh)) {
      // Result
      T_r_v_icp = EdgeTransform(T_r_v_var->value(),
                                covariance.query(T_r_v_var));
      matched_points_ratio = static_cast<float>(filtered_sample_inds.size()) /
                             static_cast<float>(sample_inds.size());
      CLOG(DEBUG, "lidar.localization_liv")
          << "Total steps: " << step
          << ", matched ratio: " << matched_points_ratio
          << ", visual loc matches: " << visual_loc_matches.size();
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "lidar.localization_liv")
            << "ICP+LIV did not converge to the specified threshold.";
        if (!refinement_stage) {
          CLOG(WARNING, "lidar.localization_liv")
              << "Did not enter refinement stage at all.";
        }
      }
      break;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Outputs
  // ════════════════════════════════════════════════════════════════════════
  if (matched_points_ratio > config_->min_matched_ratio) {
    // Update map-to-robot transform
    *qdata.T_r_v_loc = T_r_v_icp;
    *qdata.loc_success = true;

    // ── Gyro bias estimation (same as localization_icp_module) ──
    if (config_->calc_gy_bias && qdata.gyro_msgs) {
      const auto& query_stamp = *qdata.stamp;
      Eigen::Matrix4d T_r_v_loc = T_r_v_icp.matrix();
      double dt = (query_stamp - timestamp_prev_) * 1e-9;

      if (timestamp_prev_ == 0) {
        timestamp_prev_ = query_stamp;
        T_r_v_loc_prev_ = T_r_v_loc;
        return;
      }

      if (dt < config_->calc_gy_bias_thresh) {
        CLOG(DEBUG, "lidar.localization_liv")
            << "Not enough motion since last update. "
            << "Skip gyro bias estimation.";
        return;
      }

      Eigen::Matrix<double, 6, 1> phi = lgmath::se3::tran2vec(
          T_r_v_loc * T_r_v_loc_prev_.inverse());
      Eigen::Matrix<double, 6, 1> varpi_hat = phi / dt;
      Eigen::Vector3d w_hat = varpi_hat.tail<3>();

      Eigen::Vector3d gyro_avg = Eigen::Vector3d::Zero();
      for (const auto& msg : *qdata.gyro_msgs) {
        gyro_avg += Eigen::Vector3d(msg.angular_velocity.x,
                                    msg.angular_velocity.y,
                                    msg.angular_velocity.z);
      }
      gyro_avg /= static_cast<double>(qdata.gyro_msgs->size());

      Eigen::Vector3d gyro_bias_update = gyro_avg - w_hat;

      bool is_similar =
          ((*qdata.gyro_bias) - gyro_bias_update).norm() < 1e-3;
      if (!is_similar) {
        CLOG(WARNING, "lidar.localization_liv")
            << "Computed gyro bias update not similar to previous. "
            << "Skip update.";
        return;
      }

      *qdata.gyro_bias =
          0.8 * (*qdata.gyro_bias) + 0.2 * gyro_bias_update;
      CLOG(DEBUG, "lidar.localization_liv")
          << "Estimated gyro bias: "
          << (*qdata.gyro_bias).transpose();
      timestamp_prev_ = query_stamp;
      T_r_v_loc_prev_ = T_r_v_loc;
    }

  } else {
    CLOG(WARNING, "lidar.localization_liv")
        << "Matched points ratio " << matched_points_ratio
        << " is below the threshold. Localization considered failed.";
    *qdata.loc_success = false;
  }
}

}  // namespace lidar
}  // namespace vtr
