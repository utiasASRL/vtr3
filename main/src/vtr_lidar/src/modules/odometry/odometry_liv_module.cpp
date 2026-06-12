/**
 * \file odometry_liv_module.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * LIV odometry: ICP point-to-plane + motion-distorted visual reprojection
 * error, combined in a single STEAM Gauss-Newton optimisation.
 */
#include "vtr_lidar/modules/odometry/odometry_liv_module.hpp"

#include "vtr_lidar/features/md_ransac.hpp"
#include "vtr_lidar/features/range_error_eval.hpp"
#include "vtr_lidar/features/reproj_error_eval.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

namespace {

template <class PointT>
void cart2pol(pcl::PointCloud<PointT>& point_cloud) {
  for (auto& p : point_cloud) {
    p.rho = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = std::atan2(std::sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = std::atan2(p.y, p.x);
  }
}

}  // namespace

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

// ═══════════════════════════════════════════════════════════════════════════
//  Config::fromROS
// ═══════════════════════════════════════════════════════════════════════════
auto OdometryLIVModule::Config::fromROS(
    const rclcpp::Node::SharedPtr& node,
    const std::string& param_prefix) -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  // ── Trajectory ──
  config->use_trajectory_estimation = node->declare_parameter<bool>(param_prefix + ".use_trajectory_estimation", config->use_trajectory_estimation);
  config->traj_num_extra_states = node->declare_parameter<int>(param_prefix + ".traj_num_extra_states", config->traj_num_extra_states);
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 6) {
    std::string err{"traj_qc_diag must have 6 elements!"};
    CLOG(ERROR, "lidar.odometry_liv") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2], qcd[3], qcd[4], qcd[5];

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

  config->gyro_cov = node->declare_parameter<double>(param_prefix + ".gyro_cov", config->gyro_cov);
  config->use_gyro = node->declare_parameter<bool>(param_prefix + ".use_gyro", config->use_gyro);
  config->remove_orientation = node->declare_parameter<bool>(param_prefix + ".remove_orientation", false);

  // ── Visual (LIV) ──
  config->sigma_pixel = node->declare_parameter<double>(param_prefix + ".sigma_pixel", config->sigma_pixel);
  config->reproj_loss_sigma = node->declare_parameter<double>(param_prefix + ".reproj_loss_sigma", config->reproj_loss_sigma);
  config->ransac_max_iterations = node->declare_parameter<int>(param_prefix + ".ransac_max_iterations", config->ransac_max_iterations);
  config->ransac_inlier_threshold = node->declare_parameter<double>(param_prefix + ".ransac_inlier_threshold", config->ransac_inlier_threshold);
  config->ransac_min_inliers = node->declare_parameter<int>(param_prefix + ".ransac_min_inliers", config->ransac_min_inliers);
  config->use_visual = node->declare_parameter<bool>(param_prefix + ".use_visual", config->use_visual);
  config->use_lidar = node->declare_parameter<bool>(param_prefix + ".use_lidar", config->use_lidar);

  // ── Range cost on visual matches ──
  config->use_range_cost = node->declare_parameter<bool>(param_prefix + ".use_range_cost", config->use_range_cost);
  config->sigma_range = node->declare_parameter<double>(param_prefix + ".sigma_range", config->sigma_range);
  config->range_loss_scale = node->declare_parameter<double>(param_prefix + ".range_loss_scale", config->range_loss_scale);

  // ── Kinematic regulation prior ──
  config->use_kinematic_regulation = node->declare_parameter<bool>(param_prefix + ".use_kinematic_regulation", config->use_kinematic_regulation);
  const auto kin_sigma = node->declare_parameter<std::vector<double>>(param_prefix + ".kinematic_regulation_sigma", std::vector<double>{0.05, 0.01, 0.02, 0.02});
  if (kin_sigma.size() != 4) {
    std::string err{"kinematic_regulation_sigma must have 4 elements!"};
    CLOG(ERROR, "lidar.odometry_liv") << err;
    throw std::invalid_argument{err};
  }
  for (int i = 0; i < 4; ++i)
    config->kinematic_regulation_sigma(i) = std::max(kin_sigma[i], 1e-9);

  // ── OusterProjector sensor metadata ──
  // These come from the JSON file flattened at node level.
  // The IntensityFeatureExtractionModule may have already declared them.
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

  // ── Success criteria ──
  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);
  config->max_trans_vel_diff = node->declare_parameter<float>(param_prefix + ".max_trans_vel_diff", config->max_trans_vel_diff);
  config->max_rot_vel_diff = node->declare_parameter<float>(param_prefix + ".max_rot_vel_diff", config->max_rot_vel_diff);
  config->max_trans_diff = node->declare_parameter<float>(param_prefix + ".max_trans_diff", config->max_trans_diff);
  config->max_rot_diff = node->declare_parameter<float>(param_prefix + ".max_rot_diff", config->max_rot_diff);

  // clang-format on
  return config;
}

// ═══════════════════════════════════════════════════════════════════════════
//  run_
// ═══════════════════════════════════════════════════════════════════════════
void OdometryLIVModule::run_(QueryCache& qdata0, OutputCache&,
                             const Graph::Ptr&,
                             const TaskExecutor::Ptr&) {
  auto& qdata = dynamic_cast<LidarQueryCache&>(qdata0);

  CLOG(DEBUG, "lidar.odometry_liv")
      << "LIV odometry: use_lidar=" << config_->use_lidar
      << " use_visual=" << config_->use_visual;

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
    CLOG(INFO, "lidar.odometry_liv") << "OusterProjector initialized for LIV.";
  }

  // ════════════════════════════════════════════════════════════════════════
  //  First frame handling
  // ════════════════════════════════════════════════════════════════════════
  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "lidar.odometry_liv") << "First frame, simply return.";
    // clang-format off
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;

    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    qdata.T_r_m_odo_prior.emplace(lgmath::se3::Transformation());
    qdata.w_m_r_in_r_odo_prior.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    qdata.cov_prior.emplace(1e-5 * Eigen::Matrix<double, 12, 12>::Identity());
    const auto& query_points = *qdata.preprocessed_point_cloud;
    const auto compare_time = [](const auto& a, const auto& b) { return a.timestamp < b.timestamp; };
    qdata.timestamp_prior.emplace(std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp);
    *qdata.odo_success = true;
    // clang-format on

    // Store current features as prev for next frame (use = to overwrite
    // any existing value injected by the pipeline)
    if (qdata.live_intensity_features.valid()) {
      auto feat_copy = std::make_shared<IntensityFeatures>(*qdata.live_intensity_features);
      qdata.prev_intensity_features = feat_copy;
    }
    return;
  }

  CLOG(DEBUG, "lidar.odometry_liv")
      << "Retrieve input data and setup evaluators.";

  // ════════════════════════════════════════════════════════════════════════
  //  Inputs
  // ════════════════════════════════════════════════════════════════════════
  auto& query_stamp = *qdata.stamp;
  const auto& query_points = *qdata.preprocessed_point_cloud;
  const auto& T_s_r = *qdata.T_s_r;
  auto& sliding_map_odo = *qdata.sliding_map_odo;
  auto& point_map = sliding_map_odo.point_cloud();

  // Prior parameters
  const auto& T_r_m_odo_prior = *qdata.T_r_m_odo_prior;
  const auto& w_m_r_in_r_odo_prior = *qdata.w_m_r_in_r_odo_prior;
  const auto& cov_prior = *qdata.cov_prior;
  const auto& timestamp_prior = *qdata.timestamp_prior;

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;

  // clang-format off
  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;

  /// Trajectory setup
  Evaluable<lgmath::se3::Transformation>::ConstPtr T_r_m_eval = nullptr;
  lgmath::se3::Transformation T_r_m_eval_initial;
  Evaluable<Eigen::Matrix<double, 6, 1>>::ConstPtr w_m_r_in_r_eval = nullptr;
  const_vel::Interface::Ptr trajectory = nullptr;
  std::vector<StateVarBase::Ptr> state_vars;
  std::vector<VSpaceStateVar<6>::Ptr> vel_state_vars;
  const int64_t num_states = config_->traj_num_extra_states + 2;
  lgmath::se3::Transformation T_r_m_odo_prior_new;
  Eigen::Matrix<double, 6, 1> w_m_r_in_r_odo_prior_new;
  Eigen::Matrix<double, 12, 12> cov_prior_new;

  if (query_stamp < timestamp_prior) {
    CLOG(WARNING, "lidar.odometry_liv") << "Timestamp diff is " << (query_stamp - timestamp_prior) << " ns";
    query_stamp = timestamp_prior;
  }

  // Set up timestamps
  int64_t frame_end_time;
  Time query_time = Time(static_cast<int64_t>(query_stamp));
  Time prev_time(static_cast<int64_t>(timestamp_prior));
  trajectory = const_vel::Interface::MakeShared(config_->traj_qc_diag);

  const auto compare_time = [](const auto& a, const auto& b) { return a.timestamp < b.timestamp; };
  const auto frame_start_time = timestamp_prior;
  frame_end_time = std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp;
  const int64_t time_diff = (frame_end_time - frame_start_time) / (num_states - 1);

  // Set up state variables
  for (int i = 0; i < num_states; ++i) {
    const int64_t knot_time_stamp = (i == num_states - 1) ? frame_end_time : frame_start_time + i * time_diff;
    Time knot_time(static_cast<int64_t>(knot_time_stamp));
    const Eigen::Matrix<double, 6, 1> xi_m_r_in_r_odo((knot_time - prev_time).seconds() * w_m_r_in_r_odo_prior);
    const auto T_r_m_odo_extp = tactic::EdgeTransform(xi_m_r_in_r_odo) * T_r_m_odo_prior;
    const auto T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo_extp);
    const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo_prior);
    trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);
    state_vars.emplace_back(T_r_m_var);
    state_vars.emplace_back(w_m_r_in_r_var);
    vel_state_vars.emplace_back(w_m_r_in_r_var);
  }

  // State prior
  trajectory->addStatePrior(Time(frame_start_time), T_r_m_odo_prior, w_m_r_in_r_odo_prior, cov_prior);

  // Eval state at query time
  T_r_m_eval = trajectory->getPoseInterpolator(query_time);
  w_m_r_in_r_eval = trajectory->getVelocityInterpolator(query_time);
  T_r_m_eval_initial = T_r_m_eval->value();

  /// Compound transform for alignment
  const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval));

  /// Initialize aligned points
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix views
  const auto map_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// Build KD-tree of the map
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(10);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  /// Initial alignment
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < query_points.size(); i++) {
    const auto& qry_time = (config_->use_trajectory_estimation) ? query_points[i].timestamp : frame_end_time;
    const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
    const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
    const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
    aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
    aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Visual feature matching + MD-RANSAC (done once before the ICP loop)
  // ════════════════════════════════════════════════════════════════════════
  std::vector<VisualMatch> visual_matches;
  std::vector<int> visual_inliers;
  int num_visual_inliers = 0;

  if (config_->use_visual && projector_ &&
      qdata.live_intensity_features.valid() &&
      qdata.prev_intensity_features.valid()) {

    const auto& curr_feat = *qdata.live_intensity_features;
    const auto& prev_feat_ptr = *qdata.prev_intensity_features;

    if (prev_feat_ptr && !prev_feat_ptr->empty() && !curr_feat.empty()) {
      // Match ORB features (current → previous)
      auto bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
      std::vector<std::vector<cv::DMatch>> knn_matches;
      bf_matcher->knnMatch(curr_feat.descriptors, prev_feat_ptr->descriptors,
                           knn_matches, 2);

      // Time handling (livo two-knot scheme): after each successful
      // optimization, the live features are motion-undistorted to
      // frame_end_time using the *optimized* trajectory before being stored
      // as prev_intensity_features (see end of run_). Treating p1 as
      // observed exactly at frame_start_time is therefore correct, not an
      // approximation, and all residuals span only the current scan
      // duration. (Undistorting with the optimized trajectory avoids the
      // feedback loop that undistorting with the prior velocity would
      // create.) If the previous frame failed, the raw features are stored
      // and the boundary assumption degrades gracefully to the old
      // approximation.

      // Apply ratio test and build VisualMatch list
      for (const auto& match_pair : knn_matches) {
        if (match_pair.size() < 2) continue;
        const auto& best = match_pair[0];
        const auto& second = match_pair[1];

        if (best.distance < 50.0f &&
            best.distance < 0.8f * second.distance) {
          const int curr_idx = best.queryIdx;
          const int prev_idx = best.trainIdx;

          // Validate 3D points
          const Eigen::Vector3d& p1 = prev_feat_ptr->points_3d.col(prev_idx);
          if (p1.norm() < 0.5) continue;

          const Eigen::Vector3d& p2 = curr_feat.points_3d.col(curr_idx);
          if (p2.norm() < 0.5) continue;

          // Analytical projections (must match projectPoint's coord system
          // because MD-RANSAC and ReprojErrorEval call projectPoint
          // internally — do NOT use raw LUT pixel coordinates here).
          Eigen::Vector2d y1_proj, y2_proj;
          if (!projector_->projectPoint(p1, y1_proj)) continue;
          if (!projector_->projectPoint(p2, y2_proj)) continue;

          VisualMatch vm;
          vm.p1 = p1;            // undistorted to frame_start_time (prev scan end)
          vm.p2 = p2;            // measured 3-D point at t2 (curr frame)
          vm.y1 = y1_proj;
          vm.y2 = y2_proj;

          // Feature timestamps (absolute, ns). p1 lives exactly at the scan
          // boundary (frame_start_time) thanks to the undistortion above, so
          // dt is non-negative and bounded by scan_duration.
          vm.timestamp_1 = frame_start_time;
          vm.timestamp_2 = (curr_idx < static_cast<int>(curr_feat.timestamps.size()))
                               ? curr_feat.timestamps[curr_idx]
                               : frame_end_time;

          vm.dt = static_cast<double>(vm.timestamp_2 - vm.timestamp_1) * 1e-9;
          if (vm.dt < 1e-6) vm.dt = 1e-6;  // safety floor

          vm.prev_idx = prev_idx;
          vm.curr_idx = curr_idx;

          visual_matches.push_back(vm);
        }
      }

      CLOG(DEBUG, "lidar.odometry_liv")
          << "Visual matches: " << visual_matches.size();

      // Run MD-RANSAC
      if (visual_matches.size() >= 3) {
        Eigen::Matrix<double, 6, 1> ransac_omega;
        bool ransac_ok = mdRansacSolve(visual_matches, 
                                       *projector_,
                                       config_->ransac_max_iterations,
                                       config_->ransac_inlier_threshold,
                                       config_->ransac_min_inliers,
                                       visual_inliers, 
                                       ransac_omega);

        if (ransac_ok) {
          num_visual_inliers = static_cast<int>(visual_inliers.size());
          CLOG(DEBUG, "lidar.odometry_liv")
              << "MD-RANSAC: " << num_visual_inliers << " / "
              << visual_matches.size() << " inliers";
        } else {
          CLOG(WARNING, "lidar.odometry_liv")
              << "MD-RANSAC failed (" << visual_inliers.size()
              << " / " << visual_matches.size() << ")";
          visual_inliers.clear();
        }
      }
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  ICP optimisation loop (with visual cost terms)
  // ════════════════════════════════════════════════════════════════════════
  EdgeTransform T_r_m_icp;
  float matched_points_ratio = 0.0;

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(4, 4);
  bool refinement_stage = false;
  int refinement_step = 0;
  bool solver_failed = false;

  CLOG(DEBUG, "lidar.odometry_liv") << "Start the ICP+LIV optimization loop.";

  for (int step = 0;; step++) {
    /// Sample points
    std::vector<std::pair<size_t, size_t>> sample_inds;
    sample_inds.resize(query_points.size());
    for (size_t i = 0; i < query_points.size(); i++) sample_inds[i].first = i;

    /// Find nearest neighbors
    std::vector<float> nn_dists(sample_inds.size());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (size_t i = 0; i < sample_inds.size(); i++) {
      KDTreeResultSet result_set(1);
      result_set.init(&sample_inds[i].second, &nn_dists[i]);
      kdtree->findNeighbors(result_set, aligned_points[sample_inds[i].first].data, search_params);
    }

    /// Filter
    std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
    filtered_sample_inds.reserve(sample_inds.size());
    for (size_t i = 0; i < sample_inds.size(); i++) {
      if (nn_dists[i] < max_pair_d2) {
        auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
                    point_map[sample_inds[i].second].getVector3fMap();
        float planar_dist = std::abs(
            diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap()));
        if (step < first_steps || planar_dist < max_planar_d) {
          filtered_sample_inds.push_back(sample_inds[i]);
        }
      }
    }

    /// Build STEAM problem
    SlidingWindowFilter problem(config_->num_threads);

    for (const auto& var : state_vars)
      problem.addStateVariable(var);

    // Trajectory prior + smoothness cost terms
    trajectory->addPriorCostTerms(problem);

    // ── ICP point-to-plane cost terms ──
    if (config_->use_lidar) {
    auto loss_func_icp = L2LossFunc::MakeShared();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto& ind : filtered_sample_inds) {
      if (point_map[ind.second].normal_score <= 0.0) continue;
      Eigen::Vector3d nrm = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      Eigen::Matrix3d W(point_map[ind.second].normal_score * (nrm * nrm.transpose()) + 1e-5 * Eigen::Matrix3d::Identity());
      auto noise_model = StaticNoiseModel<3>::MakeShared(W, NoiseType::INFORMATION);

      const auto qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
      const bool rm_ori = config_->remove_orientation;

      auto error_func = [&]() -> Evaluable<Eigen::Matrix<double, 3, 1>>::Ptr {
        const auto& qry_time = (config_->use_trajectory_estimation) ? query_points[ind.first].timestamp : frame_end_time;
        const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
        const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
        return p2p::p2pError(T_m_s_intp_eval, ref_pt, qry_pt, rm_ori);
      }();

      auto cost = WeightedLeastSqCostTerm<3>::MakeShared(error_func, noise_model, loss_func_icp);
#pragma omp critical(odo_liv_add_p2p_error_cost)
      problem.addCostTerm(cost);
    }
    }  // end if (use_lidar)

    // ── Gyro cost terms ──
    if (config_->use_gyro && qdata.gyro_msgs) {
      const auto& T_s_r_gyro = *qdata.T_s_r_gyro;
      for (const auto& gyro_msg : *qdata.gyro_msgs) {
        const auto gyro_meas = Eigen::Vector3d(gyro_msg.angular_velocity.x, gyro_msg.angular_velocity.y, gyro_msg.angular_velocity.z);
        const rclcpp::Time gyro_stamp(gyro_msg.header.stamp);
        const auto gyro_stamp_time = static_cast<int64_t>(gyro_stamp.nanoseconds());
        Eigen::VectorXd gyro_meas_g(3);
        gyro_meas_g << gyro_meas(0), gyro_meas(1), gyro_meas(2);
        const Eigen::Matrix<double, 3, 1> gyro_meas_r = T_s_r_gyro.matrix().block<3, 3>(0, 0).transpose() * gyro_meas_g;
        auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(gyro_stamp_time));
        Eigen::Matrix<double, 6, 1> b_zero = Eigen::Matrix<double, 6, 1>::Zero();
        const auto bias = VSpaceStateVar<6>::MakeShared(b_zero);
        bias->locked() = true;
        const auto loss_func_gyro = L2LossFunc::MakeShared();
        const auto noise_model_gyro = StaticNoiseModel<3>::MakeShared(config_->gyro_cov * Eigen::Matrix<double, 3, 3>::Identity());
        const auto error_func = imu::GyroErrorEvaluator::MakeShared(w_m_r_in_r_intp_eval, bias, gyro_meas_r);
        const auto gyro_cost = WeightedLeastSqCostTerm<3>::MakeShared(error_func, noise_model_gyro, loss_func_gyro);
        problem.addCostTerm(gyro_cost);
      }
    }

    // ── Kinematic regulation cost terms ──
    // Penalizes lateral velocity, vertical velocity, roll rate and pitch
    // rate on every velocity knot (nonholonomic ground-vehicle prior).
    if (config_->use_kinematic_regulation) {
      Eigen::Matrix<double, 4, 6> H = Eigen::Matrix<double, 4, 6>::Zero();
      H(0, 1) = 1.0;  // lateral velocity: rho_y
      H(1, 2) = 1.0;  // vertical velocity: rho_z
      H(2, 3) = 1.0;  // roll rate: phi_x
      H(3, 4) = 1.0;  // pitch rate: phi_y

      Eigen::Matrix<double, 4, 4> kin_cov = Eigen::Matrix<double, 4, 4>::Zero();
      kin_cov.diagonal() = config_->kinematic_regulation_sigma.array().square().matrix();
      const auto noise_model_kin = StaticNoiseModel<4>::MakeShared(kin_cov);
      const auto loss_func_kin = L2LossFunc::MakeShared();

      for (const auto& w_var : vel_state_vars) {
        auto selected_velocity = vspace::mmult<4, 6>(w_var, H);
        auto error_func = vspace::vspace_error<4>(
            selected_velocity, Eigen::Matrix<double, 4, 1>::Zero());
        auto kin_cost = WeightedLeastSqCostTerm<4>::MakeShared(
            error_func, noise_model_kin, loss_func_kin);
        problem.addCostTerm(kin_cost);
      }
    }

    // ── Visual reprojection + range error cost terms (lidar intensity image) ──
    if (config_->use_visual && !visual_inliers.empty() && projector_) {
      const Eigen::Matrix2d meas_cov =
          config_->sigma_pixel * config_->sigma_pixel * Eigen::Matrix2d::Identity();
      const auto noise_model_vis = StaticNoiseModel<2>::MakeShared(meas_cov);
      const auto loss_func_vis = CauchyLossFunc::MakeShared(config_->reproj_loss_sigma);

      const Eigen::Matrix<double, 1, 1> range_cov =
          Eigen::Matrix<double, 1, 1>::Constant(config_->sigma_range * config_->sigma_range);
      const auto noise_model_range = StaticNoiseModel<1>::MakeShared(range_cov);
      const auto loss_func_range = CauchyLossFunc::MakeShared(config_->range_loss_scale);

      for (const int idx : visual_inliers) {
        const auto& m = visual_matches[idx];

        // When continuous-time estimation is off, evaluate the visual
        // residual using only the two knot poses (frame_start_time and
        // frame_end_time). This decouples the body velocity state from
        // the visual cost: a single image pair cannot disambiguate
        // "fast-then-slow" from constant velocity, so letting the visual
        // residual depend on per-feature t_meas leaves velocity
        // unobservable and the 24×24 Hessian rank-deficient → solver
        // oscillates frame-to-frame.
        const int64_t t_meas =
            config_->use_trajectory_estimation
                ? std::clamp(m.timestamp_2, frame_start_time, frame_end_time)
                : frame_end_time;
        
        // NOTE: we need to take care of the time offset later. This is the reason we use md_ransac
        // const int64_t t_meas = m.timestamp_2; 

        const Time t_m(static_cast<int64_t>(t_meas));

        // Build the transform that maps p1 (in sensor@prev) to sensor@curr:
        //   T = T_s_m(t_m) · T_m_s(t_prev)
        //   = compose(T_s_r, T_r_m(t_m)) · inverse(compose(T_s_r, T_r_m(t_prev)))
        auto T_r_m_t_eval = trajectory->getPoseInterpolator(t_m);
        auto T_r_m_prev_eval = trajectory->getPoseInterpolator(Time(frame_start_time));

        auto T_s_m_curr = compose(T_s_r_var, T_r_m_t_eval);
        auto T_m_s_prev = inverse(compose(T_s_r_var, T_r_m_prev_eval));
        auto T_scurr_sprev = compose(T_s_m_curr, T_m_s_prev);

        // Reprojection error: e = y2 - f(T · p1)
        auto error_func = ReprojErrorEval::MakeShared(
            T_scurr_sprev, m.p1, m.y2, projector_);

        auto cost = WeightedLeastSqCostTerm<2>::MakeShared(
            error_func, noise_model_vis, loss_func_vis);
        problem.addCostTerm(cost);

        // Range error: e = ||p2|| - ||T · p1|| (shares T_scurr_sprev)
        if (config_->use_range_cost) {
          const double range2 = m.p2.norm();
          if (range2 > 0.0 && std::isfinite(range2)) {
            auto range_error_func = RangeErrorEval::MakeShared(
                T_scurr_sprev, m.p1, range2);
            auto range_cost = WeightedLeastSqCostTerm<1>::MakeShared(
                range_error_func, noise_model_range, loss_func_range);
            problem.addCostTerm(range_cost);
          }
        }
      }

      CLOG(DEBUG, "lidar.odometry_liv")
          << "Added " << visual_inliers.size()
          << " visual reprojection cost terms (step " << step << ")";
    }

    // ── Solve ──
    GaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.max_iterations = config_->max_iterations;

    GaussNewtonSolver solver(problem, params);

    // [NEW] check for the constructed Hessian matrix and gradient vector for degeneracy. 
    Eigen::SparseMatrix<double> swf_hessian;
    Eigen::VectorXd swf_gradient_v;
    problem.buildGaussNewtonTerms(swf_hessian, swf_gradient_v);
    CLOG(INFO, "lidar.odometry_liv") << "swf_hessian: " << swf_hessian.rows() << " x " << swf_hessian.cols()
                                     << " swf_gradient_v: size=" << swf_gradient_v.size();
    // converted to normal dense matrix for operation. 
    const Eigen::MatrixXd H_dense(swf_hessian);

    try {
      solver.optimize();
    } catch (const std::runtime_error& e) {
      CLOG(WARNING, "lidar.odometry_liv")
          << "STEAM failed: " << e.what();
      solver_failed = true;
    }

    Covariance covariance(solver);

    /// Alignment update
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < query_points.size(); i++) {
      const auto& qry_time = (config_->use_trajectory_estimation) ? query_points[i].timestamp : frame_end_time;
      const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
      const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
      const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
      aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
      aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
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
      float avg_tot = step == 1 ? 1.0f : static_cast<float>(config_->averaging_num_steps);
      Eigen::Matrix4d T2 = all_tfs.block<4, 4>(all_tfs.rows() - 4, 0);
      Eigen::Matrix4d T1 = all_tfs.block<4, 4>(all_tfs.rows() - 8, 0);
      Eigen::Matrix4d diffT = T2 * T1.inverse();
      Eigen::Matrix<double, 6, 1> diffT_vec = lgmath::se3::tran2vec(diffT);
      float dT_b = diffT_vec.block<3, 1>(0, 0).norm();
      float dR_b = diffT_vec.block<3, 1>(3, 0).norm();
      mean_dT += (dT_b - mean_dT) / avg_tot;
      mean_dR += (dR_b - mean_dR) / avg_tot;
    }

    if (refinement_stage) refinement_step++;

    if (!refinement_stage && step >= first_steps) {
      if ((step >= max_it - 1) ||
          (mean_dT < config_->trans_diff_thresh &&
           mean_dR < config_->rot_diff_thresh)) {
        CLOG(DEBUG, "lidar.odometry_liv") << "Initial alignment took " << step << " steps.";
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
         mean_dR < config_->rot_diff_thresh) ||
        solver_failed) {
      // Result
      Eigen::Matrix<double, 6, 6> T_r_m_cov = Eigen::Matrix<double, 6, 6>::Identity();
      T_r_m_cov = trajectory->getCovariance(covariance, Time(static_cast<int64_t>(query_stamp))).block<6, 6>(0, 0);

      // Ensure the 6x6 covariance is positive definite and bounded before
      // storing on edge. With visual-only constraints the Hessian can be
      // near-singular, producing enormous covariance values that blow up
      // further during edge composition (T_r_m * T_vertex_this^-1).
      {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigsolver(T_r_m_cov);
        Eigen::Matrix<double, 6, 1> eigvals = eigsolver.eigenvalues();
        const double min_eig = eigvals.minCoeff();
        const double max_eig = eigvals.maxCoeff();
        // Cap: eigenvalues must be in [1e-8, 1.0]
        // 1.0 corresponds to ~1 m or ~1 rad std dev — already very uncertain
        constexpr double EIG_MIN = 1e-8;
        constexpr double EIG_MAX = 1.0;
        bool clamped = false;
        if (min_eig <= 0 || max_eig > EIG_MAX) {
          eigvals = eigvals.cwiseMax(EIG_MIN).cwiseMin(EIG_MAX);
          T_r_m_cov = eigsolver.eigenvectors() * eigvals.asDiagonal()
                      * eigsolver.eigenvectors().transpose();
          clamped = true;
        }
        CLOG(WARNING, "lidar.odometry_liv")
            << "[COV-CHECK] T_r_m_cov min_eig=" << min_eig
            << " max_eig=" << max_eig
            << (clamped ? " [CLAMPED]" : "");
      }

      T_r_m_icp = EdgeTransform(T_r_m_eval->value(), T_r_m_cov);

      // Marginalize
      std::vector<StateVarBase::Ptr> state_vars_marg;
      for (int i = 0; i < num_states * 2 - 2; ++i) {
        state_vars_marg.push_back(state_vars[i]);
      }
      problem.marginalizeVariable(state_vars_marg);
      params.max_iterations = 1;
      GaussNewtonSolver solver_marg(problem, params);
      solver_marg.optimize();
      Covariance covariance_marg(solver_marg);
      T_r_m_odo_prior_new = trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->pose()->evaluate();
      w_m_r_in_r_odo_prior_new = trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->velocity()->evaluate();
      cov_prior_new = trajectory->getCovariance(covariance_marg, Time(static_cast<int64_t>(frame_end_time))).block<12, 12>(0, 0);
      cov_prior_new = 0.5 * (cov_prior_new + cov_prior_new.transpose());

      matched_points_ratio = static_cast<float>(filtered_sample_inds.size()) / static_cast<float>(sample_inds.size());

      CLOG(DEBUG, "lidar.odometry_liv")
          << "Total steps: " << step
          << ", matched ratio: " << matched_points_ratio
          << ", visual inliers: " << num_visual_inliers;
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "lidar.odometry_liv") << "ICP+LIV did not converge.";
        if (!refinement_stage) {
          CLOG(WARNING, "lidar.odometry_liv") << "Did not enter refinement stage.";
        }
      }
      break;
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  //  Outputs
  // ════════════════════════════════════════════════════════════════════════
  bool estimate_reasonable = true;

  const auto& w_m_r_in_r_prev = *qdata.w_m_r_in_r_odo_prior;
  const auto& w_m_r_in_r_new = trajectory->getVelocityInterpolator(Time(static_cast<int64_t>(query_stamp)))->evaluate().matrix();
  const auto vel_diff = w_m_r_in_r_new - w_m_r_in_r_prev;
  const auto trans_vel_diff_norm = vel_diff.head<3>().norm();
  const auto rot_vel_diff_norm = vel_diff.tail<3>().norm();

  const auto T_r_m_query = T_r_m_eval->value();
  const auto diff_T = (T_r_m_query.inverse() * T_r_m_eval_initial).vec();
  const auto diff_T_trans = diff_T.head<3>().norm();
  const auto diff_T_rot = diff_T.tail<3>().norm();

  CLOG(DEBUG, "lidar.odometry_liv") << "Transform diff: " << diff_T.transpose();
  CLOG(DEBUG, "lidar.odometry_liv") << "Velocity diff: " << vel_diff.transpose();

  if (trans_vel_diff_norm > config_->max_trans_vel_diff ||
      rot_vel_diff_norm > config_->max_rot_vel_diff) {
    CLOG(WARNING, "lidar.odometry_liv")
        << "Velocity diff too large: trans=" << trans_vel_diff_norm
        << " rot=" << rot_vel_diff_norm;
    estimate_reasonable = false;
  }
  if (diff_T_trans > config_->max_trans_diff) {
    CLOG(WARNING, "lidar.odometry_liv") << "Translation diff too large: " << diff_T.transpose();
    estimate_reasonable = false;
  }
  if (diff_T_rot > config_->max_rot_diff) {
    CLOG(WARNING, "lidar.odometry_liv") << "Rotation diff too large: " << diff_T.transpose();
    estimate_reasonable = false;
  }

  if (matched_points_ratio > config_->min_matched_ratio &&
      estimate_reasonable && !solver_failed) {
    // Undistort the preprocessed pointcloud
    const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse().cast<float>();
    aligned_mat = T_s_m * aligned_mat;
    aligned_norms_mat = T_s_m * aligned_norms_mat;

    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;

    // Store trajectory info
    *qdata.w_m_r_in_r_odo = w_m_r_in_r_eval->value();
    *qdata.T_r_m_odo_prior = T_r_m_odo_prior_new;
    *qdata.w_m_r_in_r_odo_prior = w_m_r_in_r_odo_prior_new;
    *qdata.cov_prior = cov_prior_new;
    *qdata.timestamp_prior = frame_end_time;

    *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
    *qdata.T_r_v_odo = T_r_m_icp * sliding_map_odo.T_vertex_this().inverse();

    // Clamp the composed edge covariance. EdgeTransform operator* propagates
    // covariance via adjoints, which can produce huge / non-PD matrices when
    // T_vertex_this has large accumulated uncertainty. The pose-graph
    // NoiseModelGenerator will later feed this into StaticNoiseModel<6>,
    // which throws if not strictly PD.
    {
      Eigen::Matrix<double, 6, 6> edge_cov = qdata.T_r_v_odo->cov();
      // Symmetrize first to suppress numerical asymmetry from Adj·Σ·Adjᵀ
      edge_cov = 0.5 * (edge_cov + edge_cov.transpose());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigsolver(edge_cov);
      Eigen::Matrix<double, 6, 1> eigvals = eigsolver.eigenvalues();
      const double min_eig = eigvals.minCoeff();
      const double max_eig = eigvals.maxCoeff();
      constexpr double EDGE_EIG_MIN = 1e-8;
      constexpr double EDGE_EIG_MAX = 1.0;
      bool clamped = false;
      if (min_eig <= EDGE_EIG_MIN || max_eig > EDGE_EIG_MAX) {
        eigvals = eigvals.cwiseMax(EDGE_EIG_MIN).cwiseMin(EDGE_EIG_MAX);
        edge_cov = eigsolver.eigenvectors() * eigvals.asDiagonal()
                   * eigsolver.eigenvectors().transpose();
        edge_cov = 0.5 * (edge_cov + edge_cov.transpose());
        qdata.T_r_v_odo->setCovariance(edge_cov);
        clamped = true;
      }
      CLOG(WARNING, "lidar.odometry_liv")
          << "[EDGE-COV-CHECK] T_r_v_odo min_eig=" << min_eig
          << " max_eig=" << max_eig
          << (clamped ? " [CLAMPED]" : "");
    }

    *qdata.T_r_m_odo = T_r_m_eval->value();
    *qdata.timestamp_odo = query_stamp;

    *qdata.odo_success = true;
  } else {
    if (matched_points_ratio <= config_->min_matched_ratio) {
      CLOG(WARNING, "lidar.odometry_liv")
          << "Matched ratio " << matched_points_ratio << " below threshold.";
    }
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(query_points);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    *qdata.odo_success = false;
  }

  // ── Update prev_intensity_features for next frame ──
  // Use = to overwrite any existing value injected by the pipeline.
  //
  // Time handling:
  // on success, motion-undistort every feature 3-D point from its observation
  // time to frame_end_time using the optimized trajectory, so the next frame
  // can treat p1 as observed exactly at its frame_start_time. On failure,
  // fall back to storing the raw features (boundary approximation).
  if (qdata.live_intensity_features.valid()) {
    auto feat_copy = std::make_shared<IntensityFeatures>(*qdata.live_intensity_features);
    if (config_->use_visual && *qdata.odo_success) {
      const auto T_r_m_end_eval = trajectory->getPoseInterpolator(Time(static_cast<int64_t>(frame_end_time)));
      const Eigen::Matrix4d T_s_m_end = compose(T_s_r_var, T_r_m_end_eval)->evaluate().matrix();
      for (int i = 0; i < feat_copy->size(); ++i) {
        const Eigen::Vector3d p = feat_copy->points_3d.col(i);
        if (!p.allFinite() || p.norm() < 0.1) continue;
        const int64_t t_feat = (i < static_cast<int>(feat_copy->timestamps.size()))
            ? std::clamp(feat_copy->timestamps[i], frame_start_time, frame_end_time)
            : frame_end_time;
        const auto T_r_m_t_eval = trajectory->getPoseInterpolator(Time(t_feat));
        const Eigen::Matrix4d T_m_s_t = inverse(compose(T_s_r_var, T_r_m_t_eval))->evaluate().matrix();
        const Eigen::Vector4d p_h = (Eigen::Vector4d() << p, 1.0).finished();
        feat_copy->points_3d.col(i) = (T_s_m_end * T_m_s_t * p_h).head<3>();
        if (i < static_cast<int>(feat_copy->timestamps.size()))
          feat_copy->timestamps[i] = frame_end_time;  // now expressed at scan end
      }
    }
    qdata.prev_intensity_features = feat_copy;
  }
  // clang-format on
}

}  // namespace lidar
}  // namespace vtr
