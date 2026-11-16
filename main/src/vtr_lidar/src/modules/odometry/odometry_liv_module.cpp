/**
 * \file odometry_liv_module.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * LIV odometry: ICP point-to-plane + lidar intensity-image features, combined
 * in a single STEAM Gauss-Newton optimisation.
 *
 * Features: mutual nearest-neighbour descriptor matching against the sliding
 * landmark map (previous scan as fallback), MD-RANSAC (6-DoF) for outlier
 * rejection, and a 3-D point-to-point residual per inlier
 *     e = p_landmark - T_m_s(t) p_feature,
 *     W = intensity_feature_scale * (Sigma_landmark + R Sigma_feature R^T)^-1
 * The lidar term is the one of the lidar-only odometry, with its unit-less
 * weight (normal_score * n n^T): with use_visual = false this module adds no
 * feature term. intensity_feature_scale is a hand-tuned scalar that sets how
 * much the features count against the lidar points.
 */
#include "vtr_lidar/modules/odometry/odometry_liv_module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include "vtr_lidar/features/descriptor_matching.hpp"
#include "vtr_lidar/features/md_ransac.hpp"
#include "vtr_lidar/features/p2plane_batch_cost_term.hpp"
#include "vtr_lidar/features/range_error_eval.hpp"
#include "vtr_lidar/features/reproj_error_eval.hpp"
#include "vtr_lidar/utils/nanoflann_utils.hpp"
#include "vtr_common/timing/stopwatch.hpp"

namespace vtr {
namespace lidar {

namespace {

/// An inlier match between a live feature and a landmark (of the sliding
/// feature map, or a feature of the previous scan placed in the map frame).
struct FeatureMatch {
  Eigen::Vector3d p_ref = Eigen::Vector3d::Zero();      /// landmark, map frame
  Eigen::Matrix3d cov_ref = Eigen::Matrix3d::Identity();  /// its covariance, map frame
  Eigen::Vector3d p2 = Eigen::Vector3d::Zero();         /// measured point, sensor frame at t2
  Eigen::Matrix3d cov2 = Eigen::Matrix3d::Identity();   /// its covariance, sensor frame
  Eigen::Vector2d y2 = Eigen::Vector2d::Zero();         /// projection of p2 (reproj residual)
  int64_t t2 = 0;                                       /// observation time (ns)
  int curr_idx = -1;                                    /// index of the live feature
  int landmark_idx = -1;                                /// index in the feature map, -1 = previous scan
};

/// Stage timers of one scan, as in OdometryICPModule. Each stopwatch
/// accumulates over all ICP steps of the scan; one DEBUG line per scan.
struct StageTimers {
  enum Stage { ASSOCIATION, KNN, FILTER, BUILD, SOLVE, ALIGN, CONVERGENCE,
               FINAL, OUTPUTS, NUM_STAGES };
  static constexpr const char* names[NUM_STAGES] = {
      "association", "knn", "filter", "build", "solve", "align",
      "convergence", "final", "outputs"};
  std::vector<std::unique_ptr<vtr::common::timing::Stopwatch<>>> sw;
  StageTimers() {
    for (int i = 0; i < NUM_STAGES; ++i)
      sw.emplace_back(std::make_unique<vtr::common::timing::Stopwatch<>>(false));
  }
  void start(const Stage s) { sw[s]->start(); }
  void stop(const Stage s) { sw[s]->stop(); }
  /// "association 1.2 | knn 3.4 | ... | total 56.7" [ms]
  std::string summary() const {
    std::ostringstream os;
    os << std::fixed << std::setprecision(1);
    double total = 0.0;
    for (int i = 0; i < NUM_STAGES; ++i) {
      const double ms = sw[i]->count<std::chrono::microseconds>() * 1e-3;
      total += ms;
      os << names[i] << " " << ms << " | ";
    }
    os << "total " << total;
    return os.str();
  }
};

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
  config->batch_lidar_cost = node->declare_parameter<bool>(param_prefix + ".batch_lidar_cost", config->batch_lidar_cost);

  // ── Visual (LIV) ──
  config->sigma_pixel = node->declare_parameter<double>(param_prefix + ".sigma_pixel", config->sigma_pixel);
  config->reproj_loss_sigma = node->declare_parameter<double>(param_prefix + ".reproj_loss_sigma", config->reproj_loss_sigma);
  config->ransac_max_iterations = node->declare_parameter<int>(param_prefix + ".ransac_max_iterations", config->ransac_max_iterations);
  config->ransac_inlier_threshold = node->declare_parameter<double>(param_prefix + ".ransac_inlier_threshold", config->ransac_inlier_threshold);
  config->ransac_inlier_range = node->declare_parameter<double>(param_prefix + ".ransac_inlier_range", config->ransac_inlier_range);
  config->ransac_confidence = node->declare_parameter<double>(param_prefix + ".ransac_confidence", config->ransac_confidence);
  config->ransac_gn_steps = node->declare_parameter<int>(param_prefix + ".ransac_gn_steps", config->ransac_gn_steps);
  config->ransac_refine_rounds = node->declare_parameter<int>(param_prefix + ".ransac_refine_rounds", config->ransac_refine_rounds);
  config->match_max_hamming = node->declare_parameter<double>(param_prefix + ".match_max_hamming", config->match_max_hamming);
  config->ransac_min_inliers = node->declare_parameter<int>(param_prefix + ".ransac_min_inliers", config->ransac_min_inliers);
  config->use_visual = node->declare_parameter<bool>(param_prefix + ".use_visual", config->use_visual);
  config->use_lidar = node->declare_parameter<bool>(param_prefix + ".use_lidar", config->use_lidar);

  // ── Feature map matching ──
  config->use_feature_map_matching = node->declare_parameter<bool>(param_prefix + ".use_feature_map_matching", config->use_feature_map_matching);
  config->map_match_min_matches = node->declare_parameter<int>(param_prefix + ".map_match_min_matches", config->map_match_min_matches);

  // ── Visual residual ──
  config->visual_residual = node->declare_parameter<std::string>(param_prefix + ".visual_residual", config->visual_residual);
  if (config->visual_residual != "p2p" && config->visual_residual != "reproj") {
    std::string err{"visual_residual must be 'p2p' or 'reproj'!"};
    CLOG(ERROR, "lidar.odometry_liv") << err;
    throw std::invalid_argument{err};
  }
  config->intensity_feature_scale = node->declare_parameter<double>(param_prefix + ".intensity_feature_scale", config->intensity_feature_scale);
  config->feature_cauchy_k = node->declare_parameter<double>(param_prefix + ".feature_cauchy_k", config->feature_cauchy_k);
  config->default_feature_sigma = node->declare_parameter<double>(param_prefix + ".default_feature_sigma", config->default_feature_sigma);

  // ── Range cost on visual matches (reproj residual only) ──
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
#if false
    // undistorted raw point cloud
    auto undistorted_raw_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.raw_point_cloud);
    cart2pol(*undistorted_raw_point_cloud);
    qdata.undistorted_raw_point_cloud = undistorted_raw_point_cloud;
#endif
    // undistorted preprocessed point cloud
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
      // seed the visual feature map with the raw first frame (treated as
      // observed at scan end, like the lidar map seeding)
      if (config_->use_visual) qdata.undistorted_intensity_features = feat_copy;
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

  /// Pose evaluators T_m_s(t) (sensor at time t -> map), created once per
  /// scan and reused in every ICP step: an evaluator reads the current knot
  /// values each time it is evaluated, and its interpolation coefficients
  /// depend only on t and the knot times. Filled lazily, so only call it
  /// outside parallel regions.
  using PoseEvalPtr = Evaluable<lgmath::se3::Transformation>::ConstPtr;
  std::unordered_map<int64_t, PoseEvalPtr> T_m_s_cache;
  const auto T_m_s_at = [&](const int64_t t) -> PoseEvalPtr {
    auto it = T_m_s_cache.find(t);
    if (it == T_m_s_cache.end())
      it = T_m_s_cache.emplace(t, inverse(compose(T_s_r_var, trajectory->getPoseInterpolator(Time(t))))).first;
    return it->second;
  };

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

  /// Align the query points (and normals) with the current trajectory
  /// estimate. With use_trajectory_estimation, every point is moved with the
  /// interpolated pose at its own firing time. Otherwise every point uses the
  /// pose at the end of the scan, so that pose is evaluated once and applied
  /// to all points in one matrix product (same result, no per-point
  /// interpolator).
  // The points of a scan share a few firing times (one per measurement
  // column): with use_trajectory_estimation, the interpolated pose is
  // evaluated once per firing time and applied to all points of that time.
  std::vector<int64_t> point_times;   // unique firing times, sorted
  std::vector<int> point_time_index;  // firing time of each query point
  if (config_->use_trajectory_estimation) {
    point_times.reserve(query_points.size());
    for (const auto& pt : query_points) point_times.push_back(pt.timestamp);
    std::sort(point_times.begin(), point_times.end());
    point_times.erase(std::unique(point_times.begin(), point_times.end()), point_times.end());
    point_time_index.resize(query_points.size());
    for (size_t i = 0; i < query_points.size(); ++i)
      point_time_index[i] = static_cast<int>(
          std::lower_bound(point_times.begin(), point_times.end(), query_points[i].timestamp) - point_times.begin());
  }
  std::vector<Eigen::Matrix4f> T_m_s_times(point_times.size());
  std::vector<PoseEvalPtr> T_m_s_point_evals;  // pose evaluator per firing time
  T_m_s_point_evals.reserve(point_times.size());
  for (const auto t : point_times) T_m_s_point_evals.push_back(T_m_s_at(t));
  const PoseEvalPtr T_m_s_end_eval = T_m_s_at(frame_end_time);

  const auto alignQueryPoints = [&]() {
    if (config_->use_trajectory_estimation) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (size_t k = 0; k < point_times.size(); k++)
        T_m_s_times[k] = T_m_s_point_evals[k]->evaluate().matrix().cast<float>();
#pragma omp parallel for schedule(static) num_threads(config_->num_threads)
      for (unsigned i = 0; i < query_points.size(); i++) {
        const Eigen::Matrix4f& T_m_s = T_m_s_times[point_time_index[i]];
        aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
        aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
      }
    } else {
      const Eigen::Matrix4f T_m_s = T_m_s_end_eval->evaluate().matrix().cast<float>();
      aligned_mat = T_m_s * query_mat;
      aligned_norms_mat = T_m_s * query_norms_mat;
    }
  };

  /// Initial alignment
  alignQueryPoints();

  // Time at which a feature is used, the same rule as for the lidar points:
  // with use_trajectory_estimation, its own firing time (the trajectory is
  // interpolated there); otherwise the end of the scan for every feature
  // (rigid scan). Used by MD-RANSAC, the feature residuals and the
  // undistortion of the features before the map update.
  const auto featureTime = [&](const int64_t t_feat) -> int64_t {
    return config_->use_trajectory_estimation
               ? std::clamp(t_feat, frame_start_time, frame_end_time)
               : frame_end_time;
  };

  /// Stage timers (DEBUG line "Timing [ms] ..." at the end of the scan)
  StageTimers timers;
  timers.start(StageTimers::ASSOCIATION);

  // ════════════════════════════════════════════════════════════════════════
  //  Feature association (done once before the ICP loop)
  //
  //  Mutual nearest neighbours between the descriptors of the live features
  //  and of the landmarks of the sliding feature map, then MD-RANSAC (6-DoF,
  //  independent of the motion prior: every match has its own dt, or the
  //  same dt for all of them with a rigid scan). If too few
  //  map matches survive, the features of the previous scan are used instead.
  //  Landmarks are timeless points in the odometry map frame. The previous
  //  scan's features are stored in the sensor frame at the end of that scan
  //  (= the start of this scan), so they become map-frame points with the
  //  prior pose.
  // ════════════════════════════════════════════════════════════════════════
  std::vector<FeatureMatch> feature_matches;
  std::vector<int> matched_landmark;  // per live feature, -1 = none
  std::string feature_source = "none";

  if (config_->use_visual && projector_ &&
      qdata.live_intensity_features.valid() &&
      !qdata.live_intensity_features->empty()) {
    const auto& curr_feat = *qdata.live_intensity_features;
    matched_landmark.assign(curr_feat.size(), -1);

    const bool curr_has_covs = curr_feat.hasCovs();
    const Eigen::Matrix3d default_cov =
        config_->default_feature_sigma * config_->default_feature_sigma *
        Eigen::Matrix3d::Identity();

    // sensor pose at the start of the scan (the previous scan's optimised end)
    const Eigen::Matrix4d T_s_m_start = T_s_r.matrix() * T_r_m_odo_prior.matrix();
    const Eigen::Matrix4d T_m_s_start = T_s_m_start.inverse();

    MdRansacParams ransac_params;
    ransac_params.max_iterations = config_->ransac_max_iterations;
    ransac_params.confidence = config_->ransac_confidence;
    ransac_params.inlier_px = config_->ransac_inlier_threshold;
    ransac_params.inlier_range = config_->ransac_inlier_range;
    ransac_params.min_inliers = config_->ransac_min_inliers;
    ransac_params.gn_steps = config_->ransac_gn_steps;
    ransac_params.refine_rounds = config_->ransac_refine_rounds;

    // matches the live features against a set of landmarks given in the map
    // frame and returns the MD-RANSAC inliers
    const auto associate =
        [&](const Eigen::Matrix<double, 3, Eigen::Dynamic>& ref_points,
            const std::vector<Eigen::Matrix3d>& ref_covs,
            const cv::Mat& ref_descriptors, const bool from_map,
            int& num_candidates) {
          std::vector<FeatureMatch> inliers;
          std::vector<FeatureMatch> candidates;
          std::vector<MdRansacMatch> ransac_matches;
          const auto pairs = matchMutualNN(curr_feat.descriptors, ref_descriptors,
                                           config_->match_max_hamming);
          candidates.reserve(pairs.size());
          ransac_matches.reserve(pairs.size());
          for (const auto& pair : pairs) {
            const int curr_idx = pair.first;
            const int ref_idx = pair.second;

            const Eigen::Vector3d p2 = curr_feat.points_3d.col(curr_idx);
            if (!p2.allFinite() || p2.norm() < 0.5) continue;
            const Eigen::Vector3d p_ref = ref_points.col(ref_idx);
            if (!p_ref.allFinite()) continue;
            // analytical projection of the live observation (the coordinate
            // system of projectPoint, not the raw LUT pixel)
            Eigen::Vector2d y2;
            if (!projector_->projectPoint(p2, y2)) continue;

            FeatureMatch m;
            m.p_ref = p_ref;
            m.cov_ref = ref_covs[ref_idx];
            m.p2 = p2;
            m.cov2 = curr_has_covs ? curr_feat.covs[curr_idx] : default_cov;
            m.y2 = y2;
            m.t2 = (curr_idx < static_cast<int>(curr_feat.timestamps.size()))
                       ? curr_feat.timestamps[curr_idx]
                       : frame_end_time;
            m.curr_idx = curr_idx;
            m.landmark_idx = from_map ? ref_idx : -1;

            MdRansacMatch r;
            // landmark in the sensor frame at the start of the scan
            r.p1 = (T_s_m_start * p_ref.homogeneous()).head<3>();
            r.p2 = p2;
            r.dt = static_cast<double>(featureTime(m.t2) - frame_start_time) * 1e-9;

            candidates.push_back(m);
            ransac_matches.push_back(r);
          }
          num_candidates = static_cast<int>(candidates.size());
          if (candidates.size() < 3) return inliers;

          const auto ransac =
              mdRansacSolve(ransac_matches, *projector_, ransac_params);
          if (!ransac.success) return inliers;
          inliers.reserve(ransac.inliers.size());
          for (const int i : ransac.inliers) inliers.push_back(candidates[i]);
          return inliers;
        };

    // ── (1) sliding feature map ──
    int num_map_candidates = 0;
    if (config_->use_feature_map_matching && qdata.sliding_feature_map_odo &&
        qdata.sliding_feature_map_odo->size() > 0) {
      const auto& feature_map = *qdata.sliding_feature_map_odo;
      std::vector<Eigen::Matrix3d> map_covs;
      map_covs.reserve(feature_map.size());
      for (const auto& lm : feature_map.landmarks()) map_covs.push_back(lm.cov);
      feature_matches = associate(feature_map.points(), map_covs,
                                  feature_map.descriptors(), true,
                                  num_map_candidates);
      if (!feature_matches.empty()) feature_source = "map";
      CLOG(DEBUG, "lidar.odometry_liv")
          << "Feature map matches: " << feature_matches.size() << " inliers / "
          << num_map_candidates << " (map size " << feature_map.size() << ")";
    }

    // ── (2) too few map inliers: the previous scan's features ──
    if (static_cast<int>(feature_matches.size()) < config_->map_match_min_matches &&
        qdata.prev_intensity_features.valid() &&
        *qdata.prev_intensity_features &&
        !(*qdata.prev_intensity_features)->empty()) {
      const auto& prev_feat = **qdata.prev_intensity_features;
      const Eigen::Matrix3d R_m_s_start = T_m_s_start.block<3, 3>(0, 0);
      const Eigen::Matrix<double, 3, Eigen::Dynamic> prev_points_m =
          (R_m_s_start * prev_feat.points_3d).colwise() +
          T_m_s_start.block<3, 1>(0, 3);
      std::vector<Eigen::Matrix3d> prev_covs_m(prev_feat.size(), default_cov);
      if (prev_feat.hasCovs())
        for (int i = 0; i < prev_feat.size(); ++i)
          prev_covs_m[i] =
              R_m_s_start * prev_feat.covs[i] * R_m_s_start.transpose();

      int num_prev_candidates = 0;
      auto prev_matches = associate(prev_points_m, prev_covs_m,
                                    prev_feat.descriptors, false,
                                    num_prev_candidates);
      CLOG(DEBUG, "lidar.odometry_liv")
          << "Previous scan matches: " << prev_matches.size() << " inliers / "
          << num_prev_candidates;
      if (prev_matches.size() > feature_matches.size()) {
        feature_matches = std::move(prev_matches);
        feature_source = "previous scan";
      }
    }

    for (const auto& m : feature_matches)
      if (m.landmark_idx >= 0) matched_landmark[m.curr_idx] = m.landmark_idx;
  }
  const int num_visual_inliers = static_cast<int>(feature_matches.size());
  timers.stop(StageTimers::ASSOCIATION);

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

  int num_steps = 0;
  for (int step = 0;; step++) {
    timers.stop(StageTimers::FINAL);  // end of the previous step
    num_steps = step + 1;
    timers.start(StageTimers::KNN);
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

    timers.stop(StageTimers::KNN);

    /// Filter
    timers.start(StageTimers::FILTER);
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

    timers.stop(StageTimers::FILTER);

    /// Build STEAM problem
    timers.start(StageTimers::BUILD);
    SlidingWindowFilter problem(config_->num_threads);

    for (const auto& var : state_vars)
      problem.addStateVariable(var);

    // Trajectory prior + smoothness cost terms
    trajectory->addPriorCostTerms(problem);

    // ── ICP point-to-plane cost terms ──
    if (config_->use_lidar && config_->batch_lidar_cost) {
      // Batched: the same residuals and weights as the per-point terms below,
      // summed per pose (one pose for a rigid scan, one per firing time with
      // trajectory estimation) and chained to the states once. The
      // correspondences are split into many small cost terms, since STEAM
      // parallelizes over cost terms.
      struct Corr { int64_t time; P2PlaneBatchCostTerm::Point point; };
      std::vector<Corr> corrs;
      corrs.reserve(filtered_sample_inds.size());
      for (const auto& ind : filtered_sample_inds) {
        if (point_map[ind.second].normal_score <= 0.0) continue;
        const Eigen::Vector3d nrm = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
        Corr c;
        c.time = (config_->use_trajectory_estimation) ? query_points[ind.first].timestamp : frame_end_time;
        c.point.query = query_mat.block<3, 1>(0, ind.first).cast<double>();
        c.point.reference = map_mat.block<3, 1>(0, ind.second).cast<double>();
        c.point.W = point_map[ind.second].normal_score * (nrm * nrm.transpose()) + 1e-5 * Eigen::Matrix3d::Identity();
        corrs.push_back(c);
      }
      // contiguous times per chunk -> few pose evaluators per chunk
      if (config_->use_trajectory_estimation)
        std::stable_sort(corrs.begin(), corrs.end(), [](const Corr& a, const Corr& b) { return a.time < b.time; });
      // Many small terms (about 32 points each, at most 256): STEAM hands the
      // cost terms to its threads in contiguous blocks (static OpenMP
      // schedule), and the lidar terms sit between the priors and the
      // feature terms. With only one term per thread, all lidar work would
      // fall into one or two threads' blocks.
      const size_t num_chunks = std::max<size_t>(1, std::min<size_t>(256, corrs.size() / 32));
      const size_t chunk_size = (corrs.size() + num_chunks - 1) / num_chunks;
      for (size_t start = 0; start < corrs.size(); start += chunk_size) {
        const size_t end = std::min(corrs.size(), start + chunk_size);
        std::vector<P2PlaneBatchCostTerm::Group> groups;
        for (size_t k = start; k < end; ++k) {
          if (groups.empty() || corrs[k].time != corrs[k - 1].time)
            groups.push_back({T_m_s_at(corrs[k].time), {}});
          groups.back().points.push_back(corrs[k].point);
        }
        problem.addCostTerm(P2PlaneBatchCostTerm::MakeShared(std::move(groups), config_->remove_orientation));
      }
    } else if (config_->use_lidar) {
    // per-point cost terms (one steam evaluator graph per point)
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

    // ── Intensity-image feature cost terms ──
    if (config_->use_visual && !feature_matches.empty() &&
        config_->visual_residual == "p2p") {
      // 3-D point-to-point: e = p_landmark - T_m_s(t) p_feature, with
      //   W = scale (Sigma_landmark + R_m_s Sigma_feature R_m_s^T)^-1
      // recomputed every step with the current rotation, scale =
      // intensity_feature_scale. The Cauchy scale is multiplied by sqrt(scale)
      // so that feature_cauchy_k acts on the Mahalanobis distance (steam
      // applies the loss to the whitened error, which the scale factor
      // multiplies by sqrt(scale)).
      const double scale = config_->intensity_feature_scale;
      BaseLossFunc::Ptr loss_func_feat = nullptr;
      if (config_->feature_cauchy_k > 0.0)
        loss_func_feat = CauchyLossFunc::MakeShared(config_->feature_cauchy_k * std::sqrt(scale));
      else
        loss_func_feat = L2LossFunc::MakeShared();

      int num_added = 0;
      for (const auto& m : feature_matches) {
        const auto T_m_s_t_eval = T_m_s_at(featureTime(m.t2));
        const Eigen::Matrix3d R_m_s = T_m_s_t_eval->evaluate().matrix().block<3, 3>(0, 0);

        const Eigen::Matrix3d cov = m.cov_ref + R_m_s * m.cov2 * R_m_s.transpose();
        Eigen::Matrix3d W = scale * cov.inverse();
        W = 0.5 * (W + W.transpose());
        if (!W.allFinite()) continue;
        const auto noise_model_feat = StaticNoiseModel<3>::MakeShared(W, NoiseType::INFORMATION);

        const auto error_func = p2p::p2pError(T_m_s_t_eval, m.p_ref, m.p2);
        const auto cost = WeightedLeastSqCostTerm<3>::MakeShared(error_func, noise_model_feat, loss_func_feat);
        problem.addCostTerm(cost);
        num_added++;
      }

      CLOG(DEBUG, "lidar.odometry_liv")
          << "Added " << num_added << " feature point-to-point cost terms ("
          << feature_source << ", step " << step << ")";
    } else if (config_->use_visual && projector_ && !feature_matches.empty()) {
      // 2-D reprojection (+ optional range) residual, kept for comparison:
      //   e = y2 − f(T_s_m(t2) · p_landmark)
      const Eigen::Matrix2d meas_cov =
          config_->sigma_pixel * config_->sigma_pixel * Eigen::Matrix2d::Identity();
      const auto noise_model_vis = StaticNoiseModel<2>::MakeShared(meas_cov);
      const auto loss_func_vis = CauchyLossFunc::MakeShared(config_->reproj_loss_sigma);

      const Eigen::Matrix<double, 1, 1> range_cov =
          Eigen::Matrix<double, 1, 1>::Constant(config_->sigma_range * config_->sigma_range);
      const auto noise_model_range = StaticNoiseModel<1>::MakeShared(range_cov);
      const auto loss_func_range = CauchyLossFunc::MakeShared(config_->range_loss_scale);

      for (const auto& m : feature_matches) {
        auto T_r_m_t_eval = trajectory->getPoseInterpolator(Time(featureTime(m.t2)));
        auto T_s_m_eval = compose(T_s_r_var, T_r_m_t_eval);

        auto error_func = ReprojErrorEval::MakeShared(T_s_m_eval, m.p_ref, m.y2, projector_);
        auto cost = WeightedLeastSqCostTerm<2>::MakeShared(error_func, noise_model_vis, loss_func_vis);
        problem.addCostTerm(cost);

        // Range error: e = ||p2|| - ||T_s_m(t2) · p_landmark||
        const double range2 = m.p2.norm();
        if (config_->use_range_cost && range2 > 0.0 && std::isfinite(range2)) {
          auto range_error_func = RangeErrorEval::MakeShared(T_s_m_eval, m.p_ref, range2);
          auto range_cost = WeightedLeastSqCostTerm<1>::MakeShared(range_error_func, noise_model_range, loss_func_range);
          problem.addCostTerm(range_cost);
        }
      }

      CLOG(DEBUG, "lidar.odometry_liv")
          << "Added " << feature_matches.size()
          << " feature reprojection cost terms (" << feature_source
          << ", step " << step << ")";
    }

    timers.stop(StageTimers::BUILD);

    // ── Solve ──
    timers.start(StageTimers::SOLVE);
    GaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.max_iterations = config_->max_iterations;

    GaussNewtonSolver solver(problem, params);

    // // [NEW] check for the constructed Hessian matrix and gradient vector for degeneracy. 
    // Eigen::SparseMatrix<double> swf_hessian;
    // Eigen::VectorXd swf_gradient_v;
    // problem.buildGaussNewtonTerms(swf_hessian, swf_gradient_v);
    // CLOG(INFO, "lidar.odometry_liv") << "swf_hessian: " << swf_hessian.rows() << " x " << swf_hessian.cols()
    //                                  << " swf_gradient_v: size=" << swf_gradient_v.size();
    // // converted to normal dense matrix for operation. 
    // const Eigen::MatrixXd H_dense(swf_hessian);

    try {
      solver.optimize();
    } catch (const std::runtime_error& e) {
      CLOG(WARNING, "lidar.odometry_liv")
          << "STEAM failed: " << e.what();
      solver_failed = true;
    }

    Covariance covariance(solver);
    timers.stop(StageTimers::SOLVE);

    /// Alignment update
    timers.start(StageTimers::ALIGN);
    alignQueryPoints();

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

    timers.stop(StageTimers::ALIGN);

    /// Check convergence
    timers.start(StageTimers::CONVERGENCE);
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

    timers.stop(StageTimers::CONVERGENCE);

    /// Last step (result, marginalization for the next prior)
    timers.start(StageTimers::FINAL);
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
          << ", feature inliers: " << num_visual_inliers;
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

  timers.stop(StageTimers::FINAL);
  timers.start(StageTimers::OUTPUTS);

  // ── Health of the feature covariances ──
  // Median squared Mahalanobis distance of the feature residuals after the
  // last step. With correct covariances it follows a chi^2 distribution with
  // 3 dof, whose median is 2.37: a smaller value means feature_sigma_px is
  // too large (conservative), a larger one that it is too small.
  if (!feature_matches.empty()) {
    std::vector<double> maha_sq;
    maha_sq.reserve(feature_matches.size());
    for (const auto& m : feature_matches) {
      const Eigen::Matrix4d T_m_s_t = T_m_s_at(featureTime(m.t2))->evaluate().matrix();
      const Eigen::Matrix3d R_m_s = T_m_s_t.block<3, 3>(0, 0);
      const Eigen::Vector3d e = m.p_ref - (R_m_s * m.p2 + T_m_s_t.block<3, 1>(0, 3));
      const Eigen::Matrix3d cov = m.cov_ref + R_m_s * m.cov2 * R_m_s.transpose();
      const double d2 = e.dot(cov.ldlt().solve(e));
      if (std::isfinite(d2)) maha_sq.push_back(d2);
    }
    if (!maha_sq.empty()) {
      const auto mid = maha_sq.begin() + maha_sq.size() / 2;
      std::nth_element(maha_sq.begin(), mid, maha_sq.end());
      CLOG(INFO, "lidar.odometry_liv")
          << "Features (" << feature_source << "): " << feature_matches.size()
          << " inliers, median squared Mahalanobis distance " << *mid
          << " (2.37 if the covariances are right)";
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
#if false
    // store undistorted raw point cloud
    auto undistorted_raw_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.raw_point_cloud);
    if (config_->use_trajectory_estimation) {
      auto &raw_points = *undistorted_raw_point_cloud;
      auto points_mat = raw_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (unsigned i = 0; i < raw_points.size(); i++) {
        const auto &qry_time = raw_points[i].timestamp;
        const auto T_rintp_m_eval = trajectory->getPoseInterpolator(Time(qry_time));
        const auto T_s_sintp_eval = inverse(compose(T_s_r_var, compose(T_rintp_m_eval, T_m_s_eval)));
        const auto T_s_sintp = T_s_sintp_eval->evaluate().matrix().cast<float>();
        points_mat.block<4, 1>(0, i) = T_s_sintp * points_mat.block<4, 1>(0, i);
      }
    }
    cart2pol(*undistorted_raw_point_cloud);
    qdata.undistorted_raw_point_cloud = undistorted_raw_point_cloud;
#endif

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
    // do not undistort the pointcloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(query_points);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
#if false
    // do not undistort the raw pointcloud as well
    auto undistorted_raw_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.raw_point_cloud);
    cart2pol(*undistorted_raw_point_cloud);
    qdata.undistorted_raw_point_cloud = undistorted_raw_point_cloud;
#endif
    *qdata.odo_success = false;
  }

  // ── Features for the map maintenance, the next frame and localization ──
  // On success, every feature 3-D point is moved from its time featureTime(t)
  // with the optimized trajectory (its covariance is rotated the same way):
  //  * to the sensor frame at frame_end_time: these go to the visual map
  //    maintenance module, together with the landmark each one was matched
  //    to, and become the previous-scan features of the next frame (they are
  //    then expressed exactly at its frame_start_time, where the prior pose
  //    is);
  //  * to the sensor frame at the scan stamp: the frame of the undistorted
  //    lidar cloud, used with it by the localization.
  // With use_trajectory_estimation this is the motion undistortion of the
  // features; with a rigid scan featureTime(t) is frame_end_time, as for the
  // lidar points.
  // On failure, the map and next-frame features are not updated (the prior
  // pose and the previous-scan features both stay at the last successful
  // scan), and localization gets the features as measured, as it gets the
  // lidar points.
  if (qdata.live_intensity_features.valid() && config_->use_visual) {
    if (*qdata.odo_success) {
      auto feat_end = std::make_shared<IntensityFeatures>(*qdata.live_intensity_features);
      auto feat_stamp = std::make_shared<IntensityFeatures>(*qdata.live_intensity_features);
      const bool has_covs = feat_end->hasCovs();
      const auto T_r_m_end_eval = trajectory->getPoseInterpolator(Time(static_cast<int64_t>(frame_end_time)));
      const Eigen::Matrix4d T_s_m_end = compose(T_s_r_var, T_r_m_end_eval)->evaluate().matrix();
      const Eigen::Matrix4d T_s_m_stamp = T_m_s_eval->evaluate().matrix().inverse();
      for (int i = 0; i < feat_end->size(); ++i) {
        const Eigen::Vector3d p = feat_end->points_3d.col(i);
        if (!p.allFinite() || p.norm() < 0.1) continue;
        const bool has_time = i < static_cast<int>(feat_end->timestamps.size());
        const int64_t t_feat = has_time ? featureTime(feat_end->timestamps[i]) : frame_end_time;
        const Eigen::Matrix4d T_m_s_t = T_m_s_at(t_feat)->evaluate().matrix();
        const Eigen::Vector4d p_h = (Eigen::Vector4d() << p, 1.0).finished();
        const Eigen::Matrix4d T_send_st = T_s_m_end * T_m_s_t;
        const Eigen::Matrix4d T_sstamp_st = T_s_m_stamp * T_m_s_t;
        feat_end->points_3d.col(i) = (T_send_st * p_h).head<3>();
        feat_stamp->points_3d.col(i) = (T_sstamp_st * p_h).head<3>();
        if (has_covs) {
          const Eigen::Matrix3d R_end = T_send_st.block<3, 3>(0, 0);
          const Eigen::Matrix3d R_stamp = T_sstamp_st.block<3, 3>(0, 0);
          feat_end->covs[i] = R_end * feat_end->covs[i] * R_end.transpose();
          feat_stamp->covs[i] = R_stamp * feat_stamp->covs[i] * R_stamp.transpose();
        }
        if (has_time) {
          feat_end->timestamps[i] = frame_end_time;  // now expressed at scan end
          feat_stamp->timestamps[i] = query_stamp;   // now expressed at the stamp
        }
      }
      // undistorted features (sensor frame at scan end) for the visual map
      // maintenance module
      qdata.undistorted_intensity_features = feat_end;
      if (static_cast<int>(matched_landmark.size()) != feat_end->size())
        matched_landmark.assign(feat_end->size(), -1);
      qdata.feature_matched_landmark = matched_landmark;
      // use = to overwrite the value injected by the pipeline
      qdata.prev_intensity_features = feat_end;
      // undistorted features (sensor frame at the stamp) for localization
      qdata.stamp_intensity_features = feat_stamp;
    } else {
      qdata.stamp_intensity_features =
          std::make_shared<IntensityFeatures>(*qdata.live_intensity_features);
    }
  }
  timers.stop(StageTimers::OUTPUTS);
  CLOG(DEBUG, "lidar.odometry_liv")
      << "Timing [ms] (" << num_steps << " steps, "
      << query_points.size() << " points, " << num_visual_inliers
      << " features): " << timers.summary();
  // clang-format on
}

}  // namespace lidar
}  // namespace vtr
