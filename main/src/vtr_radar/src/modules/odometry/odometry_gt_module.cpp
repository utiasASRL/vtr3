// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file odometry_gt_module.cpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_gt_module.hpp"

#include "vtr_radar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace radar {

namespace {

template <class PointT>
void cart2pol(pcl::PointCloud<PointT> &point_cloud) {
  for (auto &p : point_cloud) {
    p.rho = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = std::atan2(std::sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = std::atan2(p.y, p.x);
  }
}

Eigen::Vector<double, 3> vec3Dto2D(const Eigen::Matrix<double, 6, 1> &se3_vec) {
  Eigen::Vector3d se2_vec;
  se2_vec << se3_vec(0), se3_vec(1), se3_vec(5);
  return se2_vec;
}

Eigen::Vector<double, 6> vec2Dto3D(const Eigen::Matrix<double, 3, 1> &se2_vec) {
  Eigen::Vector<double, 6> se3_vec = Eigen::Vector<double, 6>::Zero();
  se3_vec << se2_vec(0), se2_vec(1), 0, 0, 0, se2_vec(2);
  return se3_vec;
}

Eigen::Matrix<double, 6, 6> cov3Dto2D(const Eigen::Matrix<double, 12, 12> &se3_cov) {
  Eigen::Matrix<double, 6, 6> se2_cov = Eigen::Matrix<double, 6, 6>::Zero();
  // Extract xy block
  se2_cov.block<2, 2>(0, 0) = se3_cov.block<2, 2>(0, 0);
  // Extract yaw, vx, vy block
  se2_cov.block<3,3>(2,2) = se3_cov.block<3,3>(5,5);
  // Extract vyaw element
  se2_cov.block<1,1>(5,5) = se3_cov.block<1,1>(11,11);
  // Extract x, y to yaw, vx, vy cross-correlation blocks
  se2_cov.block<2,3>(0,2) = se3_cov.block<2,3>(0,5);
  se2_cov.block<3,2>(2,0) = se3_cov.block<3,2>(5,0);
  // Extract x, y to vyaw cross-correlation blocks
  se2_cov.block<2,1>(0,5) = se3_cov.block<2,1>(0,11);
  se2_cov.block<1,2>(5,0) = se3_cov.block<1,2>(11,0);
  // Extract yaw, vx, vy to vyaw cross-correlation blocks
  se2_cov.block<3,1>(2,5) = se3_cov.block<3,1>(5,11);
  se2_cov.block<1,3>(5,2) = se3_cov.block<1,3>(11,5);
  // Normalize for safety
  se2_cov = 0.5 * (se2_cov + se2_cov.transpose());
  return se2_cov;
}

Eigen::Matrix<double, 12, 12> cov2Dto3D(const Eigen::Matrix<double, 6, 6> &se2_cov) {
  // Initialize to small diagonal value to make valid covariance matrix
  Eigen::Matrix<double, 12, 12> se3_cov = 1e-12 * Eigen::Matrix<double, 12, 12>::Identity();
  // Insert xy block
  se3_cov.block<2, 2>(0, 0) = se2_cov.block<2, 2>(0, 0);
  // Insert yaw, vx, vy block
  se3_cov.block<3,3>(5,5) = se2_cov.block<3,3>(2,2);
  // Insert vyaw element
  se3_cov.block<1,1>(11,11) = se2_cov.block<1,1>(5,5);
  // Insert x, y to yaw, vx, vy cross-correlation blocks
  se3_cov.block<2,3>(0,5) = se2_cov.block<2,3>(0,2);
  se3_cov.block<3,2>(5,0) = se2_cov.block<3,2>(2,0);
  // Insert x, y to vyaw cross-correlation blocks
  se3_cov.block<2,1>(0,11) = se2_cov.block<2,1>(0,5);
  se3_cov.block<1,2>(11,0) = se2_cov.block<1,2>(5,0);
  // Insert yaw, vx, vy to vyaw cross-correlation blocks
  se3_cov.block<3,1>(5,11) = se2_cov.block<3,1>(2,5);
  se3_cov.block<1,3>(11,5) = se2_cov.block<1,3>(5,2);
  // Normalize for safety
  se3_cov = 0.5 * (se3_cov + se3_cov.transpose());
  return se3_cov;
}

// Overload definition for single state 3x3 input
Eigen::Matrix<double, 6, 6> cov2Dto3D(const Eigen::Matrix<double, 3, 3> &se2_cov) {
  // Initialize to small diagonal value to make valid covariance matrix
  Eigen::Matrix<double, 6, 6> se3_cov = 1e-12 * Eigen::Matrix<double, 6, 6>::Identity();
  // Insert xy block
  se3_cov.block<2, 2>(0, 0) = se2_cov.block<2, 2>(0, 0);
  // Insert yaw element
  se3_cov.block<1,1>(5,5) = se2_cov.block<1,1>(2,2);
  // Insert x, y to yaw cross-correlation blocks
  se3_cov.block<2,1>(0,5) = se2_cov.block<2,1>(0,2);
  se3_cov.block<1,2>(5,0) = se2_cov.block<1,2>(2,0);
  // Normalize for safety
  se3_cov = 0.5 * (se3_cov + se3_cov.transpose());
  return se3_cov;
}

}  // namespace

using namespace tactic;
using namespace steam;
using namespace steam::se2;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryGTModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // motion compensation
  config->use_vel_meas = node->declare_parameter<bool>(param_prefix + ".use_vel_meas", config->use_vel_meas);
  config->traj_num_extra_states = node->declare_parameter<int>(param_prefix + ".traj_num_extra_states", config->traj_num_extra_states);
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 3) {
    std::string err{"Qc diagonal malformed. Must be 3 elements!"};
    CLOG(ERROR, "radar.odometry_gt") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2];

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
  
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", config->verbose);
  config->max_iterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", config->max_iterations);
  config->huber_delta = node->declare_parameter<double>(param_prefix + ".huber_delta", config->huber_delta);
  config->cauchy_k = node->declare_parameter<double>(param_prefix + ".cauchy_k", config->cauchy_k);
  config->dopp_cauchy_k = node->declare_parameter<double>(param_prefix + ".dopp_cauchy_k", config->dopp_cauchy_k);
  config->dopp_meas_std = node->declare_parameter<double>(param_prefix + ".dopp_meas_std", config->dopp_meas_std);
  config->vel_fwd_std = node->declare_parameter<double>(param_prefix + ".vel_fwd_std", config->vel_fwd_std);
  config->vel_side_std = node->declare_parameter<double>(param_prefix + ".vel_side_std", config->vel_side_std);
  config->use_p2pl = node->declare_parameter<bool>(param_prefix + ".use_p2pl", false);
  config->normal_score_threshold = node->declare_parameter<double>(param_prefix + ".normal_score_threshold", 0.0);

  config->gyro_cov = node->declare_parameter<double>(param_prefix + ".gyro_cov", config->gyro_cov);
  config->estimate_gyro_bias = node->declare_parameter<bool>(param_prefix + ".estimate_gyro_bias", config->estimate_gyro_bias);
  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);
  config->max_trans_vel_diff = node->declare_parameter<float>(param_prefix + ".max_trans_vel_diff", config->max_trans_vel_diff);
  config->max_rot_vel_diff = node->declare_parameter<float>(param_prefix + ".max_rot_vel_diff", config->max_rot_vel_diff);
  config->max_trans_diff = node->declare_parameter<float>(param_prefix + ".max_trans_diff", config->max_trans_diff);
  config->max_rot_diff = node->declare_parameter<float>(param_prefix + ".max_rot_diff", config->max_rot_diff);
  
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OdometryGTModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    return;
  }

  if (!qdata.T_rad_world_gt || !qdata.v_rad_gt) {
    CLOG(ERROR, "radar.odometry_gt")
        << "Groundtruth pose or velocity not provided!";
    throw std::runtime_error("Groundtruth pose or velocity not provided!");
  }

  if (!qdata.sliding_map_odo) {
    // Initialize all variables
    CLOG(INFO, "radar.odometry_gt") << "First frame, simply return.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(*qdata.v_rad_gt);

    qdata.timestamp_odo_radar.emplace(*qdata.stamp);
    qdata.T_r_m_odo_radar.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo_radar.emplace(*qdata.v_rad_gt);
    // Initialize prior values
    qdata.T_r_m_odo_prior.emplace(lgmath::se3::Transformation());
    qdata.w_m_r_in_r_odo_prior.emplace(*qdata.v_rad_gt);
    qdata.cov_prior.emplace(1e-5 * Eigen::Matrix<double, 12, 12>::Identity());
    // Initialize timestamp equal to the end of the first frame
    const auto &query_points = *qdata.preprocessed_point_cloud;
    const auto compare_time = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
    qdata.timestamp_prior.emplace(std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp);

    // Groundtruth stuff
    qdata.T_rad_world_gt_prev.emplace(*qdata.T_rad_world_gt);
    qdata.v_rad_gt_prev.emplace(*qdata.v_rad_gt);

    //
    *qdata.odo_success = true;
    // clang-format on

    // This is the first odomety frame
    if(qdata.first_frame)
      *qdata.first_frame = true;
    else
      qdata.first_frame.emplace(true); // reset first frame - this is the first frame! Gyro could have run before though

    return;
  }

  CLOG(DEBUG, "radar.odometry_gt")
      << "Retrieve input data and setup evaluators.";

  // Inputs (these are all 3D to be consistent with other pipelines)
  const auto &scan_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &timestamp_odo = *qdata.timestamp_odo_radar; // use last data from radar scan msg (not gyro!)
  const auto &T_r_m_odo = *qdata.T_r_m_odo_radar; // use last data from radar scan msg (not gyro!)
  const auto &w_m_r_in_r_odo = *qdata.w_m_r_in_r_odo_radar; // use last data from radar scan msg (not gyro!)
  const auto &beta = *qdata.beta;
  const auto &T_r_m_odo_prior = *qdata.T_r_m_odo_prior;
  const auto &w_m_r_in_r_odo_prior = *qdata.w_m_r_in_r_odo_prior;
  const auto &cov_prior = *qdata.cov_prior;
  const auto &timestamp_prior = *qdata.timestamp_prior;
  auto &sliding_map_odo = *qdata.sliding_map_odo;
  auto &point_map = sliding_map_odo.point_cloud();
  const auto &T_rad_world_gt = *qdata.T_rad_world_gt;
  const auto &v_rad_gt = *qdata.v_rad_gt;
  const auto &T_rad_world_gt_prev = *qdata.T_rad_world_gt_prev;
  const auto &v_rad_gt_prev = *qdata.v_rad_gt_prev;

  // Extract 2D transforms
  const lgmath::se2::Transformation T_s_r_2d = T_s_r.toSE2();
  const lgmath::se2::Transformation T_r_m_odo_2d = T_r_m_odo.toSE2();
  const Eigen::Matrix<double, 3, 1> w_m_r_in_r_odo_2d = vec3Dto2D(w_m_r_in_r_odo);
  const lgmath::se2::Transformation T_r_m_odo_prior_2d = T_r_m_odo_prior.toSE2();
  const Eigen::Matrix<double, 3, 1> w_m_r_in_r_odo_prior_2d = vec3Dto2D(w_m_r_in_r_odo_prior);
  const Eigen::Matrix<double, 6, 6> cov_prior_2d = cov3Dto2D(cov_prior);
  const lgmath::se2::Transformation T_rad_world_gt_2d = T_rad_world_gt.toSE2();
  const Eigen::Matrix<double, 3, 1> v_rad_gt_2d = vec3Dto2D(v_rad_gt);

  // Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;

  // Set up timestamps
  // This is the general odometry timestamp
  // Should be the same as the above if only radar is used, but can be different if we also use gyro
  const auto &timestamp_odo_general = *qdata.timestamp_odo; 
  auto timestamp_odo_new = *qdata.stamp;
  Time scan_time(static_cast<int64_t>(scan_stamp));
  Time odo_time_general(static_cast<int64_t>(timestamp_odo_general));
  const auto compare_time = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
  //const auto frame_start_time = std::min_element(query_points.begin(), query_points.end(), compare_time)->timestamp;
  const auto frame_start_time = timestamp_prior;
  const auto frame_end_time = std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp;

  CLOG(DEBUG, "radar.odometry_gt") << "Timestamps of concern frame stamp: " << (scan_stamp - timestamp_prior) /1e9 << " timestamp_end " << (frame_end_time - timestamp_prior) /1e9;
  // Let's check if our odometry estimate already passed the time stamp of the radar scan
  // If this is the case, we want to estimate the odometry at this time, not at the time of the scan
  // This avoids jumping 'back' in time to the last radar scan, when we already extrapolated the state using gyro
  // Instead the radar scan is then incorporated as a past measurement to correct this extrapolated state
  // If the query stamp is more recent than the last odometry estimate, we proceed as usual
  if(odo_time_general.seconds() > scan_time.seconds())
  {
    CLOG(DEBUG, "radar.odometry_gt") << "Last odometry and gyro preintegration update is more recent than radar scan.";
    timestamp_odo_new = *qdata.timestamp_odo;
  }

  // clang-format off
  /// trajectory smoothing (these are 2D since we want to optimize in 2D)
  Evaluable<lgmath::se2::Transformation>::ConstPtr T_r_m_eval = nullptr;
  Evaluable<Eigen::Matrix<double, 3, 1>>::ConstPtr w_m_r_in_r_eval = nullptr;
  Evaluable<lgmath::se2::Transformation>::ConstPtr T_r_m_eval_extp = nullptr;
  Evaluable<Eigen::Matrix<double, 3, 1>>::ConstPtr w_m_r_in_r_eval_extp = nullptr;
  const_vel_se2::Interface::Ptr trajectory = const_vel_se2::Interface::MakeShared(config_->traj_qc_diag);
  lgmath::se2::Transformation T_r_m_odo_prior_new;
  Eigen::Matrix<double, 3, 1> w_m_r_in_r_odo_prior_new;
  Eigen::Matrix<double, 6, 6> cov_prior_new;
  std::vector<StateVarBase::Ptr> state_vars;

  // Set up main state variables
  const int64_t num_states = config_->traj_num_extra_states + 2;
  const int64_t time_diff = (frame_end_time - frame_start_time) / (num_states - 1);
  for (int i = 0; i < num_states; ++i) {
    // Load in explicit end_time in case there is small rounding issues
    const int64_t knot_time_stamp = (i == num_states - 1) ? frame_end_time : frame_start_time + i * time_diff;
    Time knot_time(static_cast<int64_t>(knot_time_stamp));
    const Eigen::Matrix<double, 6, 1> xi_m_r_in_r_odo((knot_time - timestamp_prior).seconds() * w_m_r_in_r_odo_prior);
    const auto T_r_m_odo_extp = (tactic::EdgeTransform(xi_m_r_in_r_odo) * T_r_m_odo_prior).toSE2();
    const std::string T_r_m_var_name = "T_r_m_" + std::to_string(i);
    const auto T_r_m_var = SE2StateVar::MakeShared(T_r_m_odo_extp, T_r_m_var_name);
    const std::string w_m_r_in_r_var_name = "w_m_r_in_r_" + std::to_string(i);
    const auto w_m_r_in_r_var = VSpaceStateVar<3>::MakeShared(w_m_r_in_r_odo_prior_2d, w_m_r_in_r_var_name);
    trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);
    state_vars.emplace_back(T_r_m_var);
    state_vars.emplace_back(w_m_r_in_r_var);
  }

  // Set up priors
  // CLOG(DEBUG, "radar.odometry_gt") << "Adding prior to trajectory.";
  // trajectory->addStatePrior(Time(frame_start_time), T_r_m_odo_prior_2d, w_m_r_in_r_odo_prior_2d, cov_prior_2d);

  // General radar odometry (at scan time)
  T_r_m_eval = trajectory->getPoseInterpolator(scan_time);
  w_m_r_in_r_eval = trajectory->getVelocityInterpolator(scan_time);

  // Odometry at extrapolated state (might be the same as above, but not necessarily, if we have gyro)
  Time extp_time(static_cast<int64_t>(timestamp_odo_new));
  T_r_m_eval_extp = trajectory->getPoseInterpolator(extp_time);
  w_m_r_in_r_eval_extp = trajectory->getVelocityInterpolator(extp_time);

  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE2StateVar::MakeShared(T_s_r_2d);
  T_s_r_var->locked() = true;
  /// compound transform for alignment (sensor to point map transform)
  const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval));

  /// Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto map_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// perform initial alignment
  CLOG(DEBUG, "radar.odometry_gt") << "Start initial alignment.";
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < query_points.size(); ++i) {
    aligned_mat.block<4, 1>(0, i) = query_mat.block<4, 1>(0, i);
  }
  if (beta != 0) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < query_points.size(); ++i) {
      const auto &qry_time = query_points[i].timestamp;
      const auto &up_chirp = query_points[i].up_chirp;
      const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(qry_time));
      const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);
      const auto w_m_s_in_s = w_m_s_in_s_intp_eval->evaluate().matrix().cast<float>();
      // Still create 3D v_m_s_in_s but just with a 0 z component
      const Eigen::Vector3f v_m_s_in_s = Eigen::Vector3f(w_m_s_in_s(0), w_m_s_in_s(1), 0.0f);
      Eigen::Vector3f abar = aligned_mat.block<3, 1>(0, i);
      abar.normalize();
      // If up chirp azimuth, subtract Doppler shift
      if (up_chirp) { 
        aligned_mat.block<3, 1>(0, i) -= beta * abar * abar.transpose() * v_m_s_in_s;
      } else {
        aligned_mat.block<3, 1>(0, i) += beta * abar * abar.transpose() * v_m_s_in_s;
      }
    }
  }
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < query_points.size(); i++) {
    const auto &qry_time = query_points[i].timestamp;
    const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
    const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
    // Transform to 3D for point manipulation
    const auto T_m_s = T_m_s_intp_eval->evaluate().toSE3().matrix().cast<float>();
    aligned_mat.block<4, 1>(0, i) = T_m_s * aligned_mat.block<4, 1>(0, i);
    aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
  }

  // ICP results
  EdgeTransform T_r_m_icp;
  float matched_points_ratio = 0.0;

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(3, 3);
  bool refinement_stage = false;
  int refinement_step = 0;
  bool solver_failed = false;

  CLOG(DEBUG, "radar.odometry_gt") << "Start the ICP optimization loop.";
//   for (int step = 0;; step++) {
//     /// sample points
//     std::vector<std::pair<size_t, size_t>> sample_inds;
//     sample_inds.resize(query_points.size());
//     // pick queries (for now just use all of them)
//     for (size_t i = 0; i < query_points.size(); i++) sample_inds[i].first = i;

//     /// filtering based on distances metrics
//     std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
//     filtered_sample_inds.reserve(sample_inds.size());
//     for (size_t i = 0; i < sample_inds.size(); i++) {
//       if (nn_dists[i] < max_pair_d2) {
//           filtered_sample_inds.push_back(sample_inds[i]);
//       }
//     }

//     // initialize problem
//     SlidingWindowFilter problem(config_->num_threads);

//     // add variables
//     for (const auto &var : state_vars)
//       problem.addStateVariable(var);

//     // add prior cost terms
//     trajectory->addPriorCostTerms(problem);

//     // optimize
//     GaussNewtonSolver::Params params;
//     params.verbose = config_->verbose;
//     params.reuse_previous_pattern = false;
//     params.max_iterations = (unsigned int)config_->max_iterations;

//     GaussNewtonSolver solver(problem, params);
//     try {
//       solver.optimize();
//     } catch(const std::runtime_error& e) {
//       CLOG(WARNING, "radar.odometry_gt") << "STEAM failed to solve, skipping frame. Error message: " << e.what();
//       solver_failed = true;
//     }

//     Covariance covariance(solver);

//     /// Alignment
// #pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
//     for (unsigned i = 0; i < query_points.size(); ++i) {
//       aligned_mat.block<4, 1>(0, i) = query_mat.block<4, 1>(0, i);
//     }
//     if (beta != 0) {
// #pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
//       for (unsigned i = 0; i < query_points.size(); ++i) {
//         const auto &qry_time = query_points[i].timestamp;
//         const auto &up_chirp = query_points[i].up_chirp;
//         const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(qry_time));
//         const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);
//         const auto w_m_s_in_s = w_m_s_in_s_intp_eval->evaluate().matrix().cast<float>();
//         // Still create 3D v_m_s_in_s but just with a 0 z component
//         const Eigen::Vector3f v_m_s_in_s = Eigen::Vector3f(w_m_s_in_s(0), w_m_s_in_s(1), 0.0f);
//         Eigen::Vector3f abar = aligned_mat.block<3, 1>(0, i);
//         abar.normalize();
//         if (up_chirp) {
//           aligned_mat.block<3, 1>(0, i) -= beta * abar * abar.transpose() * v_m_s_in_s;
//         } else {
//           aligned_mat.block<3, 1>(0, i) += beta * abar * abar.transpose() * v_m_s_in_s;
//         }
//       }
//     }
// #pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
//     for (unsigned i = 0; i < query_points.size(); i++) {
//       const auto &qry_time = query_points[i].timestamp;
//       const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
//       const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));

//       // Transform to 3D for point manipulation
//       const auto T_m_s = T_m_s_intp_eval->evaluate().toSE3().matrix().cast<float>();
//       aligned_mat.block<4, 1>(0, i) = T_m_s * aligned_mat.block<4, 1>(0, i);
//       aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
//     }

//     // Update all result matrices
//     const auto T_m_s = T_m_s_eval->evaluate().matrix();
//     if (step == 0)
//       all_tfs = Eigen::MatrixXd(T_m_s);
//     else {
//       Eigen::MatrixXd temp(all_tfs.rows() + 3, 3);
//       temp.topRows(all_tfs.rows()) = all_tfs;
//       temp.bottomRows(3) = Eigen::MatrixXd(T_m_s);
//       all_tfs = temp;
//     }

//     /// Check convergence
//     // Update variations
//     if (step > 0) {
//       float avg_tot = step == 1 ? 1.0 : (float)config_->averaging_num_steps;

//       // Get last transformation variations
//       Eigen::Matrix3d T2 = all_tfs.block<3, 3>(all_tfs.rows() - 3, 0);
//       Eigen::Matrix3d T1 = all_tfs.block<3, 3>(all_tfs.rows() - 6, 0);
//       Eigen::Matrix3d diffT = T2 * T1.inverse();
//       Eigen::Matrix<double, 3, 1> diffT_vec = lgmath::se2::tran2vec(diffT);
//       float dT_b = diffT_vec.block<2, 1>(0, 0).norm();
//       float dR_b = fabs(diffT_vec(2));

//       mean_dT += (dT_b - mean_dT) / avg_tot;
//       mean_dR += (dR_b - mean_dR) / avg_tot;
//     }

//     // Refininement incremental
//     if (refinement_stage) refinement_step++;

//     // Stop condition
//     if (!refinement_stage && step >= first_steps) {
//       if ((step >= max_it - 1) || (mean_dT < config_->trans_diff_thresh &&
//                                    mean_dR < config_->rot_diff_thresh)) {
//         CLOG(DEBUG, "radar.odometry_gt") << "Initial alignment takes " << step << " steps.";

//         // enter the second refine stage
//         refinement_stage = true;

//         max_it = step + config_->refined_max_iter;

//         // reduce the max distance
//         max_pair_d = config_->refined_max_pairing_dist;
//         max_pair_d2 = max_pair_d * max_pair_d;
//       }
//     }

//     /// Last step
//     if ((refinement_stage && step >= max_it - 1) ||
//         (refinement_step > config_->averaging_num_steps &&
//          mean_dT < config_->trans_diff_thresh &&
//          mean_dR < config_->rot_diff_thresh) || 
//          solver_failed) {
//       // result
//       Eigen::Matrix<double, 3, 3> T_r_m_cov = Eigen::Matrix<double, 3, 3>::Identity();
//       T_r_m_cov = trajectory->getCovariance(covariance, Time(static_cast<int64_t>(timestamp_odo_new))).block<3, 3>(0, 0);
//       // Save 3D edge transform result
//       Eigen::Matrix<double, 6, 6> T_r_m_cov_3d = cov2Dto3D(T_r_m_cov);
//       T_r_m_icp = EdgeTransform(T_r_m_eval_extp->value().toSE3(), T_r_m_cov_3d);

//       // Marginalize out all but last 2 states for prior
//       std::vector<StateVarBase::Ptr> state_vars_marg;
//       for (int i = 0; i < num_states*2 - 2; ++i) {
//         state_vars_marg.push_back(state_vars[i]);
//       }
//       problem.marginalizeVariable(state_vars_marg);
//       params.max_iterations = 1; // Only one iteration for marginalization
//       GaussNewtonSolver solver_marg(problem, params);
//       solver_marg.optimize();
//       Covariance covariance_marg(solver_marg);
//       T_r_m_odo_prior_new =  trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->pose()->evaluate();
//       w_m_r_in_r_odo_prior_new = trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->velocity()->evaluate();
//       cov_prior_new = trajectory->getCovariance(covariance_marg, Time(static_cast<int64_t>(frame_end_time))).block<6, 6>(0, 0);
//       cov_prior_new = 0.5 * (cov_prior_new + cov_prior_new.transpose());

//       //
//       matched_points_ratio = (float)filtered_sample_inds.size() / (float)sample_inds.size();
//       //
//       CLOG(DEBUG, "radar.odometry_gt") << "Total number of steps: " << step << ", with matched ratio " << matched_points_ratio;
//       if (mean_dT >= config_->trans_diff_thresh ||
//           mean_dR >= config_->rot_diff_thresh) {
//         CLOG(WARNING, "radar.odometry_gt") << "ICP did not converge to the specified threshold";
//         if (!refinement_stage) {
//           CLOG(WARNING, "radar.odometry_gt") << "ICP did not enter refinement stage at all.";
//         }
//       }
//       break;
//     }
//   }

  // Outputs
  const auto &w_m_r_in_r_eval_ = trajectory->getVelocityInterpolator(Time(static_cast<int64_t>(scan_stamp)))->evaluate().matrix();
  const auto vel_diff = w_m_r_in_r_eval_ - w_m_r_in_r_odo_2d;

  const auto T_r_m_prev = T_r_m_odo_2d;
  const auto T_r_m_query = T_r_m_eval->value();
  const auto diff_T = (T_r_m_query.inverse() * T_r_m_prev).vec();
  
  CLOG(DEBUG, "radar.odometry_gt") << "Current transformation difference: " << diff_T.transpose();
  CLOG(DEBUG, "radar.odometry_gt") << "Current velocity difference: " << vel_diff.transpose();

  // undistort the preprocessed pointcloud to eval state (at query timestamp)
  const auto T_s_m = T_m_s_eval->evaluate().toSE3().matrix().inverse().cast<float>();
  aligned_mat = T_s_m * aligned_mat;
  aligned_norms_mat = T_s_m * aligned_norms_mat;

  auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
  cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
  qdata.undistorted_point_cloud = undistorted_point_cloud;

  // store trajectory info
  // odometry at radar scan
  *qdata.w_m_r_in_r_odo_radar = v_rad_gt;

  // odometry at extrapolated time
  *qdata.w_m_r_in_r_odo = v_rad_gt;

  // odometry at radar scan
  // *qdata.T_r_m_odo_radar = T_r_m_eval->value().toSE3();
  *qdata.T_r_m_odo_radar = T_rad_world_gt_2d.toSE3(); // use groundtruth at radar scan time
  *qdata.timestamp_odo_radar = scan_stamp;

  // odometry at extr. time
  // *qdata.T_r_m_odo = T_r_m_eval_extp->value().toSE3();
  *qdata.T_r_m_odo = T_rad_world_gt_2d.toSE3(); // use groundtruth at radar scan time
  *qdata.timestamp_odo = timestamp_odo_new;

  CLOG(DEBUG, "radar.odometry_gt") << "T_m_r is: " << qdata.T_r_m_odo->inverse().vec().transpose();
  CLOG(DEBUG, "radar.odometry_gt") << "w_m_r_in_r is: " << qdata.w_m_r_in_r_odo->transpose();
  //
  auto T_r_v_odo_prev = *qdata.T_r_v_odo;
  std::cout << "T_rad_world_gt_prev: " << T_rad_world_gt_prev.matrix() << std::endl;
  std::cout << "T_rad_world_gt: " << T_rad_world_gt.matrix() << std::endl;
  *qdata.T_r_v_odo = (T_rad_world_gt * T_rad_world_gt_prev.inverse()) * T_r_v_odo_prev;
  /// \todo double check that we can indeed treat m same as v for velocity
  *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
  *qdata.T_r_m_odo_prior = T_r_m_odo_prior_new.toSE3();
  *qdata.w_m_r_in_r_odo_prior = vec2Dto3D(w_m_r_in_r_odo_prior_new);
  *qdata.cov_prior = cov2Dto3D(cov_prior_new);
  *qdata.timestamp_prior = frame_end_time;

  // groundtruth stuff
  *qdata.T_rad_world_gt_prev = T_rad_world_gt;
  *qdata.v_rad_gt_prev = v_rad_gt;

  //
  *qdata.odo_success = true;
  CLOG(DEBUG, "radar.odometry_gt") << "Odometry successful. T_r_m_icp: " << T_r_m_icp;
  CLOG(DEBUG, "radar.odometry_gt") << "T_r_v_odo: " << *qdata.T_r_v_odo;
  // clang-format on
}

}  // namespace radar
}  // namespace vtr