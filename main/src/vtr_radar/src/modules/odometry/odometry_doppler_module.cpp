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
 * \file odometry_icp_module.cpp
 * \author Yuchen Wu, Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_doppler_module.hpp"

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

}  // namespace

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryDopplerModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
  const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // motion compensation
  config->use_vel_meas = node->declare_parameter<bool>(param_prefix + ".use_vel_meas", config->use_vel_meas);
  config->traj_num_extra_states = node->declare_parameter<int>(param_prefix + ".traj_num_extra_states", config->traj_num_extra_states);
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 6) {
    std::string err{"Qc diagonal malformed. Must be 6 elements!"};
    CLOG(ERROR, "radar.odometry_icp") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2], qcd[3], qcd[4], qcd[5];

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
  config->remove_orientation = node->declare_parameter<bool>(param_prefix + ".remove_orientation", false);
  config->normal_score_threshold = node->declare_parameter<double>(param_prefix + ".normal_score_threshold", 0.0);
  const auto w_icp = node->declare_parameter<std::vector<double>>(param_prefix + ".w_icp_diag", std::vector<double>(3, 1.0));
  if (w_icp.size() != 3) {
    std::string err{"W_icp diagonal malformed. Must be 3 elements!"};
    CLOG(ERROR, "radar.odometry_icp") << err;
    throw std::invalid_argument{err};
  }
  config->W_icp << w_icp[0], 0, 0, 0, w_icp[1], 0, 0, 0, w_icp[2];

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

void OdometryDopplerModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    return;
  }

  if (!qdata.sliding_map_odo) {
    // Initialize all variables
    CLOG(INFO, "radar.odometry_icp") << "First frame, simply return.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());

    qdata.timestamp_odo_radar.emplace(*qdata.stamp);
    qdata.T_r_m_odo_radar.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo_radar.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    // Initialize prior values
    qdata.T_r_m_odo_prior.emplace(lgmath::se3::Transformation());
    qdata.w_m_r_in_r_odo_prior.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    qdata.cov_prior.emplace(1e-5 * Eigen::Matrix<double, 12, 12>::Identity());
    // Initialize timestamp equal to the end of the first frame
    const auto &query_points = *qdata.preprocessed_point_cloud;
    const auto compare_time = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
    qdata.timestamp_prior.emplace(std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp);

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

  CLOG(DEBUG, "radar.odometry_icp")
      << "Retrieve input data and setup evaluators.";

  // Inputs
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

  // Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;

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

  // Let's check if our odometry estimate already passed the time stamp of the radar scan
  // If this is the case, we want to estimate the odometry at this time, not at the time of the scan
  // This avoids jumping 'back' in time to the last radar scan, when we already extrapolated the state using gyro
  // Instead the radar scan is then incorporated as a past measurement to correct this extrapolated state
  // If the query stamp is more recent than the last odometry estimate, we proceed as usual
  if(odo_time_general.seconds() > scan_time.seconds())
  {
    CLOG(DEBUG, "radar.odometry_icp") << "Last odometry and gyro preintegration update is more recent than radar scan.";
    timestamp_odo_new = *qdata.timestamp_odo;
  }

  // clang-format off
  /// trajectory smoothing
  Evaluable<lgmath::se3::Transformation>::ConstPtr T_r_m_eval = nullptr;
  Evaluable<Eigen::Matrix<double, 6, 1>>::ConstPtr w_m_r_in_r_eval = nullptr;
  Evaluable<lgmath::se3::Transformation>::ConstPtr T_r_m_eval_extp = nullptr;
  Evaluable<Eigen::Matrix<double, 6, 1>>::ConstPtr w_m_r_in_r_eval_extp = nullptr;
  const_vel::Interface::Ptr trajectory = const_vel::Interface::MakeShared(config_->traj_qc_diag);
  lgmath::se3::Transformation T_r_m_odo_prior_new; 
  Eigen::Matrix<double, 6, 1> w_m_r_in_r_odo_prior_new;
  Eigen::Matrix<double, 12, 12> cov_prior_new;
  std::vector<StateVarBase::Ptr> state_vars;

  // Set up main state variables
  const int64_t num_states = config_->traj_num_extra_states + 2;
  const int64_t time_diff = (frame_end_time - frame_start_time) / (num_states - 1);
  for (int i = 0; i < num_states; ++i) {
    // Load in explicit end_time in case there is small rounding issues
    const int64_t knot_time_stamp = (i == num_states - 1) ? frame_end_time : frame_start_time + i * time_diff;
    Time knot_time(static_cast<int64_t>(knot_time_stamp));
    const Eigen::Matrix<double,6,1> xi_m_r_in_r_odo((knot_time - timestamp_prior).seconds() * w_m_r_in_r_odo_prior);
    const auto T_r_m_odo_extp = tactic::EdgeTransform(xi_m_r_in_r_odo) * T_r_m_odo_prior;
    // Set up state variables
    const std::string T_r_m_var_name = "T_r_m_" + std::to_string(i);
    const auto T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo_extp, T_r_m_var_name);
    const std::string w_m_r_in_r_var_name = "w_m_r_in_r_" + std::to_string(i);
    const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo_prior, w_m_r_in_r_var_name);
    // Add variables
    trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);
    state_vars.emplace_back(T_r_m_var);
    state_vars.emplace_back(w_m_r_in_r_var);
  }

  // Set up priors
  CLOG(DEBUG, "radar.odometry_icp") << "Adding prior to trajectory.";
  trajectory->addStatePrior(Time(frame_start_time), T_r_m_odo_prior, w_m_r_in_r_odo_prior, cov_prior);

  // General radar odometry (at scan time)
  T_r_m_eval = trajectory->getPoseInterpolator(scan_time);
  w_m_r_in_r_eval = trajectory->getVelocityInterpolator(scan_time);

  // Odometry at extrapolated state (might be the same as above, but not necessarily, if we have gyro)
  Time extp_time(static_cast<int64_t>(timestamp_odo_new));
  T_r_m_eval_extp = trajectory->getPoseInterpolator(extp_time);
  w_m_r_in_r_eval_extp = trajectory->getVelocityInterpolator(extp_time);

  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
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

  /// create kd-tree of the map
  CLOG(DEBUG, "radar.odometry_icp") << "Start building a kd-tree of the map.";
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  /// perform initial alignment
  CLOG(DEBUG, "radar.odometry_icp") << "Start initial alignment.";
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
      const Eigen::Vector3f v_m_s_in_s = w_m_s_in_s.block<3, 1>(0, 0);
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
    const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
    aligned_mat.block<4, 1>(0, i) = T_m_s * aligned_mat.block<4, 1>(0, i);
    aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
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
  EdgeTransform T_r_m_icp;
  float matched_points_ratio = 0.0;

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(4, 4);
  bool refinement_stage = false;
  int refinement_step = 0;
  bool solver_failed = false;

  CLOG(DEBUG, "radar.odometry_icp") << "Start the ICP optimization loop.";
  if (qdata.gyro_msgs) CLOG(DEBUG, "radar.odometry_icp") << "Gyro messages are available.";
  if (qdata.doppler_scan) CLOG(DEBUG, "radar.odometry_icp") << "Doppler scan is available.";
  if (config_->remove_orientation) CLOG(DEBUG, "radar.odometry_icp") << "Removing ICP orientation contribution.";
  for (int step = 0;; step++) {
    /// sample points
    timer[0]->start();
    std::vector<std::pair<size_t, size_t>> sample_inds;
    sample_inds.resize(query_points.size());
    // pick queries (for now just use all of them)
    for (size_t i = 0; i < query_points.size(); i++) sample_inds[i].first = i;
    timer[0]->stop();

    /// find nearest neigbors and distances
    timer[1]->start();
    std::vector<float> nn_dists(sample_inds.size());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (size_t i = 0; i < sample_inds.size(); i++) {
      KDTreeResultSet result_set(1);
      result_set.init(&sample_inds[i].second, &nn_dists[i]);
      kdtree->findNeighbors(result_set, aligned_points[sample_inds[i].first].data, search_params);
    }
    timer[1]->stop();

    /// filtering based on distances metrics
    timer[2]->start();
    std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
    filtered_sample_inds.reserve(sample_inds.size());
    for (size_t i = 0; i < sample_inds.size(); i++) {
      if (nn_dists[i] < max_pair_d2) {
      //   // Check planar distance (only after a few steps for initial alignment)
      //   auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
      //               point_map[sample_inds[i].second].getVector3fMap();
      //   float planar_dist = std::abs(
      //       diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap()));
      //   if (step < first_steps || planar_dist < max_planar_d) {
          filtered_sample_inds.push_back(sample_inds[i]);
        // }
      }
    }
    timer[2]->stop();

    /// point to point optimization
    timer[3]->start();

    // initialize problem
    OptimizationProblem problem(config_->num_threads);

    // add variables
    for (const auto &var : state_vars)
      problem.addStateVariable(var);

    // add prior cost terms
    trajectory->addPriorCostTerms(problem);

    // shared loss function
    // auto loss_func = HuberLossFunc::MakeShared(config_->huber_delta);
    auto icp_loss_func = CauchyLossFunc::MakeShared(config_->cauchy_k);
    // cost terms and noise model
    bool rv_cost_added = false;
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &ind : filtered_sample_inds) {
      // noise model W = n * n.T (information matrix)
      Eigen::Matrix3d W = [&] {
        Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
        if (config_->use_p2pl && (point_map[ind.second].normal_score > config_->normal_score_threshold)) {
          // point to plane
          Eigen::Vector3d nrm = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
          W = Eigen::Matrix3d(nrm * nrm.transpose());
          W(2, 2) = 1.0;
        }
        else {
          W = config_->W_icp;
        }
        return W;
      }();
      auto icp_noise_model = StaticNoiseModel<3>::MakeShared(W);

      // query and reference point
      const auto qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
      const bool rm_ori = config_->remove_orientation;

      auto icp_error_func = [&]() -> Evaluable<Eigen::Matrix<double, 3, 1>>::Ptr {
        const auto &qry_time = query_points[ind.first].timestamp;
        const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
        const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
        if (beta != 0) {
          const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(qry_time));
          const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);
          const auto &up_chirp = query_points[ind.first].up_chirp;
          if (up_chirp) {
            return p2p::p2pErrorDoppler(T_m_s_intp_eval, w_m_s_in_s_intp_eval, ref_pt, qry_pt, beta, rm_ori);
          } else {
            return p2p::p2pErrorDoppler(T_m_s_intp_eval, w_m_s_in_s_intp_eval, ref_pt, qry_pt, -beta, rm_ori);
          }
          
        } else {
          return p2p::p2pError(T_m_s_intp_eval, ref_pt, qry_pt, rm_ori);
        }
      }();

      // create cost term and add to problem
      auto icp_cost = WeightedLeastSqCostTerm<3>::MakeShared(icp_error_func, icp_noise_model, icp_loss_func, "icp_cost_" + std::to_string(ind.first));

// Only add cost terms one thread at a time
#pragma omp critical(odo_icp_add_p2p_error_cost)
{
      // problem.addCostTerm(icp_cost);
}
    }

    // Add Doppler velocity measurements if available
    if (qdata.doppler_scan && !config_->use_vel_meas) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (const auto &azimuth_vel : *qdata.doppler_scan) {
        // Load in Doppler measurement and timestamp
        const auto radial_velocity = azimuth_vel.radial_velocity;
        const auto dopp_time = azimuth_vel.timestamp;
        const auto azimuth = azimuth_vel.azimuth;
        const auto azimuth_idx = azimuth_vel.azimuth_idx;
        const Eigen::Matrix<double, 1, 1> radial_velocity_vec = radial_velocity * Eigen::Matrix<double, 1, 1>::Ones();

        // Get velocity in radar frame
        const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(dopp_time));
        const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);

        // Generate empty bias state (for now)
        Eigen::Matrix<double, 6, 1> b_zero = Eigen::Matrix<double, 6, 1>::Zero();
        // b_zero(0) = 0.10;
        // b_zero(1) = -0.11;
        const auto bias = VSpaceStateVar<6>::MakeShared(b_zero);
        bias->locked() = true;

        // Form error terms
        const auto loss_func = CauchyLossFunc::MakeShared(config_->dopp_cauchy_k);
        const auto noise_model = StaticNoiseModel<1>::MakeShared(Eigen::Matrix<double, 1, 1>(config_->dopp_meas_std));
        const auto error_func = imu::DopplerErrorEvaluatorSE2::MakeShared(w_m_s_in_s_intp_eval, bias, azimuth, radial_velocity_vec);
        const auto doppler_cost = WeightedLeastSqCostTerm<1>::MakeShared(error_func, noise_model, loss_func, "doppler_cost_" + std::to_string(azimuth_idx));
        // Add cost term to problem
#pragma omp critical(odo_icp_add_doppler_error_cost)
{
        // problem.addCostTerm(doppler_cost);
}
      }
    }

    // Add individual gyro cost terms if populated
    if (qdata.gyro_msgs) {
      // Load in transform between gyro and robot frame
      const auto &T_s_r_gyro = *qdata.T_s_r_gyro;

      for (const auto &gyro_msg : *qdata.gyro_msgs) {
        // Load in gyro measurement and timestamp
        const auto gyro_meas = Eigen::Vector3d(gyro_msg.angular_velocity.x, gyro_msg.angular_velocity.y, gyro_msg.angular_velocity.z);
        const rclcpp::Time gyro_stamp(gyro_msg.header.stamp);
        const auto gyro_stamp_time = static_cast<int64_t>(gyro_stamp.nanoseconds());

        // Transform gyro measurement into robot frame
        Eigen::VectorXd gyro_meas_g(3); // Create a 6x1 vector
        gyro_meas_g << gyro_meas(0), gyro_meas(1), gyro_meas(2);
        const Eigen::Matrix<double, 3, 1> gyro_meas_r = T_s_r_gyro.matrix().block<3, 3>(0, 0).transpose() * gyro_meas_g;

        // Interpolate velocity measurement at gyro stamp
        auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(gyro_stamp_time));

        // Generate empty bias state
        Eigen::Matrix<double, 6, 1> b_zero = Eigen::Matrix<double, 6, 1>::Zero();
        const auto bias = VSpaceStateVar<6>::MakeShared(b_zero);
        bias->locked() = true;
        const auto loss_func = L2LossFunc::MakeShared();
        const auto noise_model = StaticNoiseModel<1>::MakeShared(Eigen::Matrix<double, 1, 1>(config_->gyro_cov));
        const auto error_func = imu::GyroErrorEvaluatorSE2::MakeShared(w_m_r_in_r_intp_eval, bias, gyro_meas_r);
        const auto gyro_cost = WeightedLeastSqCostTerm<1>::MakeShared(error_func, noise_model, loss_func, "gyro_cost_" + std::to_string(gyro_stamp_time));

        // problem.addCostTerm(gyro_cost);
      }
    }


    // optimize
    GaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.max_iterations = (unsigned int)config_->max_iterations;

    GaussNewtonSolver solver(problem, params);
    try {
      solver.optimize();
    } catch(const std::runtime_error& e) {
      CLOG(WARNING, "radar.odometry_icp") << "STEAM failed to solve, skipping frame. Error message: " << e.what();
      solver_failed = true;
    }

    Covariance covariance(solver);
    timer[3]->stop();

    /// Alignment
    timer[4]->start();
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
        const Eigen::Vector3f v_m_s_in_s = w_m_s_in_s.block<3, 1>(0, 0);
        Eigen::Vector3f abar = aligned_mat.block<3, 1>(0, i);
        abar.normalize();
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
      const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
      aligned_mat.block<4, 1>(0, i) = T_m_s * aligned_mat.block<4, 1>(0, i);
      aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
    }

    // Update all result matrices
    const auto T_m_s = T_m_s_eval->evaluate().matrix();
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

    // Refininement incremental
    if (refinement_stage) refinement_step++;

    // Stop condition
    if (!refinement_stage && step >= first_steps) {
      if ((step >= max_it - 1) || (mean_dT < config_->trans_diff_thresh &&
                                   mean_dR < config_->rot_diff_thresh)) {
        CLOG(DEBUG, "radar.odometry_icp") << "Initial alignment takes " << step << " steps.";

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
         mean_dR < config_->rot_diff_thresh) || 
         solver_failed) {
      // result
      Eigen::Matrix<double, 6, 6> T_r_m_cov = Eigen::Matrix<double, 6, 6>::Identity();
      T_r_m_cov = trajectory->getCovariance(covariance, Time(static_cast<int64_t>(timestamp_odo_new))).block<6, 6>(0, 0);
      T_r_m_icp = EdgeTransform(T_r_m_eval_extp->value(), T_r_m_cov);

      // We are marginalizing out everything but the last two states. Therefore, our prior will just be equal to the final solution/cov
      // on those two states.
      T_r_m_odo_prior_new =  trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->pose()->evaluate();
      w_m_r_in_r_odo_prior_new = trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->velocity()->evaluate();
      cov_prior_new = trajectory->getCovariance(covariance, Time(static_cast<int64_t>(frame_end_time))).block<12, 12>(0, 0);
      cov_prior_new = 0.5 * (cov_prior_new + cov_prior_new.transpose());

      //
      matched_points_ratio = (float)filtered_sample_inds.size() / (float)sample_inds.size();
      //
      CLOG(DEBUG, "radar.odometry_icp") << "Total number of steps: " << step << ", with matched ratio " << matched_points_ratio;
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "radar.odometry_icp") << "ICP did not converge to the specified threshold";
        if (!refinement_stage) {
          CLOG(WARNING, "radar.odometry_icp") << "ICP did not enter refinement stage at all.";
        }
      }
      break;
    }
    timer[6]->stop();
  }

  /// Dump timing info
  CLOG(DEBUG, "radar.odometry_icp") << "Dump timing info inside loop: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "radar.odometry_icp") << "  " << clock_str[i] << timer[i]->count();
  }

  // Outputs
  bool estimate_reasonable = true;
  // Check if change between initial and final velocity is reasonable
  if (true) {
    const auto &w_m_r_in_r_eval_ = trajectory->getVelocityInterpolator(Time(static_cast<int64_t>(scan_stamp)))->evaluate().matrix();
    const auto vel_diff = w_m_r_in_r_eval_ - w_m_r_in_r_odo;
    const auto vel_diff_norm = vel_diff.norm();
    const auto trans_vel_diff_norm = vel_diff.head<3>().norm();
    const auto rot_vel_diff_norm = vel_diff.tail<3>().norm();

    const auto T_r_m_prev = *qdata.T_r_m_odo;
    const auto T_r_m_query = T_r_m_eval->value();
    const auto diff_T = (T_r_m_query.inverse() * T_r_m_prev).vec();
    const auto diff_T_trans = diff_T.head<3>().norm();
    const auto diff_T_rot = diff_T.tail<3>().norm();
    
    CLOG(DEBUG, "radar.odometry_icp") << "Current transformation difference: " << diff_T.transpose();
    CLOG(DEBUG, "radar.odometry_icp") << "Current velocity difference: " << vel_diff.transpose();

    if (trans_vel_diff_norm > config_->max_trans_vel_diff || rot_vel_diff_norm > config_->max_rot_vel_diff) {
      CLOG(WARNING, "radar.odometry_icp") << "Velocity difference between initial and final is too large: " << vel_diff_norm << " translational velocity difference: " << trans_vel_diff_norm << " rotational velocity difference: " << rot_vel_diff_norm;
      estimate_reasonable = false;
    }

    if (diff_T_trans > config_->max_trans_diff) {
      CLOG(WARNING, "radar.odometry_icp") << "Transformation difference between initial and final translation is too large. Transform difference vector: " << diff_T.transpose();
      estimate_reasonable = false;
    }
    if (diff_T_rot > config_->max_rot_diff) {
      CLOG(WARNING, "radar.odometry_icp") << "Transformation difference between initial and final rotation is too large. Transform difference vector: " << diff_T.transpose();
      estimate_reasonable = false;
    }
  }

  if (matched_points_ratio > config_->min_matched_ratio && estimate_reasonable && !solver_failed) {
    // undistort the preprocessed pointcloud to eval state (at query timestamp)
    const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse().cast<float>();
    aligned_mat = T_s_m * aligned_mat;
    aligned_norms_mat = T_s_m * aligned_norms_mat;

    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
    cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
    qdata.undistorted_point_cloud = undistorted_point_cloud;
#if false
    // store undistorted raw point cloud
    auto undistorted_raw_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.raw_point_cloud);
    auto &raw_points = *undistorted_raw_point_cloud;
    auto points_mat = raw_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < raw_points.size(); i++) {
      const auto &qry_time = raw_points[i].timestamp;
      const auto T_rintp_m_eval = trajectory->getPoseInterpolator(Time(qry_time));
      const auto T_s_sintp_eval = inverse(compose(T_s_r_eval, compose(T_rintp_m_eval, T_m_s_eval)));
      const auto T_s_sintp = T_s_sintp_eval->evaluate().matrix().cast<float>();
      points_mat.block<4, 1>(0, i) = T_s_sintp * points_mat.block<4, 1>(0, i);
    }
    cart2pol(*undistorted_raw_point_cloud);
    qdata.undistorted_raw_point_cloud = undistorted_raw_point_cloud;
#endif
    // store trajectory info
    // odometry at radar scan
    *qdata.w_m_r_in_r_odo_radar = w_m_r_in_r_eval->value();

    // odometry at extrapolated time
    *qdata.w_m_r_in_r_odo = w_m_r_in_r_eval_extp->value();
    // odometry at radar scan
    *qdata.T_r_m_odo_radar = T_r_m_eval->value();
    *qdata.timestamp_odo_radar = scan_stamp;

    // odometry at extr. time
    *qdata.T_r_m_odo = T_r_m_eval_extp->value();
    *qdata.timestamp_odo = timestamp_odo_new;

#if 1
   CLOG(DEBUG, "radar.odometry_icp") << "T_m_r is: " << qdata.T_r_m_odo->inverse().vec().transpose();
   CLOG(DEBUG, "radar.odometry_icp") << "w_m_r_in_r is: " << qdata.w_m_r_in_r_odo->transpose();
#endif
    //
    /// \todo double check validity when no vertex has been created
    *qdata.T_r_v_odo = T_r_m_icp * sliding_map_odo.T_vertex_this().inverse();
    /// \todo double check that we can indeed treat m same as v for velocity
    *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
    *qdata.T_r_m_odo_prior = T_r_m_odo_prior_new;
    *qdata.w_m_r_in_r_odo_prior = w_m_r_in_r_odo_prior_new;
    *qdata.cov_prior = cov_prior_new;
    *qdata.timestamp_prior = frame_end_time;

    //
    *qdata.odo_success = true;
    CLOG(DEBUG, "radar.odometry_icp") << "Odometry successful. T_r_m_icp: " << T_r_m_icp;
    CLOG(DEBUG, "radar.odometry_icp") << "T_r_v_odo: " << *qdata.T_r_v_odo;
  } else {
    if (matched_points_ratio <= config_->min_matched_ratio) {
      CLOG(WARNING, "radar.odometry_icp")
          << "Matched points ratio " << matched_points_ratio
          << " is below the threshold. ICP is considered failed.";
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
    // no update to map to robot transform
    *qdata.odo_success = false;
  }
  // clang-format on
}

}  // namespace radar
}  // namespace vtr