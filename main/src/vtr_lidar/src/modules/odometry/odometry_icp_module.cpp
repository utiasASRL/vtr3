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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/odometry/odometry_icp_module.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

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

auto OdometryICPModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // motion compensation
  config->use_trajectory_estimation = node->declare_parameter<bool>(param_prefix + ".use_trajectory_estimation", config->use_trajectory_estimation);
  config->traj_num_extra_states = node->declare_parameter<int>(param_prefix + ".traj_num_extra_states", config->traj_num_extra_states);
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 6) {
    std::string err{"Qc diagonal malformed. Must be 6 elements!"};
    CLOG(ERROR, "lidar.odometry_icp") << err;
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
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", false);
  config->max_iterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", 1);

  config->gyro_cov = node->declare_parameter<double>(param_prefix + ".gyro_cov", config->gyro_cov);
  config->remove_orientation = node->declare_parameter<bool>(param_prefix + ".remove_orientation", false);

  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);
  config->max_trans_vel_diff = node->declare_parameter<float>(param_prefix + ".max_trans_vel_diff", config->max_trans_vel_diff);
  config->max_rot_vel_diff = node->declare_parameter<float>(param_prefix + ".max_rot_vel_diff", config->max_rot_vel_diff);
  config->max_trans_diff = node->declare_parameter<float>(param_prefix + ".max_trans_diff", config->max_trans_diff);
  config->max_rot_diff = node->declare_parameter<float>(param_prefix + ".max_rot_diff", config->max_rot_diff);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OdometryICPModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "lidar.odometry_icp") << "First frame, simply return.";
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
    //
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
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
    return;
  }

  CLOG(DEBUG, "lidar.odometry_icp")
      << "Retrieve input data and setup evaluators.";

  // Inputs
  auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  auto &sliding_map_odo = *qdata.sliding_map_odo;
  auto &point_map = sliding_map_odo.point_cloud();

  // Load in prior parameters
  const auto &T_r_m_odo_prior = *qdata.T_r_m_odo_prior;
  const auto &w_m_r_in_r_odo_prior = *qdata.w_m_r_in_r_odo_prior;
  const auto &cov_prior = *qdata.cov_prior;
  const auto &timestamp_prior = *qdata.timestamp_prior;

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


  /// trajectory smoothing
  Evaluable<lgmath::se3::Transformation>::ConstPtr T_r_m_eval = nullptr;
  lgmath::se3::Transformation T_r_m_eval_initial = lgmath::se3::Transformation();
  Evaluable<Eigen::Matrix<double, 6, 1>>::ConstPtr w_m_r_in_r_eval = nullptr;
  const_vel::Interface::Ptr trajectory = nullptr;
  std::vector<StateVarBase::Ptr> state_vars;
  const int64_t num_states = config_->traj_num_extra_states + 2;
  // Set up variables for new prior generation
  lgmath::se3::Transformation T_r_m_odo_prior_new; 
  Eigen::Matrix<double, 6, 1> w_m_r_in_r_odo_prior_new;
  Eigen::Matrix<double, 12, 12> cov_prior_new;

  if(query_stamp < timestamp_prior) { 
    CLOG(WARNING, "lidar.odometry_icp") << "Difference between the two stamps is " << (query_stamp - timestamp_prior) << " ns";
    query_stamp = timestamp_prior;
  } 

  // Set up shared timestamps
  int64_t frame_end_time;
  Time query_time = Time(static_cast<int64_t>(query_stamp));


  Time prev_time(static_cast<int64_t>(timestamp_prior));
  trajectory = const_vel::Interface::MakeShared(config_->traj_qc_diag);

  // Set up problem timestamps
  const auto compare_time = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
  const auto frame_start_time = timestamp_prior;
  frame_end_time = std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp;
  const int64_t time_diff = (frame_end_time - frame_start_time) / (num_states - 1);

  // Set up main state variables
  for (int i = 0; i < num_states; ++i) {
    // Load in explicit end_time in case there is small rounding issues
    const int64_t knot_time_stamp = (i == num_states - 1) ? frame_end_time : frame_start_time + i * time_diff;
    Time knot_time(static_cast<int64_t>(knot_time_stamp));
    const Eigen::Matrix<double,6,1> xi_m_r_in_r_odo((knot_time - prev_time).seconds() * w_m_r_in_r_odo_prior);
    const auto T_r_m_odo_extp = tactic::EdgeTransform(xi_m_r_in_r_odo) * T_r_m_odo_prior;
    const auto T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo_extp);
    const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo_prior);
    trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);
    state_vars.emplace_back(T_r_m_var);
    state_vars.emplace_back(w_m_r_in_r_var);
  }

  // Set up priors
  CLOG(DEBUG, "lidar.odometry_icp") << "Adding prior to trajectory.";
  trajectory->addStatePrior(Time(frame_start_time), T_r_m_odo_prior, w_m_r_in_r_odo_prior, cov_prior);

  // Set up eval state at which results will be generated and at which pointcloud will get undistorted to
  T_r_m_eval = trajectory->getPoseInterpolator(query_time);
  w_m_r_in_r_eval = trajectory->getVelocityInterpolator(query_time);
  
  T_r_m_eval_initial = T_r_m_eval->value();

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
  CLOG(DEBUG, "lidar.odometry_icp") << "Start building a kd-tree of the map.";
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  /// perform initial alignment
  CLOG(DEBUG, "lidar.odometry_icp") << "Start initial alignment.";
  #pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < query_points.size(); i++) {
    const auto &qry_time = (config_->use_trajectory_estimation) ? query_points[i].timestamp : frame_end_time;
    const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
    const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
    const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
    aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
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
  
  CLOG(DEBUG, "lidar.odometry_icp") << "Start the ICP optimization loop.";
  CLOG_IF(qdata.gyro_msgs, DEBUG, "lidar.odometry_icp") << "Gyro messages are available.";
  CLOG_IF(config_->remove_orientation, DEBUG, "lidar.odometry_icp") << "Removing ICP orientation contribution.";

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
        // Check planar distance (only after a few steps for initial alignment)
        auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
                    point_map[sample_inds[i].second].getVector3fMap();
        float planar_dist = std::abs(
            diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap()));
        if (step < first_steps || planar_dist < max_planar_d) {
          filtered_sample_inds.push_back(sample_inds[i]);
        }
      }
    }
    timer[2]->stop();

    /// point to plane optimization
    timer[3]->start();

    // initialize problem
    SlidingWindowFilter problem(config_->num_threads);

    // add variables
    for (const auto &var : state_vars)
      problem.addStateVariable(var);

    // add prior cost terms
    trajectory->addPriorCostTerms(problem);

    // shared loss function
    auto loss_func = L2LossFunc::MakeShared();
    // cost terms and noise model
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &ind : filtered_sample_inds) {
      // noise model W = n * n.T (information matrix)
      if (point_map[ind.second].normal_score <= 0.0) continue;
      Eigen::Vector3d nrm = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      Eigen::Matrix3d W(point_map[ind.second].normal_score * (nrm * nrm.transpose()) + 1e-5 * Eigen::Matrix3d::Identity());
      auto noise_model = StaticNoiseModel<3>::MakeShared(W, NoiseType::INFORMATION);

      // query and reference point
      const auto qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();
      const bool rm_ori = config_->remove_orientation;

      auto error_func = [&]() -> Evaluable<Eigen::Matrix<double, 3, 1>>::Ptr {
        const auto &qry_time = (config_->use_trajectory_estimation) ? query_points[ind.first].timestamp : frame_end_time;
        const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
        const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
        return p2p::p2pError(T_m_s_intp_eval, ref_pt, qry_pt, rm_ori);
      }();

      // create cost term and add to problem
      auto cost = WeightedLeastSqCostTerm<3>::MakeShared(error_func, noise_model, loss_func);

#pragma omp critical(odo_icp_add_p2p_error_cost)
      problem.addCostTerm(cost);
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
        Eigen::VectorXd gyro_meas_g(3);
        gyro_meas_g << gyro_meas(0), gyro_meas(1), gyro_meas(2);
        const Eigen::Matrix<double, 3, 1> gyro_meas_r = T_s_r_gyro.matrix().block<3, 3>(0, 0).transpose() * gyro_meas_g;

        // Interpolate velocity measurement at gyro stamp
        auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(gyro_stamp_time));

        // Generate empty bias state
        Eigen::Matrix<double, 6, 1> b_zero = Eigen::Matrix<double, 6, 1>::Zero();
        const auto bias = VSpaceStateVar<6>::MakeShared(b_zero);
        bias->locked() = true;
        const auto loss_func = L2LossFunc::MakeShared();
        const auto noise_model = StaticNoiseModel<3>::MakeShared(config_->gyro_cov * Eigen::Matrix<double, 3, 3>::Identity());
        const auto error_func = imu::GyroErrorEvaluator::MakeShared(w_m_r_in_r_intp_eval, bias, gyro_meas_r);
        const auto gyro_cost = WeightedLeastSqCostTerm<3>::MakeShared(error_func, noise_model, loss_func);

        problem.addCostTerm(gyro_cost);
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
      CLOG(WARNING, "lidar.odometry_icp") << "STEAM failed to solve, skipping frame. Error message: " << e.what();
      solver_failed = true;
    }

    Covariance covariance(solver);
    timer[3]->stop();

    /// Alignment
    timer[4]->start();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < query_points.size(); i++) {
      const auto &qry_time = (config_->use_trajectory_estimation) ? query_points[i].timestamp : frame_end_time;
      const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
      const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
      const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
      aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
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
        CLOG(DEBUG, "lidar.odometry_icp") << "Initial alignment takes " << step << " steps.";

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
      T_r_m_cov = trajectory->getCovariance(covariance, Time(static_cast<int64_t>(query_stamp))).block<6, 6>(0, 0);
      T_r_m_icp = EdgeTransform(T_r_m_eval->value(), T_r_m_cov);

      // Marginalize out all but last 2 states for prior
      std::vector<StateVarBase::Ptr> state_vars_marg;
      for (int i = 0; i < num_states*2 - 2; ++i) {
        state_vars_marg.push_back(state_vars[i]);
      }
      problem.marginalizeVariable(state_vars_marg);
      params.max_iterations = 1; // Only one iteration for marginalization
      GaussNewtonSolver solver_marg(problem, params);
      solver_marg.optimize();
      Covariance covariance_marg(solver_marg);
      T_r_m_odo_prior_new =  trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->pose()->evaluate();
      w_m_r_in_r_odo_prior_new = trajectory->get(Time(static_cast<int64_t>(frame_end_time)))->velocity()->evaluate();
      cov_prior_new = trajectory->getCovariance(covariance_marg, Time(static_cast<int64_t>(frame_end_time))).block<12, 12>(0, 0);
      cov_prior_new = 0.5 * (cov_prior_new + cov_prior_new.transpose());
      
      matched_points_ratio = (float)filtered_sample_inds.size() / (float)sample_inds.size();
      //
      CLOG(DEBUG, "lidar.odometry_icp") << "Total number of steps: " << step << ", with matched ratio " << matched_points_ratio;
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "lidar.odometry_icp") << "ICP did not converge to the specified threshold";
        if (!refinement_stage) {
          CLOG(WARNING, "lidar.odometry_icp") << "ICP did not enter refinement stage at all.";
        }
      }
      break;
    }
    timer[6]->stop();
  }

  /// Dump timing info
  CLOG(DEBUG, "lidar.odometry_icp") << "Dump timing info inside loop: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "lidar.odometry_icp") << "  " << clock_str[i] << timer[i]->count();
  }

  /// Outputs
  bool estimate_reasonable = true;
  // Check if change between initial and final velocity is reasonable
  const auto &w_m_r_in_r_prev = *qdata.w_m_r_in_r_odo_prior;
  const auto &w_m_r_in_r_new = trajectory->getVelocityInterpolator(Time(static_cast<int64_t>(query_stamp)))->evaluate().matrix();
  const auto vel_diff = w_m_r_in_r_new - w_m_r_in_r_prev;
  const auto vel_diff_norm = vel_diff.norm();
  const auto trans_vel_diff_norm = vel_diff.head<3>().norm();
  const auto rot_vel_diff_norm = vel_diff.tail<3>().norm();

  const auto T_r_m_query = T_r_m_eval->value();
  const auto diff_T = (T_r_m_query.inverse() * T_r_m_eval_initial).vec();
  const auto diff_T_trans = diff_T.head<3>().norm();
  const auto diff_T_rot = diff_T.tail<3>().norm();
  
  CLOG(DEBUG, "lidar.odometry_icp") << "Current transformation difference: " << diff_T.transpose();
  CLOG(DEBUG, "lidar.odometry_icp") << "Current velocity difference: " << vel_diff.transpose();

  if (trans_vel_diff_norm > config_->max_trans_vel_diff || rot_vel_diff_norm > config_->max_rot_vel_diff) {
    CLOG(WARNING, "lidar.odometry_icp") << "Velocity difference between initial and final is too large: " << vel_diff_norm << ". Translational velocity difference: " << trans_vel_diff_norm << ". Rotational velocity difference: " << rot_vel_diff_norm;
    estimate_reasonable = false;
  }

  if (diff_T_trans > config_->max_trans_diff) {
    CLOG(WARNING, "lidar.odometry_icp") << "Transformation difference between initial and final translation is too large. Transform difference vector: " << diff_T.transpose();
    estimate_reasonable = false;
  }
  if (diff_T_rot > config_->max_rot_diff) {
    CLOG(WARNING, "lidar.odometry_icp") << "Transformation difference between initial and final rotation is too large. Transform difference vector: " << diff_T.transpose();
    estimate_reasonable = false;
  }
  

  if (matched_points_ratio > config_->min_matched_ratio && estimate_reasonable && !solver_failed) {
    // undistort the preprocessed pointcloud
    const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse().cast<float>();
    aligned_mat = T_s_m * aligned_mat;
    aligned_norms_mat = T_s_m * aligned_norms_mat;

    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
    cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
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
        const auto T_s_sintp_eval = inverse(compose(T_s_r_eval, compose(T_rintp_m_eval, T_m_s_eval)));
        const auto T_s_sintp = T_s_sintp_eval->evaluate().matrix().cast<float>();
        points_mat.block<4, 1>(0, i) = T_s_sintp * points_mat.block<4, 1>(0, i);
      }
    }
    cart2pol(*undistorted_raw_point_cloud);
    qdata.undistorted_raw_point_cloud = undistorted_raw_point_cloud;
#endif
    // store trajectory info
    *qdata.w_m_r_in_r_odo = w_m_r_in_r_eval->value();
    *qdata.T_r_m_odo_prior = T_r_m_odo_prior_new;
    *qdata.w_m_r_in_r_odo_prior = w_m_r_in_r_odo_prior_new;
    *qdata.cov_prior = cov_prior_new;
    *qdata.timestamp_prior = frame_end_time;
    
    *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
    *qdata.T_r_v_odo = T_r_m_icp * sliding_map_odo.T_vertex_this().inverse();
    *qdata.T_r_m_odo = T_r_m_eval->value();
    *qdata.timestamp_odo = query_stamp;
    
    *qdata.odo_success = true;
  } else {
    if (matched_points_ratio <= config_->min_matched_ratio) {
      CLOG(WARNING, "lidar.odometry_icp")
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

}  // namespace lidar
}  // namespace vtr