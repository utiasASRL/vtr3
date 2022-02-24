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
 * \file odometry_icp_module_v3.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/odometry/odometry_icp_module_v3.hpp"

namespace vtr {
namespace lidar {

namespace {
bool checkDiagonal(Eigen::Array<double, 1, 6> &diag) {
  for (int idx = 0; idx < 6; ++idx)
    if (diag(idx) <= 0) return false;

  return true;
}
}  // namespace

using namespace tactic;
using namespace steam;
using namespace steam::se3;

auto OdometryICPModuleV3::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);
  // trajectory smoothing
  config->trajectory_smoothing = node->declare_parameter<bool>(param_prefix + ".trajectory_smoothing", config->trajectory_smoothing);

  config->lock_prev_velocity = node->declare_parameter<bool>(param_prefix + ".lock_prev_velocity", config->lock_prev_velocity);
  config->use_prev_velocity_as_prior = node->declare_parameter<bool>(param_prefix + ".use_prev_velocity_as_prior", config->use_prev_velocity_as_prior);
  const auto pvc = node->declare_parameter<std::vector<double>>(param_prefix + ".prev_velocity_cov", std::vector<double>());
  if (pvc.size() != 6) {
    std::string err{"Previous velocity covariance malformed. Must be 6 elements!"};
    CLOG(ERROR, "tactic") << err;
    throw std::invalid_argument{err};
  }
  config->prev_velocity_cov.diagonal() << pvc[0], pvc[1], pvc[2], pvc[3], pvc[4], pvc[5];

  config->use_constant_acc = node->declare_parameter<bool>(param_prefix + ".use_constant_acc", config->use_constant_acc);
  config->lin_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_x", config->lin_acc_std_dev_x);
  config->lin_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_y", config->lin_acc_std_dev_y);
  config->lin_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_z", config->lin_acc_std_dev_z);
  config->ang_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_x", config->ang_acc_std_dev_x);
  config->ang_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_y", config->ang_acc_std_dev_y);
  config->ang_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_z", config->ang_acc_std_dev_z);

  // icp params
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
#ifdef VTR_DETERMINISTIC
  CLOG_IF(config->num_threads != 1, WARNING, "lidar.odometry_icp") << "ICP number of threads set to 1 in deterministic mode.";
  config->num_threads = 1;
#endif
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

  // steam params
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", false);
  config->maxIterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", 1);
  config->absoluteCostThreshold = node->declare_parameter<double>(param_prefix + ".abs_cost_thresh", 0.0);
  config->absoluteCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".abs_cost_change_thresh", 0.0001);
  config->relativeCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".rel_cost_change_thresh", 0.0001);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on

  // Make Qc_inv
  Eigen::Array<double, 1, 6> Qc_diag;
  Qc_diag << config->lin_acc_std_dev_x, config->lin_acc_std_dev_y,
      config->lin_acc_std_dev_z, config->ang_acc_std_dev_x,
      config->ang_acc_std_dev_y, config->ang_acc_std_dev_z;
  if (checkDiagonal(Qc_diag) == false && config->trajectory_smoothing) {
    throw std::runtime_error(
        "Elements of the smoothing factor must be greater than zero!");
  }
  config->smoothing_factor_information.setZero();
  config->smoothing_factor_information.diagonal() = 1.0 / Qc_diag;

  return config;
}

void OdometryICPModuleV3::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    pub_ = qdata.node->create_publisher<PointCloudMsg>("udist_scan", 5);
    raw_pub_ = qdata.node->create_publisher<PointCloudMsg>("udist_raw_scan", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  if (!qdata.point_map_odo) {
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
    //
    *qdata.odo_success = true;
    // clang-format on
    return;
  }

  CLOG(DEBUG, "lidar.odometry_icp")
      << "Retrieve input data and setup evaluators.";

  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &timestamp_odo = *qdata.timestamp_odo;
  const auto &T_r_m_odo = *qdata.T_r_m_odo;
  const auto &w_m_r_in_r_odo = *qdata.w_m_r_in_r_odo;
  auto &point_map_odo = *qdata.point_map_odo;
  auto &point_map = point_map_odo.point_map();

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;
  // clang-format off
  /// Create and add the T_robot_map variable, here m = vertex frame.
  auto T_r_m_odo_extp = T_r_m_odo;
  if (config_->trajectory_smoothing) {
    Eigen::Matrix<double,6,1> xi_m_r_in_r_odo(Time(query_stamp - timestamp_odo).seconds() * w_m_r_in_r_odo);
    T_r_m_odo_extp = tactic::EdgeTransform(xi_m_r_in_r_odo) * T_r_m_odo;
  }
  const auto T_r_m_var = std::make_shared<TransformStateVar>(T_r_m_odo_extp);

  /// Create evaluators for passing into ICP
  const auto T_s_r_eval = FixedTransformEvaluator::MakeShared(T_s_r);
  const auto T_r_m_eval = TransformStateEvaluator::MakeShared(T_r_m_var);
  // compound transform for alignment (sensor to point map transform)
  const auto T_m_s_eval = inverse(compose(T_s_r_eval, T_r_m_eval));

  CLOG(DEBUG, "lidar.odometry_icp")
            << "Trajectory smoothing initialization.";

  /// trajectory smoothing
  std::shared_ptr<SteamTrajInterface> trajectory = nullptr;
  std::vector<StateVariableBase::Ptr> trajectory_state_vars;
  std::shared_ptr<VectorSpaceStateVar> w_m_r_in_r_var = nullptr;
  auto trajectory_cost_terms = std::make_shared<ParallelizedCostTermCollection>();
  if (config_->trajectory_smoothing) {
    trajectory = std::make_shared<SteamTrajInterface>(config_->smoothing_factor_information, true);
    // last frame state
    Time prev_time(static_cast<int64_t>(timestamp_odo));
    auto prev_T_r_m_var = std::make_shared<TransformStateVar>(T_r_m_odo);
    prev_T_r_m_var->setLock(true);
    auto prev_T_r_m_eval = std::make_shared<TransformStateEvaluator>(prev_T_r_m_var);
    auto prev_w_m_r_in_r_var = std::make_shared<VectorSpaceStateVar>(w_m_r_in_r_odo);
    trajectory->add(prev_time, prev_T_r_m_eval, prev_w_m_r_in_r_var);
    if (config_->lock_prev_velocity) {
      prev_w_m_r_in_r_var->setLock(true);
    } else if (config_->use_prev_velocity_as_prior) {
      trajectory->addVelocityPrior(prev_time, w_m_r_in_r_odo, config_->prev_velocity_cov);
    }
    // curr frame state (+ velocity)
    Time query_time(static_cast<int64_t>(query_stamp));
    w_m_r_in_r_var = std::make_shared<VectorSpaceStateVar>(w_m_r_in_r_odo);
    trajectory->add(query_time, T_r_m_eval, w_m_r_in_r_var);
    // add state variables to the collection
    trajectory_state_vars.emplace_back(prev_T_r_m_var);
    trajectory_state_vars.emplace_back(prev_w_m_r_in_r_var);
    trajectory_state_vars.emplace_back(w_m_r_in_r_var);
    // add prior cost terms
    trajectory->appendPriorCostTerms(trajectory_cost_terms);
  }

  CLOG(DEBUG, "lidar.odometry_icp") << "initial alignment.";

  // Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto map_mat = point_map.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::normal_offset());

  /// Perform initial alignment
  {
    const auto T_m_s = T_m_s_eval->evaluate().matrix();
    Eigen::Matrix3f C_m_s = T_m_s.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f r_s_m_in_m = T_m_s.block<3, 1>(0, 3).cast<float>();
    aligned_mat = (C_m_s * query_mat).colwise() + r_s_m_in_m;
    aligned_norms_mat = C_m_s * query_norms_mat;
  }

  // ICP results
  EdgeTransform T_r_m_icp;
  float matched_points_ratio = 0.0;

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(4, 4);
  bool refinement_stage = false;
  int refinement_step = 0;

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

  CLOG(DEBUG, "lidar.odometry_icp") << "Start building a kd-tree of the map.";

  /// create kd-tree of the map
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree = std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  CLOG(DEBUG, "lidar.odometry_icp") << "Start the ICP optimization loop.";

  for (int step = 0;; step++) {
    /// Points Association
    // pick queries (for now just use all of them)
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

    /// Filtering based on distances metrics
    timer[2]->start();
    std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
    filtered_sample_inds.reserve(sample_inds.size());
    for (size_t i = 0; i < sample_inds.size(); i++) {
      if (nn_dists[i] < max_pair_d2) {
        // Check planar distance (only after a few steps for initial alignment)
        auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
                    point_map[sample_inds[i].second].getVector3fMap();
        float planar_dist = abs(
            diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap()));
        if (step < first_steps || planar_dist < max_planar_d) {
          filtered_sample_inds.push_back(sample_inds[i]);
        }
      }
    }
    timer[2]->stop();
    CLOG(DEBUG, "radar.odometry_icp") << "Number of matches: " << filtered_sample_inds.size();

    /// Point to plane optimization
    timer[3]->start();
    // shared loss function
    auto loss_func = std::make_shared<L2LossFunc>();
    // cost terms and noise model
    auto cost_terms = std::make_shared<ParallelizedCostTermCollection>();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &ind : filtered_sample_inds) {
      // noise model W = n * n.T (information matrix)
      Eigen::Vector3d nrm = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      Eigen::Matrix3d W(point_map[ind.second].normal_score * (nrm * nrm.transpose()) + 1e-5 * Eigen::Matrix3d::Identity());
      auto noise_model = std::make_shared<StaticNoiseModel<3>>(W, INFORMATION);

      // query and reference point
      const auto &qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto &ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();

      PointToPointErrorEval2::Ptr error_func;
      if (config_->trajectory_smoothing) {
        const auto &qry_time = query_points[ind.first].time;
        const auto T_r_m_intp_eval = trajectory->getInterpPoseEval(Time(qry_time));
        const auto T_m_s_intp_eval = inverse(compose(T_s_r_eval, T_r_m_intp_eval));
        error_func.reset(new PointToPointErrorEval2(T_m_s_intp_eval, ref_pt, qry_pt));
      } else {
        error_func.reset(new PointToPointErrorEval2(T_m_s_eval, ref_pt, qry_pt));
      }

      // create cost term and add to problem
      auto cost = std::make_shared<WeightedLeastSqCostTerm<3, 6>>(error_func, noise_model, loss_func);

#pragma omp critical(lgicp_add_cost_term)
      cost_terms->add(cost);
    }

    // initialize problem
    OptimizationProblem problem;
    problem.addStateVariable(T_r_m_var);
    problem.addCostTerm(cost_terms);

    // add prior costs
    if (config_->trajectory_smoothing) {
      for (const auto &var : trajectory_state_vars)
        problem.addStateVariable(var);
      problem.addCostTerm(trajectory_cost_terms);
    }

    // make solver
    using SolverType = VanillaGaussNewtonSolver;
    SolverType::Params params;
    params.verbose = config_->verbose;
    params.maxIterations = config_->maxIterations;
    SolverType solver(&problem, params);

    // Optimize
    try {
      solver.optimize();
    } catch (const decomp_failure &) {
      CLOG(WARNING, "lidar.odometry_icp")
          << "Steam optimization failed! T_m_s left unchanged.";
    }
    timer[3]->stop();

    /// Alignment
    timer[4]->start();
    if (config_->trajectory_smoothing) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (unsigned i = 0; i < query_points.size(); i++) {
        const auto &qry_time = query_points[i].time;
        const auto T_r_m_intp_eval = trajectory->getInterpPoseEval(Time(qry_time));
        const auto T_m_s_intp_eval = inverse(compose(T_s_r_eval, T_r_m_intp_eval));
        const auto T_m_s = T_m_s_intp_eval->evaluate().matrix();
        Eigen::Matrix3f C_m_s = T_m_s.block<3, 3>(0, 0).cast<float>();
        Eigen::Vector3f r_s_m_in_m = T_m_s.block<3, 1>(0, 3).cast<float>();
        aligned_mat.block<3, 1>(0, i) = C_m_s * query_mat.block<3, 1>(0, i) + r_s_m_in_m;
        aligned_norms_mat.block<3, 1>(0, i) = C_m_s * query_norms_mat.block<3, 1>(0, i);
      }
    } else {
      const auto T_m_s = T_m_s_eval->evaluate().matrix();
      Eigen::Matrix3f C_m_s = T_m_s.block<3, 3>(0, 0).cast<float>();
      Eigen::Vector3f r_s_m_in_m = T_m_s.block<3, 1>(0, 3).cast<float>();
      aligned_mat = (C_m_s * query_mat).colwise() + r_s_m_in_m;
      aligned_norms_mat = C_m_s * query_norms_mat;
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
         mean_dR < config_->rot_diff_thresh)) {
      // result
      T_r_m_icp = EdgeTransform(T_r_m_var->getValue(), solver.queryCovariance(T_r_m_var->getKey()));
      matched_points_ratio = (float)filtered_sample_inds.size() / (float)sample_inds.size();
      CLOG(DEBUG, "radar.odometry_icp") << "Total number of steps: " << step << ", with matched ratio " << matched_points_ratio;
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
  if (matched_points_ratio > config_->min_matched_ratio) {
    // undistort the preprocessed pointcloud
    const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse();
    Eigen::Matrix3f C_s_m = T_s_m.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f r_m_s_in_s = T_s_m.block<3, 1>(0, 3).cast<float>();
    aligned_mat = (C_s_m * aligned_mat).colwise() + r_m_s_in_s;
    aligned_norms_mat = C_s_m * aligned_norms_mat;

    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
    cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
    qdata.undistorted_point_cloud = undistorted_point_cloud;
#if false
    // store potentially undistorted raw point cloud
    auto undistorted_raw_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.raw_point_cloud);
    if (config_->trajectory_smoothing) {
      auto &raw_points = *undistorted_raw_point_cloud;
      auto points_mat = raw_points.getMatrixXfMap(3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (unsigned i = 0; i < raw_points.size(); i++) {
        const auto &qry_time = raw_points[i].time;
        const auto T_rintp_m_eval = trajectory->getInterpPoseEval(Time(qry_time));
        const auto T_s_sintp_eval = inverse(compose(T_s_r_eval, compose(T_rintp_m_eval, T_m_s_eval)));
        const auto T_s_sintp = T_s_sintp_eval->evaluate().matrix();
        Eigen::Matrix3f C_s_sintp = T_s_sintp.block<3, 3>(0, 0).cast<float>();
        Eigen::Vector3f r_sintp_s_in_s = T_s_sintp.block<3, 1>(0, 3).cast<float>();
        points_mat.block<3, 1>(0, i) = C_s_sintp * points_mat.block<3, 1>(0, i) + r_sintp_s_in_s;
      }
    }
    cart2pol(*undistorted_raw_point_cloud);
    qdata.undistorted_raw_point_cloud = undistorted_raw_point_cloud;
#endif
    // store trajectory info
    *qdata.timestamp_odo = query_stamp;
    *qdata.T_r_m_odo = T_r_m_var->getValue();
    if (config_->trajectory_smoothing)
      *qdata.w_m_r_in_r_odo = w_m_r_in_r_var->getValue();
    //
    /// \todo double check validity when no vertex has been created
    *qdata.T_r_v_odo = T_r_m_icp * point_map_odo.T_vertex_map().inverse();
    /// \todo double check that we can indeed treat m same as v for velocity
    if (config_->trajectory_smoothing)
      *qdata.w_v_r_in_r_odo = w_m_r_in_r_var->getValue();
    //
    *qdata.odo_success = true;
  } else {
    CLOG(WARNING, "lidar.odometry_icp")
        << "Matched points ratio " << matched_points_ratio
        << " is below the threshold. ICP is considered failed.";
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

  if (config_->visualize) {
#if false  /// publish raw point cloud
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(*qdata.undistorted_raw_point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "lidar";
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      raw_pub_->publish(pc2_msg);
    }
#endif
    {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(*qdata.undistorted_point_cloud, pc2_msg);
      pc2_msg.header.frame_id = "lidar";
      pc2_msg.header.stamp = rclcpp::Time(*qdata.stamp);
      pub_->publish(pc2_msg);
    }
  }
  // clang-format on
}

}  // namespace lidar
}  // namespace vtr