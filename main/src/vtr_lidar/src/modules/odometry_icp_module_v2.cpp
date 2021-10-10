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
 * \file odometry_icp_module_v2.cpp
 * \brief OdometryICPModuleV2 class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/odometry_icp_module_v2.hpp>

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

void OdometryICPModuleV2::configFromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config_->min_matched_ratio);
  // trajectory smoothing
  config_->trajectory_smoothing = node->declare_parameter<bool>(param_prefix + ".trajectory_smoothing", config_->trajectory_smoothing);
  config_->use_constant_acc = node->declare_parameter<bool>(param_prefix + ".use_constant_acc", config_->use_constant_acc);
  config_->lin_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_x", config_->lin_acc_std_dev_x);
  config_->lin_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_y", config_->lin_acc_std_dev_y);
  config_->lin_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_z", config_->lin_acc_std_dev_z);
  config_->ang_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_x", config_->ang_acc_std_dev_x);
  config_->ang_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_y", config_->ang_acc_std_dev_y);
  config_->ang_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_z", config_->ang_acc_std_dev_z);

  // icp params
  config_->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config_->num_threads);
#ifdef VTR_DETERMINISTIC
  LOG_IF(config_->num_threads != 1, WARNING) << "ICP number of threads set to 1 in deterministic mode.";
  config_->num_threads = 1;
#endif
  config_->first_num_steps = node->declare_parameter<int>(param_prefix + ".first_num_steps", config_->first_num_steps);
  config_->initial_max_iter = node->declare_parameter<int>(param_prefix + ".initial_max_iter", config_->initial_max_iter);
  config_->initial_num_samples = node->declare_parameter<int>(param_prefix + ".initial_num_samples", config_->initial_num_samples);
  config_->initial_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".initial_max_pairing_dist", config_->initial_max_pairing_dist);
  config_->initial_max_planar_dist = node->declare_parameter<float>(param_prefix + ".initial_max_planar_dist", config_->initial_max_planar_dist);
  config_->refined_max_iter = node->declare_parameter<int>(param_prefix + ".refined_max_iter", config_->refined_max_iter);
  config_->refined_num_samples = node->declare_parameter<int>(param_prefix + ".refined_num_samples", config_->refined_num_samples);
  config_->refined_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".refined_max_pairing_dist", config_->refined_max_pairing_dist);
  config_->refined_max_planar_dist = node->declare_parameter<float>(param_prefix + ".refined_max_planar_dist", config_->refined_max_planar_dist);

  config_->averaging_num_steps = node->declare_parameter<int>(param_prefix + ".averaging_num_steps", config_->averaging_num_steps);
  config_->rot_diff_thresh = node->declare_parameter<float>(param_prefix + ".rot_diff_thresh", config_->rot_diff_thresh);
  config_->trans_diff_thresh = node->declare_parameter<float>(param_prefix + ".trans_diff_thresh", config_->trans_diff_thresh);

  // steam params
  config_->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", config_->verbose);
  config_->maxIterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", config_->maxIterations);
  config_->absoluteCostThreshold = node->declare_parameter<double>(param_prefix + ".abs_cost_thresh", config_->absoluteCostThreshold);
  config_->absoluteCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".abs_cost_change_thresh", config_->absoluteCostChangeThreshold);
  config_->relativeCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".rel_cost_change_thresh", config_->relativeCostChangeThreshold);
  // clang-format on

  // Make Qc_inv
  Eigen::Array<double, 1, 6> Qc_diag;
  Qc_diag << config_->lin_acc_std_dev_x, config_->lin_acc_std_dev_y,
      config_->lin_acc_std_dev_z, config_->ang_acc_std_dev_x,
      config_->ang_acc_std_dev_y, config_->ang_acc_std_dev_z;
  if (checkDiagonal(Qc_diag) == false && config_->trajectory_smoothing) {
    throw std::runtime_error(
        "Elements of the smoothing factor must be greater than zero!");
  }
  smoothing_factor_information_.setZero();
  smoothing_factor_information_.diagonal() = 1.0 / Qc_diag;
}

void OdometryICPModuleV2::runImpl(QueryCache &qdata0,
                                  const Graph::ConstPtr &graph) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (!qdata.curr_map_odo) {
    CLOG(INFO, "lidar.odometry_icp") << "First keyframe, simply return.";
    qdata.undistorted_point_cloud.emplace(*qdata.preprocessed_point_cloud);
    *qdata.odo_success = true;
    return;
  }

  // Inputs
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m = *qdata.T_r_m_odo;  // used as prior
  const auto &T_m_pm = qdata.curr_map_odo->T_vertex_map();
  auto &point_map = qdata.curr_map_odo->point_map();

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  size_t num_samples = config_->initial_num_samples;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;

  /// Create and add the T_robot_map variable, here map is in vertex frame.
  const auto T_r_m_var = boost::make_shared<TransformStateVar>(T_r_m);

  /// Create evaluators for passing into ICP
  auto T_s_r_eval = FixedTransformEvaluator::MakeShared(T_s_r);
  auto T_m_pm_eval = FixedTransformEvaluator::MakeShared(T_m_pm);
  auto T_r_m_eval = TransformStateEvaluator::MakeShared(T_r_m_var);
  // compound transform for alignment (sensor to point map transform)
  const auto T_pm_s_eval =
      inverse(compose(T_s_r_eval, compose(T_r_m_eval, T_m_pm_eval)));

  /// Priors
  auto prior_cost_terms = boost::make_shared<ParallelizedCostTermCollection>();
  std::map<unsigned int, StateVariableBase::Ptr> traj_state_vars;
  if (config_->trajectory_smoothing) {
    computeTrajectory(qdata, graph, T_r_m_eval, traj_state_vars,
                      prior_cost_terms);
    CLOG(DEBUG, "lidar.odometry_icp") << "Number of trajectory cost terms: "
                                      << prior_cost_terms->numCostTerms();
    CLOG(DEBUG, "lidar.odometry_icp")
        << "Number of state variables: " << traj_state_vars.size();
  }

  // Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto map_mat = point_map.getMatrixXfMap(3, 16, 0);
  const auto map_normals_mat = point_map.getMatrixXfMap(3, 16, 4);
  const auto query_mat = query_points.getMatrixXfMap(3, 16, 0);
  const auto query_norms_mat = query_points.getMatrixXfMap(3, 16, 4);
  auto aligned_mat = aligned_points.getMatrixXfMap(3, 16, 0);
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(3, 16, 4);

  /// Perform initial alignment (no motion distortion for the first iteration)
  const auto T_pm_s_init = T_pm_s_eval->evaluate().matrix();
  Eigen::Matrix3f C_pm_s_init = (T_pm_s_init.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f r_s_pm_in_pm_init =
      (T_pm_s_init.block<3, 1>(0, 3)).cast<float>();
  aligned_mat = (C_pm_s_init * query_mat).colwise() + r_s_pm_in_pm_init;
  aligned_norms_mat = C_pm_s_init * query_norms_mat;

  // ICP results
  EdgeTransform T_r_m_icp;
  float matched_points_ratio = 0.0;

  // Random generator
  std::default_random_engine generator;
  // generator.seed(0);
  std::vector<float> query_weights;
  query_weights.reserve(query_points.size());
  for (const auto &p : query_points) query_weights.emplace_back(p.icp_score);
  std::discrete_distribution<int> distribution(query_weights.begin(),
                                               query_weights.end());

  // Convergence variables
  float mean_dT = 0;
  float mean_dR = 0;
  Eigen::MatrixXd all_tfs = Eigen::MatrixXd::Zero(4, 4);
  bool refinement_stage = false;
  int refinement_step = 0;

  std::vector<common::timing::Stopwatch> timer(6);
  std::vector<std::string> clock_str;
  clock_str.push_back("Random Sample ..... ");
  clock_str.push_back("KNN Search ........ ");
  clock_str.push_back("Point Filtering ... ");
  clock_str.push_back("Optimization ...... ");
  clock_str.push_back("Alignment ......... ");
  clock_str.push_back("Check Convergence . ");

  /// create kd-tree of the map
  NanoFLANNAdapter<PointWithInfo> adapter(point_map);
  KDTreeParams tree_params(10 /* max leaf */);
  auto kdtree =
      std::make_unique<KDTree<PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

  for (int step = 0;; step++) {
    /// Points Association
    // Pick random queries (use unordered set to ensure uniqueness)
    timer[0].start();
    std::vector<std::pair<size_t, size_t>> sample_inds;
    if (num_samples < query_points.size()) {
      std::unordered_set<size_t> unique_inds;
      while (unique_inds.size() < num_samples)
        unique_inds.insert((size_t)distribution(generator));

      sample_inds = std::vector<std::pair<size_t, size_t>>(num_samples);
      size_t i = 0;
      for (const auto &ind : unique_inds) {
        sample_inds[i].first = ind;
        i++;
      }
    } else {
      sample_inds = std::vector<std::pair<size_t, size_t>>(query_points.size());
      for (size_t i = 0; i < query_points.size(); i++) sample_inds[i].first = i;
    }
    timer[0].stop();

    timer[1].start();
    // Init neighbors container
    std::vector<float> nn_dists(sample_inds.size());

    // Find nearest neigbors
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (size_t i = 0; i < sample_inds.size(); i++) {
      KDTreeResultSet result_set(1);
      result_set.init(&sample_inds[i].second, &nn_dists[i]);
      kdtree->findNeighbors(
          result_set, aligned_points[sample_inds[i].first].data, search_params);
    }
    timer[1].stop();

    /// Filtering based on distances metrics
    timer[2].start();
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
    timer[2].stop();

    /// Point to plane optimization
    timer[3].start();
    // shared loss function
    auto loss_func = boost::make_shared<L2LossFunc>();

    // cost terms and noise model
    auto cost_terms = boost::make_shared<ParallelizedCostTermCollection>();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &ind : filtered_sample_inds) {
      // noise model W = n * n.T (information matrix)
      Eigen::Vector3d nrm =
          map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      Eigen::Matrix3d W(
          point_map[ind.second].normal_score * (nrm * nrm.transpose()) +
          1e-5 * Eigen::Matrix3d::Identity());  // add a small value to prevent
                                                // numerical issues
      auto noise_model =
          boost::make_shared<StaticNoiseModel<3>>(W, INFORMATION);

      // query and reference point
      const auto &qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto &ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();

      PointToPointErrorEval2::Ptr error_func;
      if (config_->trajectory_smoothing) {
        const auto &qry_time = query_points[ind.first].time;
        const auto T_r_m_intp_eval =
            trajectory_->getInterpPoseEval(Time(qry_time));
        const auto T_pm_s_intp_eval =
            inverse(compose(T_s_r_eval, compose(T_r_m_intp_eval, T_m_pm_eval)));
        error_func.reset(
            new PointToPointErrorEval2(T_pm_s_intp_eval, ref_pt, qry_pt));
      } else {
        error_func.reset(
            new PointToPointErrorEval2(T_pm_s_eval, ref_pt, qry_pt));
      }

      // create cost term and add to problem
      auto cost = boost::make_shared<WeightedLeastSqCostTerm<3, 6>>(
          error_func, noise_model, loss_func);

#pragma omp critical(lgicp_add_cost_term)
      cost_terms->add(cost);
    }

    // initialize problem
    OptimizationProblem problem;
    problem.addStateVariable(T_r_m_var);
    problem.addCostTerm(cost_terms);
    // add prior costs
    if (config_->trajectory_smoothing) {
      problem.addCostTerm(prior_cost_terms);
      for (const auto &var : traj_state_vars)
        problem.addStateVariable(var.second);
    }

    using SolverType = VanillaGaussNewtonSolver;
    SolverType::Params params;
    params.verbose = false;
    params.maxIterations = 1;

    // Make solver
    SolverType solver(&problem, params);

    // Optimize
    try {
      solver.optimize();
    } catch (const decomp_failure &) {
      CLOG(WARNING, "lidar.odometry_icp")
          << "Steam optimization failed! T_pm_s left unchanged.";
    }

    timer[3].stop();

    timer[4].start();
    /// Alignment
    if (config_->trajectory_smoothing) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (unsigned i = 0; i < query_points.size(); i++) {
        const auto &qry_time = query_points[i].time;
        const auto T_r_m_intp_eval =
            trajectory_->getInterpPoseEval(Time(qry_time));
        const auto T_pm_s_intp_eval =
            inverse(compose(T_s_r_eval, compose(T_r_m_intp_eval, T_m_pm_eval)));
        const auto T_pm_s = T_pm_s_intp_eval->evaluate().matrix();
        Eigen::Matrix3f C_pm_s = T_pm_s.block<3, 3>(0, 0).cast<float>();
        Eigen::Vector3f r_s_pm_in_pm = T_pm_s.block<3, 1>(0, 3).cast<float>();
        aligned_mat.block<3, 1>(0, i) =
            C_pm_s * query_mat.block<3, 1>(0, i) + r_s_pm_in_pm;
        aligned_norms_mat.block<3, 1>(0, i) =
            C_pm_s * query_norms_mat.block<3, 1>(0, i);
      }
    } else {
      const auto T_pm_s = T_pm_s_eval->evaluate().matrix();
      Eigen::Matrix3f C_pm_s = T_pm_s.block<3, 3>(0, 0).cast<float>();
      Eigen::Vector3f r_s_pm_in_pm = T_pm_s.block<3, 1>(0, 3).cast<float>();
      aligned_mat = (C_pm_s * query_mat).colwise() + r_s_pm_in_pm;
      aligned_norms_mat = C_pm_s * query_norms_mat;
    }

    // Update all result matrices
    const auto T_pm_s = T_pm_s_eval->evaluate().matrix();
    if (step == 0)
      all_tfs = Eigen::MatrixXd(T_pm_s);
    else {
      Eigen::MatrixXd temp(all_tfs.rows() + 4, 4);
      temp.topRows(all_tfs.rows()) = all_tfs;
      temp.bottomRows(4) = Eigen::MatrixXd(T_pm_s);
      all_tfs = temp;
    }

    timer[4].stop();

    /// Check convergence
    timer[5].start();
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
        CLOG(DEBUG, "lidar.odometry_icp")
            << "Initial alignment takes " << step << " steps.";

        // enter the second refine stage
        refinement_stage = true;

        max_it = step + config_->refined_max_iter;
        // increase number of samples
        num_samples = config_->refined_num_samples;
        // reduce the max distance
        max_pair_d = config_->refined_max_pairing_dist;
        max_pair_d2 = max_pair_d * max_pair_d;
        max_planar_d = config_->refined_max_planar_dist;
      }
    }
    timer[5].stop();

    // Last step
    if ((refinement_stage && step >= max_it - 1) ||
        (refinement_step > config_->averaging_num_steps &&
         mean_dT < config_->trans_diff_thresh &&
         mean_dR < config_->rot_diff_thresh)) {
      CLOG(DEBUG, "lidar.odometry_icp")
          << "Total number of steps: " << step << ".";
      // result
      T_r_m_icp = EdgeTransform(T_r_m_var->getValue(),
                                solver.queryCovariance(T_r_m_var->getKey()));
      matched_points_ratio =
          (float)filtered_sample_inds.size() / (float)sample_inds.size();
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "lidar.odometry_icp")
            << "ICP did not converge to threshold, "
               "matched_points_ratio set to 0.";
        if (!refinement_stage) {
          CLOG(WARNING, "lidar.odometry_icp")
              << "ICP did not enter refinement stage at all.";
        }
        // matched_points_ratio = 0;
      }
      break;
    }
  }

  /// Dump timing info
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "lidar.odometry_icp")
        << clock_str[i] << timer[i].count() << "ms";
  }

  /// Outputs
  qdata.matched_points_ratio.emplace(matched_points_ratio);
  if (matched_points_ratio > config_->min_matched_ratio) {
    // store undistorted pointcloud
    const auto T_s_pm = T_pm_s_eval->evaluate().matrix().inverse();
    Eigen::Matrix3f C_s_pm = T_s_pm.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f r_pm_s_in_s = T_s_pm.block<3, 1>(0, 3).cast<float>();
    aligned_mat = (C_s_pm * aligned_mat).colwise() + r_pm_s_in_s;
    aligned_norms_mat = C_s_pm * aligned_norms_mat;
    qdata.undistorted_point_cloud.emplace(aligned_points);
    //
    *qdata.T_r_m_odo = T_r_m_icp;
    //
    *qdata.odo_success = true;
    //
    if (config_->trajectory_smoothing) qdata.trajectory = trajectory_;
  } else {
    CLOG(WARNING, "lidar.odometry_icp")
        << "Matched points ratio " << matched_points_ratio
        << " is below the threshold. ICP is considered failed.";
    // do not undistort the pointcloud
    qdata.undistorted_point_cloud.emplace(query_points);
    //
    // no update to map to robot transform
    //
    *qdata.odo_success = false;
  }
}

void OdometryICPModuleV2::computeTrajectory(
    LidarQueryCache &qdata, const Graph::ConstPtr &graph,
    const TransformEvaluator::Ptr &T_r_m_eval,
    std::map<unsigned int, StateVariableBase::Ptr> &state_vars,
    const ParallelizedCostTermCollection::Ptr &prior_cost_terms) {
  // reset the trajectory
  if (config_->use_constant_acc)
    trajectory_.reset(
        new SteamCATrajInterface(smoothing_factor_information_, true));
  else
    trajectory_.reset(
        new SteamTrajInterface(smoothing_factor_information_, true));

  // get the live vertex
  const auto live_vertex = graph->at(*qdata.live_id);
  CLOG(DEBUG, "lidar.odometry_icp") << "Looking at live id: " << *qdata.live_id;

  /// Set up a search for the previous keyframes in the graph
  auto tempeval = std::make_shared<TemporalEvaluator<GraphBase>>();
  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  using DirectionEvaluator =
      pose_graph::eval::Mask::DirectionFromVertexDirect<GraphBase>;
  auto direval = std::make_shared<DirectionEvaluator>(*qdata.live_id, true);
  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);
  evaluator->setGraph((void *)graph.get());

  // look back number of vertices
  const int temporal_depth = 3;

  // get the subgraph to work on first (to be thread safe)
  const auto subgraph =
      graph->getSubgraph(*qdata.live_id, temporal_depth, evaluator);
  evaluator->setGraph((void *)subgraph.get());

  std::map<VertexId, VectorSpaceStateVar::Ptr> velocity_map;
  std::map<VertexId, VectorSpaceStateVar::Ptr> acceleration_map;

  // perform the search and automatically step back one
  auto itr = subgraph->beginDfs(*qdata.live_id, temporal_depth, evaluator);
  itr++;

  // initialize the compunded transform
  EdgeTransform T_p_l = EdgeTransform(true);  // T_previous_live
  // initialize the timestamp that will be used as
  auto next_stamp = live_vertex->keyframeTime();

  // Trajectory is of the following form, where the origin = 0 is at m
  // which is the most recent keyframe in the graph.
  //                      local
  //       T_1_0     T_2_1     T_l_2      T_q_l
  //    0---------1---------2---------l----------q
  //  T_0_l     T_1_l     T_2_l       0        T_q_1
  //                      global
  // loop through all the found vertices
  for (; itr != subgraph->end(); ++itr) {
    // get the stamp of the vertex we're looking at
    auto prev_vertex = subgraph->at(itr->to());
    auto prev_stamp = prev_vertex->keyframeTime();

    // get the transform and compund it
    const auto &T_p_pp = itr->e()->T();
    T_p_l = T_p_pp.inverse() * T_p_l;

    // set up a locked global pose for this vertex, with an tf evaluator
    // Note: normally steam would have states T_a_0, T_b_0, ..., where 'a' and
    // 'b' are always sequential in time. So in our case, since our locked '0'
    // frame is in the future, 'a' is actually further from '0' than 'b'.
    const auto prev_pose = boost::make_shared<TransformStateVar>(T_p_l);
    prev_pose->setLock(true);
    const auto tf_prev = boost::make_shared<TransformStateEvaluator>(prev_pose);

    // time difference between next and previous
    int64_t next_prev_dt = next_stamp - prev_stamp;

    // generate a velocity estimate
    // The velocity is in the body frame, helping you get from 'a' to 'b'.
    // This part can ignore the fact that the transforms above are weird
    // (new to old instead of old to new), and keep doing vel_b_a.
    // we use p_pp instead of p_pm1 for convenience
    Eigen::Matrix<double, 6, 1> prev_velocity =
        T_p_pp.vec() / (next_prev_dt / 1e9);

    auto prev_frame_velocity =
        boost::make_shared<VectorSpaceStateVar>(prev_velocity);

    velocity_map.insert({prev_vertex->id(), prev_frame_velocity});

    // generate an acceleration map
    Eigen::Matrix<double, 6, 1> prev_acceleration =
        Eigen::Matrix<double, 6, 1>::Zero();

    auto prev_frame_acceleration =
        boost::make_shared<VectorSpaceStateVar>(prev_acceleration);

    acceleration_map.insert({prev_vertex->id(), prev_frame_acceleration});

    // make a steam time from the timstamp
    Time prev_time(static_cast<int64_t>(prev_stamp));

    // Add the poses to the trajectory
    trajectory_->add(prev_time, tf_prev, prev_frame_velocity,
                     prev_frame_acceleration);
    next_stamp = prev_stamp;

    CLOG(DEBUG, "lidar.odometry_icp")
        << "Looking at previous vertex id: " << prev_vertex->id() << std::endl
        << "T_previous_live: " << T_p_l.vec().transpose() << std::endl
        << "velocity: " << prev_velocity.transpose() << std::endl
        << "time difference (sec): " << next_prev_dt / 1e9;
  }
  // lock the velocity at the begining of the trajectory
  if (!velocity_map.empty()) {
    auto itr = velocity_map.begin();
    itr->second->setLock(true);
    CLOG(DEBUG, "lidar.odometry_icp")
        << "Locking the first velocity corresponding to vertex id: "
        << itr->first;
    itr++;
    for (; itr != velocity_map.end(); itr++) {
      CLOG(DEBUG, "lidar.odometry_icp")
          << "Adding velocity corresponding to vertex id: " << itr->first;
      state_vars[itr->second->getKey().getID()] = itr->second;
    }
  }
  // lock the acceleration at the begining of the trajectory
  if (config_->use_constant_acc && !acceleration_map.empty()) {
    auto itr = acceleration_map.begin();
    // itr->second->setLock(true);
    // CLOG(DEBUG, "lidar.odometry_icp")
    //   << "Locking the first acc corresponding to vertex id: " << itr->first;
    // itr++;
    for (; itr != acceleration_map.end(); itr++) {
      CLOG(DEBUG, "lidar.odometry_icp")
          << "Adding acceleration corresponding to vertex id: " << itr->first;
      state_vars[itr->second->getKey().getID()] = itr->second;
    }
  }

  // get the stamps
  const auto &query_stamp = *qdata.stamp;
  const auto &live_stamp = live_vertex->keyframeTime();
  // time difference between query and live frame
  int64_t query_live_dt = query_stamp - live_stamp;
  // generate velocity estimate
  Eigen::Matrix<double, 6, 1> query_velocity =
      (*qdata.T_r_m_odo).vec() / (query_live_dt / 1e9);
  CLOG(DEBUG, "lidar.odometry_icp")
      << "Adding query-live velocity: " << query_velocity.transpose()
      << ", time difference: " << query_live_dt / 1e9;
  // generate acceleration estimate
  Eigen::Matrix<double, 6, 1> query_acceleration =
      Eigen::Matrix<double, 6, 1>::Zero();

  Time live_time(static_cast<int64_t>(live_stamp));

  // Add the poses to the trajectory
  const auto live_pose =
      boost::make_shared<TransformStateVar>(lgmath::se3::Transformation());
  live_pose->setLock(true);  // lock the 'origin' pose
  const auto tf_live = boost::make_shared<TransformStateEvaluator>(live_pose);

  auto live_frame_velocity =
      boost::make_shared<VectorSpaceStateVar>(query_velocity);
  velocity_map.insert({*qdata.live_id, live_frame_velocity});
  state_vars[live_frame_velocity->getKey().getID()] = live_frame_velocity;

  auto live_frame_acceleration =
      boost::make_shared<VectorSpaceStateVar>(query_acceleration);
  acceleration_map.insert({*qdata.live_id, live_frame_acceleration});
  if (config_->use_constant_acc) {
    state_vars[live_frame_acceleration->getKey().getID()] =
        live_frame_acceleration;
  }

  trajectory_->add(live_time, tf_live, live_frame_velocity,
                   live_frame_acceleration);

  Time query_time(static_cast<int64_t>(query_stamp));

  auto query_frame_velocity =
      boost::make_shared<VectorSpaceStateVar>(query_velocity);
  velocity_map.insert({VertexId::Invalid(), query_frame_velocity});
  state_vars[query_frame_velocity->getKey().getID()] = query_frame_velocity;

  auto query_frame_acceleration =
      boost::make_shared<VectorSpaceStateVar>(query_acceleration);
  acceleration_map.insert({VertexId::Invalid(), query_frame_acceleration});
  if (config_->use_constant_acc) {
    state_vars[query_frame_acceleration->getKey().getID()] =
        query_frame_acceleration;
  }

  trajectory_->add(query_time, T_r_m_eval, query_frame_velocity,
                   query_frame_acceleration);

  // Trajectory prior smoothing terms
  trajectory_->appendPriorCostTerms(prior_cost_terms);
}

}  // namespace lidar
}  // namespace vtr