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
 * \file localization_icp_module_v2.cpp
 * \brief LocalizationICPModuleV2 class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/localization_icp_module_v2.hpp"

#include "vtr_lidar_msgs/msg/icp_result.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;
using namespace steam;
using namespace se3;

using LocalizationICPResult = vtr_lidar_msgs::msg::ICPResult;

auto LocalizationICPModuleV2::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);

  config->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config->use_pose_prior);

  // icp params
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
#ifdef VTR_DETERMINISTIC
  LOG_IF(config->num_threads != 1, WARNING) << "ICP number of threads set to 1 in deterministic mode.";
  config->num_threads = 1;
#endif
  config->first_num_steps = node->declare_parameter<int>(param_prefix + ".first_num_steps", config->first_num_steps);
  config->initial_max_iter = node->declare_parameter<int>(param_prefix + ".initial_max_iter", config->initial_max_iter);
  config->initial_num_samples = node->declare_parameter<int>(param_prefix + ".initial_num_samples", config->initial_num_samples);
  config->initial_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".initial_max_pairing_dist", config->initial_max_pairing_dist);
  config->initial_max_planar_dist = node->declare_parameter<float>(param_prefix + ".initial_max_planar_dist", config->initial_max_planar_dist);
  config->refined_max_iter = node->declare_parameter<int>(param_prefix + ".refined_max_iter", config->refined_max_iter);
  config->refined_num_samples = node->declare_parameter<int>(param_prefix + ".refined_num_samples", config->refined_num_samples);
  config->refined_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".refined_max_pairing_dist", config->refined_max_pairing_dist);
  config->refined_max_planar_dist = node->declare_parameter<float>(param_prefix + ".refined_max_planar_dist", config->refined_max_planar_dist);

  config->averaging_num_steps = node->declare_parameter<int>(param_prefix + ".averaging_num_steps", config->averaging_num_steps);
  config->rot_diff_thresh = node->declare_parameter<float>(param_prefix + ".rot_diff_thresh", config->rot_diff_thresh);
  config->trans_diff_thresh = node->declare_parameter<float>(param_prefix + ".trans_diff_thresh", config->trans_diff_thresh);

  // steam params
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", config->verbose);
  config->maxIterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", config->maxIterations);
  config->absoluteCostThreshold = node->declare_parameter<double>(param_prefix + ".abs_cost_thresh", config->absoluteCostThreshold);
  config->absoluteCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".abs_cost_change_thresh", config->absoluteCostChangeThreshold);
  config->relativeCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".rel_cost_change_thresh", config->relativeCostChangeThreshold);
  // clang-format on

  return config;
}

void LocalizationICPModuleV2::runImpl(QueryCache &qdata0, OutputCache &,
                                      const Graph::Ptr &graph,
                                      const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // Inputs
  const auto &stamp = *qdata.stamp;
  const auto &query_points = *qdata.undistorted_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m = *qdata.T_r_m_loc;  // used as prior
  const auto &T_m_pm = qdata.curr_map_loc->T_vertex_map();
  const auto &map_version = qdata.curr_map_loc->version();
  auto &point_map = qdata.curr_map_loc->point_map();

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  size_t num_samples = config_->initial_num_samples;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  KDTreeSearchParams search_params;

  /// Create and add the T_robot_map variable, here map is in vertex frame.
  const auto T_r_m_var = std::make_shared<TransformStateVar>(T_r_m);

  /// Create evaluators for passing into ICP
  auto T_s_r_eval = FixedTransformEvaluator::MakeShared(T_s_r);
  auto T_m_pm_eval = FixedTransformEvaluator::MakeShared(T_m_pm);
  auto T_r_m_eval = TransformStateEvaluator::MakeShared(T_r_m_var);
  // compound transform for alignment (sensor to point map transform)
  const auto T_pm_s_eval =
      inverse(compose(T_s_r_eval, compose(T_r_m_eval, T_m_pm_eval)));

  /// Priors
  auto prior_cost_terms = std::make_shared<ParallelizedCostTermCollection>();
  /// pose prior term
  if (config_->use_pose_prior) {
    addPosePrior(T_r_m, T_r_m_eval, prior_cost_terms);
    CLOG(DEBUG, "lidar.localization_icp")
        << "Adding prior cost term: " << prior_cost_terms->numCostTerms();
  }

  // Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto map_mat = point_map.getMatrixXfMap(
      3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(
      3, PointWithInfo::size(), PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(
      3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(
      3, PointWithInfo::size(), PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(
      3, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(
      3, PointWithInfo::size(), PointWithInfo::normal_offset());

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
    auto loss_func = std::make_shared<L2LossFunc>();

    // cost terms and noise model
    auto cost_terms = std::make_shared<ParallelizedCostTermCollection>();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &ind : filtered_sample_inds) {
      // noise model W = n * n.T (information matrix)
      Eigen::Vector3d nrm =
          map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      Eigen::Matrix3d W(
          point_map[ind.second].normal_score * (nrm * nrm.transpose()) +
          1e-5 * Eigen::Matrix3d::Identity());  // add a small value to prevent
                                                // numerical issues
      auto noise_model = std::make_shared<StaticNoiseModel<3>>(W, INFORMATION);

      // query and reference point
      const auto &qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto &ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();

      PointToPointErrorEval2::Ptr error_func;
      error_func.reset(new PointToPointErrorEval2(T_pm_s_eval, ref_pt, qry_pt));

      // create cost term and add to problem
      auto cost = std::make_shared<WeightedLeastSqCostTerm<3, 6>>(
          error_func, noise_model, loss_func);

#pragma omp critical(lgicp_add_cost_term)
      cost_terms->add(cost);
    }

    // initialize problem
    OptimizationProblem problem;
    problem.addStateVariable(T_r_m_var);
    problem.addCostTerm(cost_terms);
    if (refinement_stage) {
      // add prior costs
      if (config_->use_pose_prior) problem.addCostTerm(prior_cost_terms);
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
      CLOG(WARNING, "lidar.localization_icp")
          << "Steam optimization failed! T_pm_s left unchanged.";
    }

    timer[3].stop();

    timer[4].start();
    /// Alignment
    const auto T_pm_s = T_pm_s_eval->evaluate().matrix();
    Eigen::Matrix3f C_pm_s = T_pm_s.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f r_s_pm_in_pm = T_pm_s.block<3, 1>(0, 3).cast<float>();
    aligned_mat = (C_pm_s * query_mat).colwise() + r_s_pm_in_pm;
    aligned_norms_mat = C_pm_s * query_norms_mat;

    // Update all result matrices
    // const auto T_pm_s = T_pm_s_eval->evaluate().matrix(); // defined above
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
        CLOG(DEBUG, "lidar.localization_icp")
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
      CLOG(DEBUG, "lidar.localization_icp")
          << "Total number of steps: " << step << ".";
      // result
      T_r_m_icp = EdgeTransform(T_r_m_var->getValue(),
                                solver.queryCovariance(T_r_m_var->getKey()));
      matched_points_ratio =
          (float)filtered_sample_inds.size() / (float)sample_inds.size();
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "lidar.localization_icp")
            << "ICP did not converge to threshold, "
               "matched_points_ratio set to 0.";
        if (!refinement_stage) {
          CLOG(WARNING, "lidar.localization_icp")
              << "ICP did not enter refinement stage at all.";
        }
        // matched_points_ratio = 0;
      }
      break;
    }
  }

  /// Dump timing info
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "lidar.localization_icp")
        << clock_str[i] << timer[i].count() << "ms";
  }

  /// \todo DEBUGGING: dump the alignment results analysis
  {
    auto icp_result = std::make_shared<LocalizationICPResult>();
    icp_result->timestamp = stamp;
    icp_result->map_version = map_version;
    icp_result->nn_inds.resize(aligned_points.size());
    icp_result->nn_dists.resize(aligned_points.size());
    icp_result->nn_planar_dists.resize(aligned_points.size());

    // Find nearest neigbors
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (size_t i = 0; i < aligned_points.size(); i++) {
      KDTreeResultSet result_set(1);
      result_set.init(&icp_result->nn_inds[i], &icp_result->nn_dists[i]);
      kdtree->findNeighbors(result_set, aligned_points[i].data, search_params);

      auto diff = aligned_points[i].getVector3fMap() -
                  point_map[icp_result->nn_inds[i]].getVector3fMap();
      float planar_dist = abs(
          diff.dot(point_map[icp_result->nn_inds[i]].getNormalVector3fMap()));
      // planar distance update
      icp_result->nn_planar_dists[i] = planar_dist;
    }

    // Store result into the current run (not associated with a vertex), slow!
    const auto curr_run = graph->runs()->sharedLocked().get().rbegin()->second;
    CLOG(DEBUG, "lidar.localization_icp")
        << "Saving ICP localization result to run " << curr_run->id();
    using LocICPResLM = storage::LockableMessage<LocalizationICPResult>;
    auto msg = std::make_shared<LocICPResLM>(icp_result, stamp);
    curr_run->write<LocalizationICPResult>("localization_icp_result",
                                           "vtr_lidar_msgs/msg/ICPResult", msg);
  }

  /// Outputs
  qdata.matched_points_ratio.emplace(matched_points_ratio);
  CLOG(DEBUG, "lidar.localization_icp")
      << "Matched points ratio " << matched_points_ratio;
  if (matched_points_ratio > config_->min_matched_ratio) {
    // update map to robot transform
    *qdata.T_r_m_loc = T_r_m_icp;
    // set success
    *qdata.loc_success = true;
  } else {
    CLOG(WARNING, "lidar.localization_icp")
        << "Matched points ratio " << matched_points_ratio
        << " is below the threshold. ICP is considered failed.";
    // no update to map to robot transform
    // set success
    *qdata.loc_success = false;
  }
}

void LocalizationICPModuleV2::addPosePrior(
    const EdgeTransform &T_r_m, const TransformEvaluator::Ptr &T_r_m_eval,
    const ParallelizedCostTermCollection::Ptr &prior_cost_terms) {
  auto loss_func = std::make_shared<L2LossFunc>();
  auto noise_model = std::make_shared<StaticNoiseModel<6>>(T_r_m.cov());
  auto error_func = std::make_shared<TransformErrorEval>(T_r_m, T_r_m_eval);
  // Create cost term and add to problem
  auto prior_cost = std::make_shared<WeightedLeastSqCostTerm<6, 6>>(
      error_func, noise_model, loss_func);
  prior_cost_terms->add(prior_cost);
}

}  // namespace lidar
}  // namespace vtr