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
 * \file localization_icp_module.cpp
 * \brief LocalizationICPModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/localization_icp_module.hpp>

namespace vtr {
namespace lidar {

using namespace tactic;

void LocalizationICPModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config_->min_matched_ratio);

  config_->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config_->use_pose_prior);

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
}

void LocalizationICPModule::runImpl(QueryCache &qdata0,
                                    const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // Inputs
  // const auto &query_times = *qdata.preprocessed_pointcloud_time;
  const auto &query_points = *qdata.undistorted_pointcloud;
  const auto &query_normals = *qdata.undistorted_normals;
  const auto &query_weights = *qdata.icp_scores;
  const size_t query_num_pts = query_points.size();
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m = *qdata.T_r_m_loc;  // used as prior
  const auto T_m_pm = EdgeTransform(true);
  auto &map = *qdata.current_map_loc;
  const auto &map_weights = map.normal_scores;

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  size_t num_samples = config_->initial_num_samples;
  float max_pair_d2 =
      config_->initial_max_pairing_dist * config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  nanoflann::SearchParams search_params;  // kd-tree search parameters

  /// Create and add the T_robot_map variable, here map is in vertex frame.
  const auto T_r_m_var = std::make_shared<steam::se3::TransformStateVar>(T_r_m);

  /// Create evaluators for passing into ICP
  auto T_s_r_eval = steam::se3::FixedTransformEvaluator::MakeShared(T_s_r);
  auto T_m_pm_eval = steam::se3::FixedTransformEvaluator::MakeShared(T_m_pm);
  auto T_r_m_eval = steam::se3::TransformStateEvaluator::MakeShared(T_r_m_var);
  // compound transform for alignment (query to map transform)
  const auto T_mq_eval = steam::se3::inverse(steam::se3::compose(
      T_s_r_eval, steam::se3::compose(T_r_m_eval, T_m_pm_eval)));

  /// Priors
  steam::ParallelizedCostTermCollection::Ptr prior_cost_terms(
      new steam::ParallelizedCostTermCollection());
  /// pose prior term
  if (config_->use_pose_prior) {
    addPosePrior(T_r_m, T_r_m_eval, prior_cost_terms);
    CLOG(DEBUG, "lidar.localization_icp")
        << "Adding prior cost term: " << prior_cost_terms->numCostTerms();
  }

  // Initializ aligned points for matching (Deep copy of targets)
  std::vector<PointXYZ> aligned_points(query_points);
  std::vector<PointXYZ> aligned_normals(query_normals);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  using PCEigen = Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>;
  PCEigen map_mat((float *)map.cloud.pts.data(), 3, map.cloud.pts.size());
  PCEigen map_normals_mat((float *)map.normals.data(), 3, map.normals.size());
  PCEigen query_mat((float *)query_points.data(), 3, query_num_pts);
  PCEigen query_norms_mat((float *)query_normals.data(), 3, query_num_pts);
  PCEigen aligned_mat((float *)aligned_points.data(), 3, query_num_pts);
  PCEigen aligned_norms_mat((float *)aligned_normals.data(), 3, query_num_pts);

  /// Perform initial alignment (no motion distortion for the first iteration)
  const auto T_mq_init = T_mq_eval->evaluate().matrix();
  Eigen::Matrix3f C_mq_init = (T_mq_init.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f r_m_qm_init = (T_mq_init.block<3, 1>(0, 3)).cast<float>();
  aligned_mat = (C_mq_init * query_mat).colwise() + r_m_qm_init;
  aligned_norms_mat = C_mq_init * query_norms_mat;

  // ICP results
  EdgeTransform T_r_m_icp;
  float matched_points_ratio = 0.0;

  // Random generator
  std::default_random_engine generator;
  // generator.seed(0);
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

  for (int step = 0;; step++) {
    /// Points Association
    // Pick random queries (use unordered set to ensure uniqueness)
    timer[0].start();
    std::vector<std::pair<size_t, size_t>> sample_inds;
    if (num_samples < query_num_pts) {
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
      sample_inds = std::vector<std::pair<size_t, size_t>>(query_num_pts);
      for (size_t i = 0; i < query_num_pts; i++) sample_inds[i].first = i;
    }
    timer[0].stop();

    timer[1].start();
    // Init neighbors container
    std::vector<float> nn_dists(sample_inds.size());

    // Find nearest neigbors
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (size_t i = 0; i < sample_inds.size(); i++) {
      nanoflann::KNNResultSet<float> result_set(1);
      result_set.init(&sample_inds[i].second, &nn_dists[i]);
      map.tree.findNeighbors(result_set,
                             (float *)&aligned_points[sample_inds[i].first],
                             search_params);
    }
    timer[1].stop();

    /// Filtering based on distances metrics
    timer[2].start();
    // Erase sample_inds if dists is too big
    std::vector<std::pair<size_t, size_t>> filtered_sample_inds;
    filtered_sample_inds.reserve(sample_inds.size());
    float rms2 = 0;
    float prms2 = 0;
    for (size_t i = 0; i < sample_inds.size(); i++) {
      if (nn_dists[i] < max_pair_d2) {
        // Check planar distance (only after a few steps for initial
        // alignment)
        PointXYZ diff = (map.cloud.pts[sample_inds[i].second] -
                         aligned_points[sample_inds[i].first]);
        float planar_dist = abs(diff.dot(map.normals[sample_inds[i].second]));
        if (step < first_steps || planar_dist < max_planar_d) {
          // Keep samples
          filtered_sample_inds.push_back(sample_inds[i]);

          // Update pt2pt rms
          rms2 += nn_dists[i];

          // update pt2pl rms
          prms2 += planar_dist;
        }
      }
    }
    timer[2].stop();

    /// Point to plane optimization
    timer[3].start();
    // shared loss function
    steam::L2LossFunc::Ptr loss_func(new steam::L2LossFunc());

    // cost terms and noise model
    steam::ParallelizedCostTermCollection::Ptr cost_terms(
        new steam::ParallelizedCostTermCollection());
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &ind : filtered_sample_inds) {
      // noise model W = n * n.T (information matrix)
      Eigen::Vector3d nrm =
          map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
      Eigen::Matrix3d W(
          map_weights[ind.second] * (nrm * nrm.transpose()) +
          1e-5 * Eigen::Matrix3d::Identity());  // add a small value to prevent
                                                // numerical issues
      steam::BaseNoiseModel<3>::Ptr noise_model(
          new steam::StaticNoiseModel<3>(W, steam::INFORMATION));

      // query and reference point
      const auto &qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto &ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();

      steam::PointToPointErrorEval2::Ptr error_func;
      error_func.reset(
          new steam::PointToPointErrorEval2(T_mq_eval, ref_pt, qry_pt));

      // create cost term and add to problem
      steam::WeightedLeastSqCostTerm<3, 6>::Ptr cost(
          new steam::WeightedLeastSqCostTerm<3, 6>(error_func, noise_model,
                                                   loss_func));
#pragma omp critical(lgicp_add_cost_term)
      cost_terms->add(cost);
    }

    // initialize problem
    steam::OptimizationProblem problem;
    problem.addStateVariable(T_r_m_var);
    problem.addCostTerm(cost_terms);
    if (refinement_stage) {
      // add prior costs
      if (config_->use_pose_prior) problem.addCostTerm(prior_cost_terms);
    }

    using SolverType = steam::VanillaGaussNewtonSolver;
    SolverType::Params params;
    params.verbose = false;
    params.maxIterations = 1;

    // Make solver
    SolverType solver(&problem, params);

    // Optimize
    try {
      solver.optimize();
    } catch (const steam::decomp_failure &) {
      CLOG(WARNING, "lidar.localization_icp")
          << "Steam optimization failed! T_mq left unchanged.";
    }

    timer[3].stop();

    timer[4].start();
    /// Alignment
    const auto T_mq = T_mq_eval->evaluate().matrix();
    Eigen::Matrix3f C_mq = T_mq.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f r_m_qm = T_mq.block<3, 1>(0, 3).cast<float>();
    aligned_mat = (C_mq * query_mat).colwise() + r_m_qm;
    aligned_norms_mat = C_mq * query_norms_mat;

    // Update all result matrices
    // const auto T_mq = T_mq_eval->evaluate().matrix(); // defined above
    if (step == 0)
      all_tfs = Eigen::MatrixXd(T_mq);
    else {
      Eigen::MatrixXd temp(all_tfs.rows() + 4, 4);
      temp.topRows(all_tfs.rows()) = all_tfs;
      temp.bottomRows(4) = Eigen::MatrixXd(T_mq);
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
        max_pair_d2 = config_->refined_max_pairing_dist *
                      config_->refined_max_pairing_dist;
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

  /// Outputs
  // Whether ICP is successful
  qdata.matched_points_ratio.fallback(matched_points_ratio);
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

void LocalizationICPModule::addPosePrior(
    const EdgeTransform &T_r_m,
    const steam::se3::TransformEvaluator::Ptr &T_r_m_eval,
    const steam::ParallelizedCostTermCollection::Ptr &prior_cost_terms) {
  steam::L2LossFunc::Ptr loss_func(new steam::L2LossFunc());
  steam::BaseNoiseModel<6>::Ptr noise_model(
      new steam::StaticNoiseModel<6>(T_r_m.cov()));
  steam::TransformErrorEval::Ptr error_func(
      new steam::TransformErrorEval(T_r_m, T_r_m_eval));
  // Create cost term and add to problem
  steam::WeightedLeastSqCostTerm<6, 6>::Ptr prior_cost(
      new steam::WeightedLeastSqCostTerm<6, 6>(error_func, noise_model,
                                               loss_func));
  prior_cost_terms->add(prior_cost);
}

}  // namespace lidar
}  // namespace vtr