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
 * \author Yuchen Wu, Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar_lidar/modules/localization/localization_icp_module.hpp"

namespace vtr {
namespace radar_lidar {

using namespace tactic;
using namespace steam;
using namespace se3;

auto LocalizationICPModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config->min_matched_ratio);

  config->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config->use_pose_prior);

  config->elevation_threshold = node->declare_parameter<float>(param_prefix + ".elevation_threshold", config->elevation_threshold);
  config->normal_threshold = node->declare_parameter<float>(param_prefix + ".normal_threshold", config->normal_threshold);

  // icp params
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
#ifdef VTR_DETERMINISTIC
  CLOG_IF(config->num_threads != 1, WARNING, "radar_lidar.localization_icp") << "ICP number of threads set to 1 in deterministic mode.";
  config->num_threads = 1;
#endif
  config->first_num_steps = node->declare_parameter<int>(param_prefix + ".first_num_steps", config->first_num_steps);
  config->initial_max_iter = node->declare_parameter<int>(param_prefix + ".initial_max_iter", config->initial_max_iter);
  config->initial_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".initial_max_pairing_dist", config->initial_max_pairing_dist);
  config->initial_max_planar_dist = node->declare_parameter<float>(param_prefix + ".initial_max_planar_dist", config->initial_max_planar_dist);
  config->refined_max_iter = node->declare_parameter<int>(param_prefix + ".refined_max_iter", config->refined_max_iter);
  config->refined_max_pairing_dist = node->declare_parameter<float>(param_prefix + ".refined_max_pairing_dist", config->refined_max_pairing_dist);
  config->refined_max_planar_dist = node->declare_parameter<float>(param_prefix + ".refined_max_planar_dist", config->refined_max_planar_dist);
  config->huber_delta = node->declare_parameter<double>(param_prefix + ".huber_delta", config->huber_delta);

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

  return config;
}

void LocalizationICPModule::run_(QueryCache &qdata0, OutputCache &,
                                 const Graph::Ptr &,
                                 const TaskExecutor::Ptr &) {
  auto &radar_qdata = dynamic_cast<radar::RadarQueryCache &>(qdata0);
  auto &lidar_qdata = dynamic_cast<lidar::LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    tmp_scan_pub_ = radar_qdata.node->create_publisher<PointCloudMsg>("curr_scan_loc", 5);
    map_pub_ = radar_qdata.node->create_publisher<PointCloudMsg>("curr_map_loc_filtered", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Inputs
  // const auto &query_stamp = *radar_qdata.stamp;
  const auto &query_points = *radar_qdata.undistorted_point_cloud;
  const auto &T_s_r = *radar_qdata.T_s_r;
  const auto &T_v_m = lidar_qdata.curr_map_loc->T_vertex_map();
  // const auto &map_version = lidar_qdata.curr_map_loc->version();
  auto &lidar_point_map = lidar_qdata.curr_map_loc->point_map();
  /// \note this may be used as a prior
  auto T_r_v = *radar_qdata.T_r_v_loc;

  // se3 projection
#if true
  {
    auto T_v_r_vec = T_r_v.inverse().vec();
    auto T_v_r_cov = T_r_v.inverse().cov();
    CLOG(DEBUG, "radar_lidar.localization_icp") << "T_v_r_vec: \n" << T_v_r_vec;
    // basically setting z, pitch, roll to zero
    T_v_r_vec(2) = 0;
    T_v_r_vec(3) = 0;
    T_v_r_vec(4) = 0;
    // corresponding covariances to zero
    // for (int i = 0; i < 6; ++i) {
    //   for (int j = 2; j < 5; ++j) {
    //     T_v_r_cov(i, j) = 1e-5;
    //     T_v_r_cov(j, i) = 1e-5;
    //   }
    // }
    EdgeTransform T_v_r(T_v_r_vec, T_v_r_cov);
    CLOG(DEBUG, "radar_lidar.localization_icp") << "T_v_r projected: \n"
                                                << T_v_r;
    T_r_v = T_v_r.inverse();
  }
#else
  // project robot to vertex frame, along z-axis of the vertex fram while
  // keeping x-axis of robot frame in the same direction
  {
    auto T_v_r_mat = T_r_v.inverse().matrix();
    // origin x and y should be the same
    // origin z should be zero
    T_v_r_mat(2, 3) = 0;
    // x-axis should be in the same direction
    const auto planar_xaxis_norm = T_v_r_mat.block<2, 1>(0, 0).norm();
    T_v_r_mat(0, 0) /= planar_xaxis_norm;
    T_v_r_mat(1, 0) /= planar_xaxis_norm;
    T_v_r_mat(2, 0) = 0;
    // z-axis should be (0, 0, 1)
    T_v_r_mat(0, 2) = 0;
    T_v_r_mat(1, 2) = 0;
    T_v_r_mat(2, 2) = 1;
    // y-axis should be the cross of z-axis and x-axis
    T_v_r_mat.block<3, 1>(0, 1) =
        T_v_r_mat.block<3, 1>(0, 2).cross(T_v_r_mat.block<3, 1>(0, 0));
    // we no longer have a valid covariance
    EdgeTransform T_v_r(T_v_r_mat, Eigen::Matrix<double, 6, 6>::Identity());
    CLOG(DEBUG, "radar_lidar.localization_icp") << "T_v_r: \n"
                                                << T_r_v.inverse().matrix();
    CLOG(DEBUG, "radar_lidar.localization_icp") << "T_v_r projected: \n"
                                                << T_v_r.matrix();
    T_r_v = T_v_r.inverse();
  }
#endif
  // find points that are within the radar scan FOV
  std::vector<int> indices;
  {
    const auto T_s_m = (T_s_r * T_r_v * T_v_m).matrix();
    Eigen::Matrix3f C_s_m = (T_s_m.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_m_s_in_s = (T_s_m.block<3, 1>(0, 3)).cast<float>();
    for (int i = 0; i < (int)lidar_point_map.size(); ++i) {
      const auto &point = lidar_point_map.at(i);
      // point and normal in radar frame
      Eigen::Vector3f p_in_s = C_s_m * point.getVector3fMap() + r_m_s_in_s;
      Eigen::Vector3f n_in_s = C_s_m * point.getNormalVector3fMap();
      // filter by elevation
      // xy = np.sqrt(np.sum(xyz[:, :2] ** 2, axis=1))
      // ele = np.arctan2(xyz[:, 2], xy)
      // mask = np.abs(ele) < thres
      const auto elev = std::atan2(
          p_in_s(2), std::sqrt(p_in_s(0) * p_in_s(0) + p_in_s(1) * p_in_s(1)));
      if (std::abs(elev) > config_->elevation_threshold) continue;

      // filter by normal vector
      if (std::abs(n_in_s(2)) > config_->normal_threshold) continue;

      // // filter by z value
      // if (p_in_s(2) < -0.5 || p_in_s(2) > 0.5) continue;

      indices.emplace_back(i);
    }
  }
  pcl::PointCloud<lidar::PointWithInfo> point_map(lidar_point_map, indices);

  // project points to 2D
  {
    auto aligned_point_map = point_map;
    // clang-format off
    auto map_mat = point_map.getMatrixXfMap(3, lidar::PointWithInfo::size(), lidar::PointWithInfo::cartesian_offset());
    auto map_normals_mat = point_map.getMatrixXfMap(3, lidar::PointWithInfo::size(), lidar::PointWithInfo::normal_offset());
    auto aligned_map_mat = aligned_point_map.getMatrixXfMap(3, lidar::PointWithInfo::size(), lidar::PointWithInfo::cartesian_offset());
    auto aligned_map_normals_mat = aligned_point_map.getMatrixXfMap(3, lidar::PointWithInfo::size(), lidar::PointWithInfo::normal_offset());
    // clang-format on
    // convert to sensor frame
    const auto T_s_m = (T_s_r * T_r_v * T_v_m).matrix();
    Eigen::Matrix3f C_s_m = (T_s_m.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_m_s_in_s = (T_s_m.block<3, 1>(0, 3)).cast<float>();
    aligned_map_mat = (C_s_m * map_mat).colwise() + r_m_s_in_s;
    aligned_map_normals_mat = C_s_m * map_normals_mat;

    // project to 2D
    aligned_map_mat.row(2).setZero();
    aligned_map_normals_mat.row(2).setZero();
    // \todo double check correctness of this normal projection
    Eigen::MatrixXf aligned_map_norms_mat =
        aligned_map_normals_mat.colwise().norm();
    aligned_map_normals_mat.row(0).array() /= aligned_map_norms_mat.array();
    aligned_map_normals_mat.row(1).array() /= aligned_map_norms_mat.array();

    // convert back to point map frame
    const auto T_m_s = T_s_m.inverse();
    Eigen::Matrix3f C_m_s = (T_m_s.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_s_m_in_m = (T_m_s.block<3, 1>(0, 3)).cast<float>();
    map_mat = (C_m_s * aligned_map_mat).colwise() + r_s_m_in_m;
    map_normals_mat = C_m_s * aligned_map_normals_mat;
  }

  if (config_->visualize) {
    // clang-format off
#if false
    auto query_points_in_v = query_points;
    auto query_point_mat = query_points_in_v.getMatrixXfMap(3, radar::PointWithInfo::size(), radar::PointWithInfo::cartesian_offset());
    Eigen::Matrix4d T_v_s_mat = (T_s_r * T_r_v).inverse().matrix();
    Eigen::Matrix3f C_v_s = (T_v_s_mat.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_s_v_in_v = (T_v_s_mat.block<3, 1>(0, 3)).cast<float>();
    query_point_mat = (C_v_s * query_point_mat).colwise() + r_s_v_in_v;
#endif
    auto point_map_in_v = point_map;  // makes a copy
    auto map_point_mat = point_map_in_v.getMatrixXfMap(3, lidar::PointWithInfo::size(), lidar::PointWithInfo::cartesian_offset());
    Eigen::Matrix4d T_v_m_mat = T_v_m.matrix();
    Eigen::Matrix3f C_v_m = (T_v_m_mat.block<3, 3>(0, 0)).cast<float>();
    Eigen::Vector3f r_m_v_in_v = (T_v_m_mat.block<3, 1>(0, 3)).cast<float>();
    map_point_mat = (C_v_m * map_point_mat).colwise() + r_m_v_in_v;

    int check = 0;
#if false
    while (check < 10) {
      CLOG(INFO, "radar_lidar.localization_icp") << "Publish map!!!";
      {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(query_points_in_v, pc2_msg);
      pc2_msg.header.frame_id = "localization keyframe (offset)";
      pc2_msg.header.stamp = rclcpp::Time(*radar_qdata.stamp + check);
      tmp_scan_pub_->publish(pc2_msg);
      }
#endif
      {
      PointCloudMsg pc2_msg;
      pcl::toROSMsg(point_map_in_v, pc2_msg);
      pc2_msg.header.frame_id = "localization keyframe (offset)";
      pc2_msg.header.stamp = rclcpp::Time(*radar_qdata.stamp + check);
      map_pub_->publish(pc2_msg);
      }
#if false
      check += 1;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
#endif
    // clang-format on
  }

  /// Parameters
  int first_steps = config_->first_num_steps;
  int max_it = config_->initial_max_iter;
  float max_pair_d = config_->initial_max_pairing_dist;
  float max_planar_d = config_->initial_max_planar_dist;
  float max_pair_d2 = max_pair_d * max_pair_d;
  lidar::KDTreeSearchParams search_params;
  // clang-format off
  /// Create and add the T_robot_map variable, here m = vertex frame.
  const auto T_r_v_var = std::make_shared<TransformStateVar>(T_r_v);

  /// Create evaluators for passing into ICP
  const auto T_s_r_eval = FixedTransformEvaluator::MakeShared(T_s_r);
  const auto T_v_m_eval = FixedTransformEvaluator::MakeShared(T_v_m);
  const auto T_r_v_eval = TransformStateEvaluator::MakeShared(T_r_v_var);
  // compound transform for alignment (sensor to point map transform)
  const auto T_m_s_eval = inverse(compose(T_s_r_eval, compose(T_r_v_eval, T_v_m_eval)));

  /// use odometry as a prior
  auto prior_cost_terms = std::make_shared<ParallelizedCostTermCollection>();
  if (config_->use_pose_prior) {
    auto loss_func = std::make_shared<L2LossFunc>();
    auto noise_model = std::make_shared<StaticNoiseModel<6>>(T_r_v.cov());
    auto error_func = std::make_shared<TransformErrorEval>(T_r_v, T_r_v_eval);
    auto cost = std::make_shared<WeightedLeastSqCostTerm<6, 6>>(error_func, noise_model, loss_func);
    prior_cost_terms->add(cost);

    CLOG(DEBUG, "radar_lidar.localization_icp")
        << "Adding prior cost term: " << prior_cost_terms->numCostTerms();
  }

  // Initialize aligned points for matching (Deep copy of targets)
  pcl::PointCloud<radar::PointWithInfo> aligned_points(query_points);

  /// Eigen matrix of original data (only shallow copy of ref clouds)
  const auto map_mat = point_map.getMatrixXfMap(3, lidar::PointWithInfo::size(), lidar::PointWithInfo::cartesian_offset());
  const auto map_normals_mat = point_map.getMatrixXfMap(3, lidar::PointWithInfo::size(), lidar::PointWithInfo::normal_offset());
  const auto query_mat = query_points.getMatrixXfMap(3, radar::PointWithInfo::size(), radar::PointWithInfo::cartesian_offset());
  const auto query_norms_mat = query_points.getMatrixXfMap(3, radar::PointWithInfo::size(), radar::PointWithInfo::normal_offset());
  auto aligned_mat = aligned_points.getMatrixXfMap(3, radar::PointWithInfo::size(), radar::PointWithInfo::cartesian_offset());
  auto aligned_norms_mat = aligned_points.getMatrixXfMap(3, radar::PointWithInfo::size(), radar::PointWithInfo::normal_offset());

  /// Perform initial alignment (no motion distortion for the first iteration)
  const auto T_m_s_init = T_m_s_eval->evaluate().matrix();
  Eigen::Matrix3f C_m_s_init = (T_m_s_init.block<3, 3>(0, 0)).cast<float>();
  Eigen::Vector3f r_s_m_in_m_init = (T_m_s_init.block<3, 1>(0, 3)).cast<float>();
  aligned_mat = (C_m_s_init * query_mat).colwise() + r_s_m_in_m_init;
  aligned_norms_mat = C_m_s_init * query_norms_mat;

  // ICP results
  EdgeTransform T_r_v_icp;
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
  clock_str.push_back("Random Sample ..... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("KNN Search ........ ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Point Filtering ... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Optimization ...... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Alignment ......... ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));
  clock_str.push_back("Check Convergence . ");
  timer.emplace_back(std::make_unique<Stopwatch>(false));

  /// create kd-tree of the map
  lidar::NanoFLANNAdapter<lidar::PointWithInfo> adapter(point_map);
  lidar::KDTreeParams tree_params(/* max leaf */ 10);
  auto kdtree = std::make_unique<lidar::KDTree<lidar::PointWithInfo>>(3, adapter, tree_params);
  kdtree->buildIndex();

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
      lidar::KDTreeResultSet result_set(1);
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
        // // Check planar distance (only after a few steps for initial alignment)
        // auto diff = aligned_points[sample_inds[i].first].getVector3fMap() -
        //             point_map[sample_inds[i].second].getVector3fMap();
        // float planar_dist = abs(
        //     diff.dot(point_map[sample_inds[i].second].getNormalVector3fMap()));
        // if (step < first_steps || planar_dist < max_planar_d) {
          filtered_sample_inds.push_back(sample_inds[i]);
        // }
      }
    }
    timer[2]->stop();

    /// Point to point optimization
    timer[3]->start();
    // shared loss function
    auto loss_func = std::make_shared<HuberLossFunc>(config_->huber_delta);
    // cost terms and noise model
    auto cost_terms = std::make_shared<ParallelizedCostTermCollection>();
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &ind : filtered_sample_inds) {
      // noise model W = n * n.T (information matrix)
      Eigen::Matrix3d W = [&] {
        // point to line
        // if (point_map[ind.second].normal_score > 0) {
        //   Eigen::Vector3d nrm = map_normals_mat.block<3, 1>(0, ind.second).cast<double>();
        //   return Eigen::Matrix3d((nrm * nrm.transpose()) + 1e-5 * Eigen::Matrix3d::Identity());
        // } else {
        //   Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
        //   W(2, 2) = 1e-5;
        //   return W;
        // }
        /// point to point
        Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
        return W;
      }();
      auto noise_model = std::make_shared<StaticNoiseModel<3>>(W, INFORMATION);

      // query and reference point
      const auto &qry_pt = query_mat.block<3, 1>(0, ind.first).cast<double>();
      const auto &ref_pt = map_mat.block<3, 1>(0, ind.second).cast<double>();

      PointToPointErrorEval2::Ptr error_func;
      error_func.reset(new PointToPointErrorEval2(T_m_s_eval, ref_pt, qry_pt));

      // create cost term and add to problem
      auto cost = std::make_shared<WeightedLeastSqCostTerm<3, 6>>(error_func, noise_model, loss_func);

#pragma omp critical(lgicp_add_cost_term)
      cost_terms->add(cost);
    }

    // initialize problem
    OptimizationProblem problem;
    problem.addStateVariable(T_r_v_var);
    problem.addCostTerm(cost_terms);

    // add prior costs
    if (config_->use_pose_prior) problem.addCostTerm(prior_cost_terms);

    // make solver
    using SolverType = VanillaGaussNewtonSolver;
    SolverType::Params params;
    params.verbose = config_->verbose;
    params.maxIterations = config_->maxIterations;
    SolverType solver(&problem, params);

    // optimize
    try {
      solver.optimize();
    } catch (const decomp_failure &) {
      CLOG(WARNING, "radar_lidar.localization_icp")
          << "Steam optimization failed! T_m_s left unchanged.";
    }
    timer[3]->stop();

    /// Alignment
    timer[4]->start();
    const auto T_m_s = T_m_s_eval->evaluate().matrix();
    Eigen::Matrix3f C_m_s = T_m_s.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f r_s_m_in_m = T_m_s.block<3, 1>(0, 3).cast<float>();
    aligned_mat = (C_m_s * query_mat).colwise() + r_s_m_in_m;
    aligned_norms_mat = C_m_s * query_norms_mat;

    // update all result matrices
    // const auto T_m_s = T_m_s_eval->evaluate().matrix(); // defined above
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
    // update variations
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

    // refininement incremental
    if (refinement_stage) refinement_step++;

    // Stop condition
    if (!refinement_stage && step >= first_steps) {
      if ((step >= max_it - 1) || (mean_dT < config_->trans_diff_thresh &&
                                   mean_dR < config_->rot_diff_thresh)) {
        CLOG(DEBUG, "radar_lidar.localization_icp") << "Initial alignment takes " << step << " steps.";

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
    if ((refinement_stage && step >= max_it - 1) ||
        (refinement_step > config_->averaging_num_steps &&
         mean_dT < config_->trans_diff_thresh &&
         mean_dR < config_->rot_diff_thresh)) {
      CLOG(DEBUG, "radar_lidar.localization_icp") << "Total number of steps: " << step << ".";
      // result
      T_r_v_icp = EdgeTransform(T_r_v_var->getValue(), solver.queryCovariance(T_r_v_var->getKey()));
      matched_points_ratio = (float)filtered_sample_inds.size() / (float)sample_inds.size();
      if (mean_dT >= config_->trans_diff_thresh ||
          mean_dR >= config_->rot_diff_thresh) {
        CLOG(WARNING, "radar_lidar.localization_icp") << "ICP did not converge to the specified threshold.";
        if (!refinement_stage) {
          CLOG(WARNING, "radar_lidar.localization_icp") << "ICP did not enter refinement stage at all.";
        }
      }
      break;
    }
  }

  /// Dump timing info
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "radar_lidar.localization_icp") << clock_str[i] << timer[i]->count();
  }

  /// Outputs
  CLOG(DEBUG, "radar_lidar.localization_icp")
      << "Matched points ratio " << matched_points_ratio;
  if (matched_points_ratio > config_->min_matched_ratio) {
    // update map to robot transform
    *radar_qdata.T_r_v_loc = T_r_v_icp;
    // set success
    *radar_qdata.loc_success = true;
  } else {
    CLOG(WARNING, "radar_lidar.localization_icp")
        << "Matched points ratio " << matched_points_ratio
        << " is below the specified threshold. ICP is considered failed.";
    // update map to robot transform to be the projected one
    *radar_qdata.T_r_v_loc = T_r_v;
    // set success
    *radar_qdata.loc_success = false;
  }
}

}  // namespace radar_lidar
}  // namespace vtr