#include <vtr_tactic/modules/lidar/icp_module.hpp>

namespace vtr {
namespace tactic {
namespace lidar {

namespace {
bool checkDiagonal(Eigen::Array<double, 1, 6> &diag) {
  for (int idx = 0; idx < 6; ++idx)
    if (diag(idx) <= 0) return false;

  return true;
}
}  // namespace

void ICPModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                              const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->source = node->declare_parameter<std::string>(param_prefix + ".source", config_->source);
  config_->use_pose_prior = node->declare_parameter<bool>(param_prefix + ".use_pose_prior", config_->use_pose_prior);
  config_->min_matched_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_ratio", config_->min_matched_ratio);

  // trajectory smoothing
  config_->trajectory_smoothing = node->declare_parameter<bool>(param_prefix + ".trajectory_smoothing", config_->trajectory_smoothing);
  if (config_->source != "live" && config_->trajectory_smoothing) {
    std::string err{"Cannot apply trajectory smoothing when source is not live."};
    LOG(ERROR) << err;
    throw std::runtime_error{err};
  }
  config_->use_constant_acc = node->declare_parameter<bool>(param_prefix + ".use_constant_acc", config_->use_constant_acc);
  config_->lin_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_x", config_->lin_acc_std_dev_x);
  config_->lin_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_y", config_->lin_acc_std_dev_y);
  config_->lin_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_z", config_->lin_acc_std_dev_z);
  config_->ang_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_x", config_->ang_acc_std_dev_x);
  config_->ang_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_y", config_->ang_acc_std_dev_y);
  config_->ang_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_z", config_->ang_acc_std_dev_z);

  // icp params
  config_->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config_->num_threads);
#ifdef DETERMINISTIC_VTR
  LOG_IF(config_->num_threads != 1, WARNING) << "ICP number of threads set to 1 in deterministic mode.";
  config_->num_threads = 1;
#endif
  config_->n_samples = node->declare_parameter<int>(param_prefix + ".n_samples", config_->n_samples);
  config_->max_pairing_dist = node->declare_parameter<float>(param_prefix + ".max_pairing_dist", config_->max_pairing_dist);
  config_->max_planar_dist = node->declare_parameter<float>(param_prefix + ".max_planar_dist", config_->max_planar_dist);
  config_->max_iter = node->declare_parameter<int>(param_prefix + ".max_iter", config_->max_iter);
  config_->avg_steps = node->declare_parameter<int>(param_prefix + ".avg_steps", config_->avg_steps);
  config_->rot_diff_thresh = node->declare_parameter<float>(param_prefix + ".rotDiffThresh", config_->rot_diff_thresh);
  config_->trans_diff_thresh = node->declare_parameter<float>(param_prefix + ".transDiffThresh", config_->trans_diff_thresh);

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

void ICPModule::runImpl(QueryCache &qdata, MapCache &,
                        const Graph::ConstPtr &graph) {
  if (config_->source == "live") {
    LOG(DEBUG) << "Adding undistorted point cloud";
    qdata.undistorted_pointcloud.fallback(*qdata.preprocessed_pointcloud);
    qdata.undistorted_normals.fallback(*qdata.normals);
  }

  if (config_->source == "live" && !qdata.current_map_odo) {
    LOG(INFO) << "First keyframe, simply return.";
    return;
  }

  // clang-format off
  // Inputs
  const auto &query_times = *qdata.preprocessed_pointcloud_time;
  const auto &query_points = config_->source == "live" ? *qdata.preprocessed_pointcloud: *qdata.undistorted_pointcloud;
  const auto &query_normals = config_->source == "live" ? *qdata.normals: *qdata.undistorted_normals;
  const auto &query_weights = *qdata.icp_scores;
  const size_t query_num_pts = query_points.size();
  const auto &T_s_r = *qdata.T_s_r;
  const auto T_m_pm = config_->source == "live" ? *qdata.current_map_odo_T_v_m : EdgeTransform(true);
  auto &map = config_->source == "live" ? *qdata.current_map_odo : *qdata.current_map_loc;
  const auto &map_weights = map.scores;
  // Outputs
  auto &T_r_m = config_->source == "live" ? *qdata.T_r_m_odo : *qdata.T_r_m_loc;
  auto &success = config_->source == "live" ? *qdata.odo_success: *qdata.loc_success;
  auto &undistorted_pointcloud = *qdata.undistorted_pointcloud;
  auto &undistorted_normals = *qdata.undistorted_normals;
  // clang-format on

  /// Parameters
  size_t first_steps = 3;
  size_t max_it = config_->max_iter;
  size_t num_samples = config_->n_samples;
  float max_pair_d2 = config_->max_pairing_dist * config_->max_pairing_dist;
  float max_planar_d = config_->max_planar_dist;
  nanoflann::SearchParams search_params;  // kd-tree search parameters

  /// Create and add the T_robot_map variable, here map is in vertex frame.
  const auto T_r_m_var =
      boost::make_shared<steam::se3::TransformStateVar>(T_r_m);

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
    LOG(DEBUG) << "Adding prior cost term: "
               << prior_cost_terms->numCostTerms();
  }
  /// \note has to be for odometry not localization
  std::map<unsigned int, steam::StateVariableBase::Ptr> traj_state_vars;
  if (config_->trajectory_smoothing) {
    computeTrajectory(qdata, graph, T_r_m_eval, traj_state_vars,
                      prior_cost_terms);
    LOG(DEBUG) << "Number of trajectory cost terms: "
               << prior_cost_terms->numCostTerms();
    LOG(DEBUG) << "Number of state variables: " << traj_state_vars.size();
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
  bool stop_cond = false;

  std::vector<clock_t> timer(6);
  std::vector<std::string> clock_str;
  clock_str.push_back("Random_Sample ... ");
  clock_str.push_back("KNN_search ...... ");
  clock_str.push_back("Optimization .... ");
  clock_str.push_back("Regularization .. ");
  clock_str.push_back("Result .......... ");

  for (size_t step = 0;; step++) {
    /// Points Association
    // Pick random queries (use unordered set to ensure uniqueness)
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

    timer[1] = std::clock();

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

    timer[2] = std::clock();

    /// Filtering based on distances metrics
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

    timer[3] = std::clock();

    /// Point to plane optimization
    // shared loss function
    steam::L2LossFunc::Ptr loss_func(new steam::L2LossFunc());

    // cost terms and noise model
    steam::ParallelizedCostTermCollection::Ptr cost_terms(
        new steam::ParallelizedCostTermCollection());
#pragma omp parallel for schedule(dynamic, 10) num_threads(8)
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
      if (stop_cond && config_->trajectory_smoothing) {
        const auto &qry_time = query_times[ind.first];
        const auto T_rm_intp_eval =
            trajectory_->getInterpPoseEval(steam::Time(qry_time));
        const auto T_mq_intp_eval = steam::se3::inverse(steam::se3::compose(
            T_s_r_eval, steam::se3::compose(T_rm_intp_eval, T_m_pm_eval)));
        error_func.reset(
            new steam::PointToPointErrorEval2(T_mq_intp_eval, ref_pt, qry_pt));
      } else {
        error_func.reset(
            new steam::PointToPointErrorEval2(T_mq_eval, ref_pt, qry_pt));
      }

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
    if (stop_cond) {
      // add prior costs
      if (config_->trajectory_smoothing || config_->use_pose_prior)
        problem.addCostTerm(prior_cost_terms);

      if (config_->trajectory_smoothing) {
        for (const auto &var : traj_state_vars)
          problem.addStateVariable(var.second);
      }
    }

    using SolverType = steam::VanillaGaussNewtonSolver;
    SolverType::Params params;
    params.verbose = false;
    params.maxIterations = 1;

    // Make solver
    SolverType solver(&problem, params);

    // Optimize
    solver.optimize();

    timer[4] = std::clock();

    /// Alignment
    if (stop_cond && config_->trajectory_smoothing) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(8)
      for (unsigned i = 0; i < query_times.size(); i++) {
        const auto &qry_time = query_times[i];
        const auto T_rm_intp_eval =
            trajectory_->getInterpPoseEval(steam::Time(qry_time));
        const auto T_mq_intp_eval = steam::se3::inverse(steam::se3::compose(
            T_s_r_eval, steam::se3::compose(T_rm_intp_eval, T_m_pm_eval)));
        const auto T_mq = T_mq_intp_eval->evaluate().matrix();
        Eigen::Matrix3f C_mq = T_mq.block<3, 3>(0, 0).cast<float>();
        Eigen::Vector3f r_m_qm = T_mq.block<3, 1>(0, 3).cast<float>();
        aligned_mat.block<3, 1>(0, i) =
            C_mq * query_mat.block<3, 1>(0, i) + r_m_qm;
        aligned_norms_mat.block<3, 1>(0, i) =
            C_mq * query_norms_mat.block<3, 1>(0, i);
      }
    } else {
      const auto T_mq = T_mq_eval->evaluate().matrix();
      Eigen::Matrix3f C_mq = T_mq.block<3, 3>(0, 0).cast<float>();
      Eigen::Vector3f r_m_qm = T_mq.block<3, 1>(0, 3).cast<float>();
      aligned_mat = (C_mq * query_mat).colwise() + r_m_qm;
      aligned_norms_mat = C_mq * query_norms_mat;
    }

    // Update all result matrices
    const auto T_mq = T_mq_eval->evaluate().matrix();
    if (step == 0)
      all_tfs = Eigen::MatrixXd(T_mq);
    else {
      Eigen::MatrixXd temp(all_tfs.rows() + 4, 4);
      temp.topRows(all_tfs.rows()) = all_tfs;
      temp.bottomRows(4) = Eigen::MatrixXd(T_mq);
      all_tfs = temp;
    }

    timer[5] = std::clock();

    /// Check convergence
    // Update variations
    if (!stop_cond && step > 0) {
      float avg_tot = step == 1 ? 1.0 : (float)config_->avg_steps;

      // Get last transformation variations
      Eigen::Matrix3d R2 = all_tfs.block(all_tfs.rows() - 4, 0, 3, 3);
      Eigen::Matrix3d R1 = all_tfs.block(all_tfs.rows() - 8, 0, 3, 3);
      Eigen::Vector3d T2 = all_tfs.block(all_tfs.rows() - 4, 3, 3, 1);
      Eigen::Vector3d T1 = all_tfs.block(all_tfs.rows() - 8, 3, 3, 1);
      R1 = R2 * R1.transpose();
      T1 = T2 - T1;
      float dR_b = acos((R1.trace() - 1) / 2);
      float dT_b = T1.norm();
      mean_dT += (dT_b - mean_dT) / avg_tot;
      mean_dR += (dR_b - mean_dR) / avg_tot;
    }

    // Stop condition
    if (!stop_cond && step > config_->avg_steps) {
      if (step >= max_it - 1 || (mean_dT < config_->trans_diff_thresh &&
                                 mean_dR < config_->rot_diff_thresh)) {
        // Do not stop right away. Have a last few averaging steps
        stop_cond = true;
        max_it = step + config_->avg_steps;

        // For these last steps,
        // increase number of samples
        num_samples = 5000;
        // reduce the max distance
        max_planar_d = 0.1;
      }
    }

    // Last step
    if (step >= max_it - 1) {
      // result
      T_r_m_icp = EdgeTransform(T_r_m_var->getValue(),
                                solver.queryCovariance(T_r_m_var->getKey()));
      matched_points_ratio =
          (float)filtered_sample_inds.size() / (float)num_samples;
      if (!stop_cond) {
        LOG(WARNING) << "ICP did not converge matched_points_ratio set to 0.";
        // matched_points_ratio = 0;
      }
      break;
    }
  }

  /// Whether ICP is successful
  qdata.matched_points_ratio.fallback(matched_points_ratio);
  if (matched_points_ratio > config_->min_matched_ratio) {
    // store undistorted pointcloud
    const auto T_qm = T_mq_eval->evaluate().matrix().inverse();
    Eigen::Matrix3f C_qm = T_qm.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f r_q_mq = T_qm.block<3, 1>(0, 3).cast<float>();
    aligned_mat = (C_qm * aligned_mat).colwise() + r_q_mq;
    aligned_norms_mat = C_qm * aligned_norms_mat;
    undistorted_pointcloud = aligned_points;
    undistorted_normals = aligned_normals;
    // update map to robot transform
    T_r_m = T_r_m_icp;
    // set success
    success = true;
    if (config_->trajectory_smoothing) qdata.trajectory = trajectory_;
  } else {
    LOG(ERROR) << "Matched points ratio " << matched_points_ratio
               << " is below the threshold. ICP is considered failed.";
  }
}

void ICPModule::computeTrajectory(
    QueryCache &qdata, const Graph::ConstPtr &graph,
    const steam::se3::TransformEvaluator::Ptr &T_r_m_eval,
    std::map<unsigned int, steam::StateVariableBase::Ptr> &state_vars,
    const steam::ParallelizedCostTermCollection::Ptr &prior_cost_terms) {
  // reset the trajectory
  if (config_->use_constant_acc)
    trajectory_.reset(new steam::se3::SteamCATrajInterface(
        smoothing_factor_information_, true));
  else
    trajectory_.reset(new steam::se3::SteamTrajInterface(
        smoothing_factor_information_, true));

  // get the live vertex
  const auto live_vertex = graph->at(*qdata.live_id);
  LOG(DEBUG) << "Looking at live id: " << *qdata.live_id;

  /// Set up a search for the previous keyframes in the graph
  TemporalEvaluator::Ptr tempeval(new TemporalEvaluator());
  tempeval->setGraph((void *)graph.get());
  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  using DirectionEvaluator =
      pose_graph::eval::Mask::DirectionFromVertexDirect<Graph>;
  auto direval = std::make_shared<DirectionEvaluator>(*qdata.live_id, true);
  direval->setGraph((void *)graph.get());
  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);
  evaluator->setGraph((void *)graph.get());

  // look back number of vertices
  int temporal_depth = 3;

  std::map<VertexId, steam::VectorSpaceStateVar::Ptr> velocity_map;
  std::map<VertexId, steam::VectorSpaceStateVar::Ptr> acceleration_map;

  // perform the search and automatically step back one
  auto itr = graph->beginDfs(*qdata.live_id, temporal_depth, evaluator);
  itr++;

  // initialize the compunded transform
  EdgeTransform T_p_l = EdgeTransform(true);  // T_previous_live
  // initialize the timestamp that will be used as
  auto next_stamp = live_vertex->keyFrameTime();

  // Trajectory is of the following form, where the origin = 0 is at m
  // which is the most recent keyframe in the graph.
  //                      local
  //       T_1_0     T_2_1     T_l_2      T_q_l
  //    0---------1---------2---------l----------q
  //  T_0_l     T_1_l     T_2_l       0        T_q_1
  //                      global
  // loop through all the found vertices
  for (; itr != graph->end(); ++itr) {
    // get the stamp of the vertex we're looking at
    auto prev_vertex = graph->at(itr->to());
    auto &prev_stamp = prev_vertex->keyFrameTime();

    // get the transform and compund it
    const auto &T_p_pp = itr->e()->T();
    T_p_l = T_p_pp.inverse() * T_p_l;

    // set up a locked global pose for this vertex, with an tf evaluator
    // Note: normally steam would have states T_a_0, T_b_0, ..., where 'a' and
    // 'b' are always sequential in time. So in our case, since our locked '0'
    // frame is in the future, 'a' is actually further from '0' than 'b'.
    const auto prev_pose =
        boost::make_shared<steam::se3::TransformStateVar>(T_p_l);
    prev_pose->setLock(true);
    const auto tf_prev =
        boost::make_shared<steam::se3::TransformStateEvaluator>(prev_pose);

    // time difference between next and previous
    int64_t next_prev_dt =
        next_stamp.nanoseconds_since_epoch - prev_stamp.nanoseconds_since_epoch;

    // generate a velocity estimate
    // The velocity is in the body frame, helping you get from 'a' to 'b'.
    // This part can ignore the fact that the transforms above are weird
    // (new to old instead of old to new), and keep doing vel_b_a.
    // we use p_pp instead of p_pm1 for convenience
    Eigen::Matrix<double, 6, 1> prev_velocity =
        T_p_pp.vec() / (next_prev_dt / 1e9);

    auto prev_frame_velocity =
        boost::make_shared<steam::VectorSpaceStateVar>(prev_velocity);

    velocity_map.insert({prev_vertex->id(), prev_frame_velocity});

    // generate an acceleration map
    Eigen::Matrix<double, 6, 1> prev_acceleration =
        Eigen::Matrix<double, 6, 1>::Zero();

    auto prev_frame_acceleration =
        boost::make_shared<steam::VectorSpaceStateVar>(prev_acceleration);

    acceleration_map.insert({prev_vertex->id(), prev_frame_acceleration});

    // make a steam time from the timstamp
    steam::Time prev_time(
        static_cast<int64_t>(prev_stamp.nanoseconds_since_epoch));

    // Add the poses to the trajectory
    trajectory_->add(prev_time, tf_prev, prev_frame_velocity,
                     prev_frame_acceleration);
    next_stamp = prev_stamp;

    LOG(DEBUG) << "Looking at previous vertex id: " << prev_vertex->id()
               << ", with T_previous_live being:\n"
               << T_p_l << ", and velocity being:" << prev_velocity.transpose()
               << ", time difference: " << next_prev_dt / 1e9;
  }
  // lock the velocity at the begining of the trajectory
  if (!velocity_map.empty()) {
    auto itr = velocity_map.begin();
    itr->second->setLock(true);
    LOG(DEBUG) << "Locking the first velocity corresponding to vertex id: "
               << itr->first;
    itr++;
    for (; itr != velocity_map.end(); itr++) {
      LOG(DEBUG) << "Adding velocity corresponding to vertex id: "
                 << itr->first;
      state_vars[itr->second->getKey().getID()] = itr->second;
    }
  }
  // lock the acceleration at the begining of the trajectory
  if (config_->use_constant_acc && !acceleration_map.empty()) {
    auto itr = acceleration_map.begin();
    // itr->second->setLock(true);
    // LOG(DEBUG) << "Locking the first acc corresponding to vertex id: "
    //            << itr->first;
    // itr++;
    for (; itr != acceleration_map.end(); itr++) {
      LOG(DEBUG) << "Adding acceleration corresponding to vertex id: "
                 << itr->first;
      state_vars[itr->second->getKey().getID()] = itr->second;
    }
  }

  // get the stamps
  const auto &query_stamp = *qdata.stamp;
  const auto &live_stamp = live_vertex->keyFrameTime();
  // time difference between query and live frame
  int64_t query_live_dt =
      query_stamp.nanoseconds_since_epoch - live_stamp.nanoseconds_since_epoch;
  // generate velocity estimate
  Eigen::Matrix<double, 6, 1> query_velocity =
      (*qdata.T_r_m_odo).vec() / (query_live_dt / 1e9);
  LOG(DEBUG) << "Adding velocity of query frame and live vertex id: "
             << *qdata.live_id << ": " << query_velocity.transpose()
             << ", time difference: " << query_live_dt / 1e9;
  // generate acceleration estimate
  Eigen::Matrix<double, 6, 1> query_acceleration =
      Eigen::Matrix<double, 6, 1>::Zero();

  steam::Time live_time(
      static_cast<int64_t>(live_stamp.nanoseconds_since_epoch));

  // Add the poses to the trajectory
  const auto live_pose = boost::make_shared<steam::se3::TransformStateVar>(
      lgmath::se3::Transformation());
  live_pose->setLock(true);  // lock the 'origin' pose
  const auto tf_live =
      boost::make_shared<steam::se3::TransformStateEvaluator>(live_pose);

  steam::VectorSpaceStateVar::Ptr live_frame_velocity(
      new steam::VectorSpaceStateVar(query_velocity));
  velocity_map.insert({*qdata.live_id, live_frame_velocity});
  state_vars[live_frame_velocity->getKey().getID()] = live_frame_velocity;

  steam::VectorSpaceStateVar::Ptr live_frame_acceleration(
      new steam::VectorSpaceStateVar(query_acceleration));
  acceleration_map.insert({*qdata.live_id, live_frame_acceleration});
  if (config_->use_constant_acc) {
    state_vars[live_frame_acceleration->getKey().getID()] =
        live_frame_acceleration;
  }

  trajectory_->add(live_time, tf_live, live_frame_velocity,
                   live_frame_acceleration);

  steam::Time query_time(
      static_cast<int64_t>(query_stamp.nanoseconds_since_epoch));

  steam::VectorSpaceStateVar::Ptr query_frame_velocity(
      new steam::VectorSpaceStateVar(query_velocity));
  velocity_map.insert({VertexId::Invalid(), query_frame_velocity});
  state_vars[query_frame_velocity->getKey().getID()] = query_frame_velocity;

  steam::VectorSpaceStateVar::Ptr query_frame_acceleration(
      new steam::VectorSpaceStateVar(query_acceleration));
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

void ICPModule::addPosePrior(
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
}  // namespace tactic
}  // namespace vtr