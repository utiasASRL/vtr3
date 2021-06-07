#include <vtr_tactic/modules/lidar/icp_module.hpp>

namespace vtr {
namespace tactic {
namespace lidar {

void ICPModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                              const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->source = node->declare_parameter<std::string>(param_prefix + ".source", config_->source);
  config_->use_prior = node->declare_parameter<bool>(param_prefix + ".use_prior", config_->use_prior);
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
}

void ICPModule::runImpl(QueryCache &qdata, MapCache &,
                        const Graph::ConstPtr &) {
  if (config_->source == "live" && !qdata.current_map_odo) {
    LOG(INFO) << "First keyframe, simply return.";
    return;
  }

  // Get input data
  auto &points = *qdata.preprocessed_pointcloud;
  auto &icp_scores = *qdata.icp_scores;
  auto &T_s_r = *qdata.T_s_r;
  /// check source in live and map
  auto &map = config_->source == "live"
                  ? (qdata.new_map ? *qdata.new_map : *qdata.current_map_odo)
                  : *qdata.current_map_loc;
  // Output
  auto &T_r_m = config_->source == "live" ? *qdata.T_r_m_odo : *qdata.T_r_m_loc;

  // Create a copy of T_r_m as prior
  auto T_r_m_prior = T_r_m;

  // Create result containers
  vtr::lidar::ICPResults icp_results;
  auto T_m_s = T_r_m.inverse() * T_s_r.inverse();
  config_->init_transform = T_m_s.matrix();
  vtr::lidar::pointToMapICP(points, icp_scores, map, *config_, icp_results);

  lgmath::se3::TransformationWithCovariance hat_T_m_s(icp_results.transform,
                                                      icp_results.covariance);
  auto T_r_m_hat = (hat_T_m_s * T_s_r).inverse();

  /// Whether ICP is successful
  qdata.matched_points_ratio.fallback(icp_results.matched_points_ratio);
  if (icp_results.matched_points_ratio < 0.4) {
    LOG(ERROR) << "Matched points ratio " << icp_results.matched_points_ratio
               << " is below the threshold. ICP is considered failed.";
    return;
  }

  *qdata.loc_success = true;

  if (!config_->use_prior) {
    T_r_m = T_r_m_hat;
    return;
  }

  LOG(INFO) << "Prior estimate is:\n" << T_r_m_prior;
  LOG(INFO) << "MLE estimate is:\n" << T_r_m_hat;

  using namespace steam;
  /// Fuse poses using steam
  se3::TransformStateVar::Ptr T_r_m_var(new se3::TransformStateVar());
  se3::TransformStateEvaluator::ConstPtr T_r_m_eval =
      se3::TransformStateEvaluator::MakeShared(T_r_m_var);
  ParallelizedCostTermCollection::Ptr cost_terms(
      new ParallelizedCostTermCollection());
  L2LossFunc::Ptr loss_func(new L2LossFunc());
  // prior p(x)
  StaticNoiseModel<6>::Ptr noise_model_prior(
      new StaticNoiseModel<6>(T_r_m_prior.cov()));
  TransformErrorEval::Ptr error_func_prior(
      new TransformErrorEval(T_r_m_prior, T_r_m_eval));
  WeightedLeastSqCostTerm<6, 6>::Ptr cost_prior(
      new WeightedLeastSqCostTerm<6, 6>(error_func_prior, noise_model_prior,
                                        loss_func));
  cost_terms->add(cost_prior);
  // maximum likelihood p(y|x)
  StaticNoiseModel<6>::Ptr noise_model_hat(
      new StaticNoiseModel<6>(T_r_m_hat.cov()));
  TransformErrorEval::Ptr error_func_hat(
      new TransformErrorEval(T_r_m_hat, T_r_m_eval));
  WeightedLeastSqCostTerm<6, 6>::Ptr cost_hat(new WeightedLeastSqCostTerm<6, 6>(
      error_func_hat, noise_model_hat, loss_func));
  cost_terms->add(cost_hat);

  /// Initialize problem
  OptimizationProblem problem;
  problem.addStateVariable(T_r_m_var);
  problem.addCostTerm(cost_terms);

  /// Solve
  // VanillaGaussNewtonSolver::Params params;
  // params.verbose = true;
  VanillaGaussNewtonSolver solver(&problem, *config_);
  T_r_m_var->setValue(T_r_m_prior);
  solver.optimize();

  /// Get result
  T_r_m = T_r_m_var->getValue();
  T_r_m.setCovariance(solver.queryCovariance(T_r_m_var->getKey()));

  LOG(INFO) << "Posterior estimate is:\n" << T_r_m;
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr