#include <vtr_tactic/modules/ros_module_factory.hpp>

namespace vtr {
namespace tactic {

ROSModuleFactory::ModulePtr ROSModuleFactory::make(
    const std::string &param_prefix) const {
  std::string param_name{param_prefix + "." + type_field_};
  auto type_str = node_->declare_parameter<std::string>(param_name, "");
  if (type_str.empty()) {
    auto msg = "No field: '" + param_name + "'";
    LOG(ERROR) << msg;
    throw std::runtime_error(msg);
  }
  auto module = ModuleFactory::make(type_str);

  /// \todo create configFromROS method for these modules
  if (isType<stereo::KeyframeOptimizationModule>(type_str))
    configureKeyframeOptimization(module, param_prefix);
  else if (isType<stereo::StereoWindowOptimizationModule>(type_str))
    configureStereoWindowOptimization(module, param_prefix);
  else
    module->configFromROS(node_, param_prefix);

  return module;
}

void ROSModuleFactory::configureKeyframeOptimization(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::KeyframeOptimizationModule::Config>();
  // clang-format off
  // Base Config
  auto base_config = std::dynamic_pointer_cast<stereo::SteamModule::Config>(config);
  configureSteam(base_config, param_prefix);
  config->pose_prior_enable = node_->declare_parameter<decltype(config->pose_prior_enable)>(param_prefix + ".pose_prior_enable", config->pose_prior_enable);
  config->depth_prior_enable = node_->declare_parameter<decltype(config->depth_prior_enable)>(param_prefix + ".depth_prior_enable", config->depth_prior_enable);
  config->depth_prior_weight = node_->declare_parameter<decltype(config->depth_prior_weight)>(param_prefix + ".depth_prior_weight", config->depth_prior_weight);
  /// config->max_point_depth = node_->declare_parameter<decltype(config->max_point_depth)>(param_prefix + ".max_point_depth", config->max_point_depth);
  config->use_migrated_points = node_->declare_parameter<decltype(config->use_migrated_points)>(param_prefix + ".use_migrated_points", config->use_migrated_points);
  // clang-format on
  std::dynamic_pointer_cast<stereo::KeyframeOptimizationModule>(module)
      ->setConfig(config);
}

void ROSModuleFactory::configureStereoWindowOptimization(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config =
      std::make_shared<stereo::StereoWindowOptimizationModule::Config>();
  // clang-format off
  // Base Config
  auto base_config = std::dynamic_pointer_cast<stereo::SteamModule::Config>(config);
  configureSteam(base_config, param_prefix);
  config->depth_prior_enable = node_->declare_parameter<decltype(config->depth_prior_enable)>(param_prefix + ".depth_prior_enable", config->depth_prior_enable);
  config->depth_prior_weight = node_->declare_parameter<decltype(config->depth_prior_weight)>(param_prefix + ".depth_prior_weight", config->depth_prior_weight);
  // config->max_point_depth = node_->declare_parameter<decltype(config->max_point_depth)>(param_prefix + ".max_point_depth", config->max_point_depth);
  // clang-format on
  std::dynamic_pointer_cast<stereo::StereoWindowOptimizationModule>(module)
      ->setConfig(config);
}

void ROSModuleFactory::configureSteam(
    std::shared_ptr<stereo::SteamModule::Config> &config,
    const std::string &param_prefix) const {
  // clang-format off
  config->solver_type = node_->declare_parameter<std::string>(param_prefix + ".solver_type", config->solver_type);
  config->loss_function = node_->declare_parameter<std::string>(param_prefix + ".loss_function", config->loss_function);
  config->verbose = node_->declare_parameter<bool>(param_prefix + ".verbose", config->verbose);
  config->use_T_q_m_prior = node_->declare_parameter<bool>(param_prefix + ".use_T_q_m_prior", config->use_T_q_m_prior);

  config->iterations = node_->declare_parameter<int>(param_prefix + ".iterations", config->iterations);
  config->absoluteCostThreshold = node_->declare_parameter<double>(param_prefix + ".absoluteCostThreshold", config->absoluteCostThreshold);
  config->absoluteCostChangeThreshold = node_->declare_parameter<double>(param_prefix + ".absoluteCostChangeThreshold", config->absoluteCostChangeThreshold);
  config->relativeCostChangeThreshold = node_->declare_parameter<double>(param_prefix + ".relativeCostChangeThreshold", config->relativeCostChangeThreshold);

  config->ratioThresholdShrink = node_->declare_parameter<double>(param_prefix + ".ratioThresholdShrink", config->ratioThresholdShrink);
  config->ratioThresholdGrow = node_->declare_parameter<double>(param_prefix + ".ratioThresholdGrow", config->ratioThresholdGrow);
  config->shrinkCoeff = node_->declare_parameter<double>(param_prefix + ".shrinkCoeff", config->shrinkCoeff);
  config->growCoeff = node_->declare_parameter<double>(param_prefix + ".growCoeff", config->growCoeff);
  config->maxShrinkSteps = node_->declare_parameter<int>(param_prefix + ".maxShrinkSteps", config->maxShrinkSteps);
  config->backtrackMultiplier = node_->declare_parameter<double>(param_prefix + ".backtrackMultiplier", config->backtrackMultiplier);
  config->maxBacktrackSteps = node_->declare_parameter<int>(param_prefix + ".maxBacktrackSteps", config->maxBacktrackSteps);

  // validity checking
  config->perform_planarity_check = node_->declare_parameter<bool>(param_prefix + ".perform_planarity_check", config->perform_planarity_check);
  config->plane_distance = node_->declare_parameter<double>(param_prefix + ".plane_distance", config->plane_distance);
  config->min_point_depth = node_->declare_parameter<double>(param_prefix + ".min_point_depth", config->min_point_depth);
  config->max_point_depth = node_->declare_parameter<double>(param_prefix + ".max_point_depth", config->max_point_depth);

  // trajectory stuff.
  config->save_trajectory = node_->declare_parameter<bool>(param_prefix + ".save_trajectory", config->save_trajectory);
  config->trajectory_smoothing = node_->declare_parameter<bool>(param_prefix + ".trajectory_smoothing", config->trajectory_smoothing);
  config->lin_acc_std_dev_x = node_->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_x", config->lin_acc_std_dev_x);
  config->lin_acc_std_dev_y = node_->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_y", config->lin_acc_std_dev_y);
  config->lin_acc_std_dev_z = node_->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_z", config->lin_acc_std_dev_z);
  config->ang_acc_std_dev_x = node_->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_x", config->ang_acc_std_dev_x);
  config->ang_acc_std_dev_y = node_->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_y", config->ang_acc_std_dev_y);
  config->ang_acc_std_dev_z = node_->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_z", config->ang_acc_std_dev_z);
  config->disable_solver = node_->declare_parameter<bool>(param_prefix + ".disable_solver", config->disable_solver);
  // velocity prior
  config->velocity_prior = node_->declare_parameter<bool>(param_prefix + ".velocity_prior", config->velocity_prior);
  config->lin_vel_mean_x = node_->declare_parameter<double>(param_prefix + ".lin_vel_mean_x", config->lin_vel_mean_x);
  config->lin_vel_mean_y = node_->declare_parameter<double>(param_prefix + ".lin_vel_mean_y", config->lin_vel_mean_y);
  config->lin_vel_mean_z = node_->declare_parameter<double>(param_prefix + ".lin_vel_mean_z", config->lin_vel_mean_z);
  config->ang_vel_mean_x = node_->declare_parameter<double>(param_prefix + ".ang_vel_mean_x", config->ang_vel_mean_x);
  config->ang_vel_mean_y = node_->declare_parameter<double>(param_prefix + ".ang_vel_mean_y", config->ang_vel_mean_y);
  config->ang_vel_mean_z = node_->declare_parameter<double>(param_prefix + ".ang_vel_mean_z", config->ang_vel_mean_z);

  config->lin_vel_std_dev_x = node_->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_x", config->lin_vel_std_dev_x);
  config->lin_vel_std_dev_y = node_->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_y", config->lin_vel_std_dev_y);
  config->lin_vel_std_dev_z = node_->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_z", config->lin_vel_std_dev_z);
  config->ang_vel_std_dev_x = node_->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_x", config->ang_vel_std_dev_x);
  config->ang_vel_std_dev_y = node_->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_y", config->ang_vel_std_dev_y);
  config->ang_vel_std_dev_z = node_->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_z", config->ang_vel_std_dev_z);
  // clang-format on
}

}  // namespace tactic
}  // namespace vtr