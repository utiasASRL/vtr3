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
  if (isType<stereo::ImageTriangulationModule>(type_str))
    configureImageTriangulation(module, param_prefix);
  else if (isType<stereo::LandmarkRecallModule>(type_str))
    configureLandmarkRecall(module, param_prefix);
  else if (isType<stereo::ASRLStereoMatcherModule>(type_str))
    configureASRLStereoMatcher(module, param_prefix);
  else if (isType<stereo::StereoRansacModule>(type_str))
    configureStereoRANSAC(module, param_prefix);
  else if (isType<stereo::KeyframeOptimizationModule>(type_str))
    configureKeyframeOptimization(module, param_prefix);
  else if (isType<stereo::SimpleVertexTestModule>(type_str))
    configureSimpleVertexTest(module, param_prefix);
  else if (isType<stereo::StereoWindowOptimizationModule>(type_str))
    configureStereoWindowOptimization(module, param_prefix);
  else if (isType<stereo::StereoWindowedRecallModule>(type_str))
    configureStereoWindowedRecallModule(module, param_prefix);
  else if (isType<stereo::SubMapExtractionModule>(type_str))
    configureSubMapExtraction(module, param_prefix);
  else if (isType<stereo::LandmarkMigrationModule>(type_str))
    configureLandmarkMigration(module, param_prefix);
  else if (isType<stereo::ExperienceTriageModule>(type_str))
    configureExperienceTriage(module, param_prefix);
  else if (isType<stereo::TodRecognitionModule>(type_str))
    configureTodRecog(module, param_prefix);
  else if (isType<stereo::MelMatcherModule>(type_str))
    configureMelMatcher(module, param_prefix);
  else
    module->configFromROS(node_, param_prefix);

  return module;
}


void ROSModuleFactory::configureImageTriangulation(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::ImageTriangulationModule::Config>();
  // clang-format off
  config->visualize_features = node_->declare_parameter<decltype(config->visualize_features)>(param_prefix + ".visualize_features", config->visualize_features);
  config->visualize_stereo_features = node_->declare_parameter<decltype(config->visualize_stereo_features)>(param_prefix + ".visualize_stereo_features", config->visualize_stereo_features);
  config->min_triangulation_depth = node_->declare_parameter<decltype(config->min_triangulation_depth)>(param_prefix + ".min_triangulation_depth", config->min_triangulation_depth);
  config->max_triangulation_depth = node_->declare_parameter<decltype(config->max_triangulation_depth)>(param_prefix + ".max_triangulation_depth", config->max_triangulation_depth);
  // clang-format on
  std::dynamic_pointer_cast<stereo::ImageTriangulationModule>(module)
      ->setConfig(config);
}

void ROSModuleFactory::configureLandmarkRecall(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::LandmarkRecallModule::Config>();
  // clang-format off
  config->landmark_source = node_->declare_parameter<decltype(config->landmark_source)>(param_prefix + ".landmark_source", config->landmark_source);
  config->landmark_matches = node_->declare_parameter<decltype(config->landmark_matches)>(param_prefix + ".landmark_matches", config->landmark_matches);
  // clang-format on
  std::dynamic_pointer_cast<stereo::LandmarkRecallModule>(module)->setConfig(
      config);
}

void ROSModuleFactory::configureASRLStereoMatcher(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::ASRLStereoMatcherModule::Config>();
  // clang-format off
  config->check_laplacian_bit = node_->declare_parameter<decltype(config->check_laplacian_bit)>(param_prefix + ".check_laplacian_bit", config->check_laplacian_bit);
  config->check_octave = node_->declare_parameter<decltype(config->check_octave)>(param_prefix + ".check_octave", config->check_octave);
  config->check_response = node_->declare_parameter<decltype(config->check_response)>(param_prefix + ".check_response", config->check_response);
  config->min_response_ratio = node_->declare_parameter<decltype(config->min_response_ratio)>(param_prefix + ".min_response_ratio", config->min_response_ratio);
  config->matching_pixel_thresh = node_->declare_parameter<decltype(config->matching_pixel_thresh)>(param_prefix + ".matching_pixel_thresh", config->matching_pixel_thresh);
  config->tight_matching_pixel_thresh = node_->declare_parameter<decltype(config->tight_matching_pixel_thresh)>(param_prefix + ".tight_matching_pixel_thresh", config->tight_matching_pixel_thresh);
  config->tight_matching_x_sigma = node_->declare_parameter<decltype(config->tight_matching_x_sigma)>(param_prefix + ".tight_matching_x_sigma", config->tight_matching_x_sigma);
  config->tight_matching_y_sigma = node_->declare_parameter<decltype(config->tight_matching_y_sigma)>(param_prefix + ".tight_matching_y_sigma", config->tight_matching_y_sigma);
  config->tight_matching_theta_sigma = node_->declare_parameter<decltype(config->tight_matching_theta_sigma)>(param_prefix + ".tight_matching_theta_sigma", config->tight_matching_theta_sigma);
  config->use_pixel_variance = node_->declare_parameter<decltype(config->use_pixel_variance)>(param_prefix + ".use_pixel_variance", config->use_pixel_variance);

  auto prediction_method = node_->declare_parameter<std::string>(param_prefix + ".prediction_method", "");
  if (!prediction_method.compare("se3"))
    config->prediction_method = stereo::ASRLStereoMatcherModule::se3;
  else if (!prediction_method.compare("none"))
    config->prediction_method = stereo::ASRLStereoMatcherModule::none;
  else
    config->prediction_method = stereo::ASRLStereoMatcherModule::none;

  config->max_point_depth = node_->declare_parameter<decltype(config->max_point_depth)>(param_prefix + ".max_point_depth", config->max_point_depth);
  config->descriptor_thresh = node_->declare_parameter<decltype(config->descriptor_thresh)>(param_prefix + ".descriptor_thresh", config->descriptor_thresh);
  config->parallel_threads = node_->declare_parameter<decltype(config->parallel_threads)>(param_prefix + ".parallel_threads", config->parallel_threads);
#ifdef DETERMINISTIC_VTR
  LOG_IF(config->parallel_threads != 1, WARNING) << "ASRL stereo matcher number of threads set to 1 in deterministic mode.";
  config->parallel_threads = 1;
#endif
  config->visualize_feature_matches = node_->declare_parameter<decltype(config->visualize_feature_matches)>(param_prefix + ".visualize_feature_matches", config->visualize_feature_matches);

  std::dynamic_pointer_cast<stereo::ASRLStereoMatcherModule>(module)->setConfig(config);
  // clang-format on
}

void ROSModuleFactory::configureStereoRANSAC(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::StereoRansacModule::Config>();
  // clang-format off
  // Base Config
  auto base_config = std::dynamic_pointer_cast<stereo::RansacModule::Config>(config);
  configureRANSAC(base_config, param_prefix);
  // Stereo RANSAC Config
  config->mask_depth = node_->declare_parameter<decltype(config->mask_depth)>(param_prefix + ".mask_depth", config->mask_depth);
  config->mask_depth_inlier_count = node_->declare_parameter<decltype(config->mask_depth_inlier_count)>(param_prefix + ".mask_depth_inlier_count", config->mask_depth_inlier_count);
  /// config->visualize_ransac_inliers = node_->declare_parameter<decltype(config->visualize_ransac_inliers)>(param_prefix + ".visualize_ransac_inliers", config->visualize_ransac_inliers);
  config->use_covariance = node_->declare_parameter<decltype(config->use_covariance)>(param_prefix + ".use_covariance", config->use_covariance);
  // clang-format on
  std::dynamic_pointer_cast<stereo::StereoRansacModule>(module)->setConfig(
      config);
}

void ROSModuleFactory::configureRANSAC(
    std::shared_ptr<stereo::RansacModule::Config> &config,
    const std::string &param_prefix) const {
  // clang-format off
  // Base RANSAC
  config->enable = node_->declare_parameter<decltype(config->enable)>(param_prefix + ".enable", config->enable);
  config->iterations = node_->declare_parameter<decltype(config->iterations)>(param_prefix + ".iterations", config->iterations);
  config->flavor = node_->declare_parameter<decltype(config->flavor)>(param_prefix + ".flavor", config->flavor);
  config->sigma = node_->declare_parameter<decltype(config->sigma)>(param_prefix + ".sigma", config->sigma);
  config->threshold = node_->declare_parameter<decltype(config->threshold)>(param_prefix + ".threshold", config->threshold);
  config->early_stop_ratio = node_->declare_parameter<decltype(config->early_stop_ratio)>(param_prefix + ".early_stop_ratio", config->early_stop_ratio);
  config->early_stop_min_inliers = node_->declare_parameter<decltype(config->early_stop_min_inliers)>(param_prefix + ".early_stop_min_inliers", config->early_stop_min_inliers);
  config->visualize_ransac_inliers = node_->declare_parameter<decltype(config->visualize_ransac_inliers)>(param_prefix + ".visualize_ransac_inliers", config->visualize_ransac_inliers);
  config->use_migrated_points = node_->declare_parameter<decltype(config->use_migrated_points)>(param_prefix + ".use_migrated_points", config->use_migrated_points);
  config->min_inliers = node_->declare_parameter<decltype(config->min_inliers)>(param_prefix + ".min_inliers", config->min_inliers);
  config->enable_local_opt = node_->declare_parameter<decltype(config->enable_local_opt)>(param_prefix + ".enable_local_opt", config->enable_local_opt);
  config->num_threads = node_->declare_parameter<decltype(config->num_threads)>(param_prefix + ".num_threads", config->num_threads);
#ifdef DETERMINISTIC_VTR
  LOG_IF(config->num_threads!=1, WARNING) << "RANSAC number of threads set to 1 in deterministic mode.";
  config->num_threads = 1;
#endif
  // clang-format on
  // sanity check
  if (config->num_threads < 1) config->num_threads = 1;
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
  config->solver_type = node_->declare_parameter<decltype(config->solver_type)>(param_prefix + ".solver_type", config->solver_type);
  config->loss_function = node_->declare_parameter<decltype(config->loss_function)>(param_prefix + ".loss_function", config->loss_function);
  config->verbose = node_->declare_parameter<decltype(config->verbose)>(param_prefix + ".verbose", config->verbose);
  config->use_T_q_m_prior = node_->declare_parameter<decltype(config->use_T_q_m_prior)>(param_prefix + ".use_T_q_m_prior", config->use_T_q_m_prior);

  config->iterations = node_->declare_parameter<decltype(config->iterations)>(param_prefix + ".iterations", config->iterations);
  config->absoluteCostThreshold = node_->declare_parameter<decltype(config->absoluteCostThreshold)>(param_prefix + ".absoluteCostThreshold", config->absoluteCostThreshold);
  config->absoluteCostChangeThreshold = node_->declare_parameter<decltype(config->absoluteCostChangeThreshold)>(param_prefix + ".absoluteCostChangeThreshold", config->absoluteCostChangeThreshold);
  config->relativeCostChangeThreshold = node_->declare_parameter<decltype(config->relativeCostChangeThreshold)>(param_prefix + ".relativeCostChangeThreshold", config->relativeCostChangeThreshold);

  config->ratioThresholdShrink = node_->declare_parameter<decltype(config->ratioThresholdShrink)>(param_prefix + ".ratioThresholdShrink", config->ratioThresholdShrink);
  config->ratioThresholdGrow = node_->declare_parameter<decltype(config->ratioThresholdGrow)>(param_prefix + ".ratioThresholdGrow", config->ratioThresholdGrow);
  config->shrinkCoeff = node_->declare_parameter<decltype(config->shrinkCoeff)>(param_prefix + ".shrinkCoeff", config->shrinkCoeff);
  config->growCoeff = node_->declare_parameter<decltype(config->growCoeff)>(param_prefix + ".growCoeff", config->growCoeff);
  config->maxShrinkSteps = node_->declare_parameter<decltype(config->maxShrinkSteps)>(param_prefix + ".maxShrinkSteps", config->maxShrinkSteps);
  config->backtrackMultiplier = node_->declare_parameter<decltype(config->backtrackMultiplier)>(param_prefix + ".backtrackMultiplier", config->backtrackMultiplier);
  config->maxBacktrackSteps = node_->declare_parameter<decltype(config->maxBacktrackSteps)>(param_prefix + ".maxBacktrackSteps", config->maxBacktrackSteps);

  // validity checking
  config->perform_planarity_check = node_->declare_parameter<decltype(config->perform_planarity_check)>(param_prefix + ".perform_planarity_check", config->perform_planarity_check);
  config->plane_distance = node_->declare_parameter<decltype(config->plane_distance)>(param_prefix + ".plane_distance", config->plane_distance);
  config->min_point_depth = node_->declare_parameter<decltype(config->min_point_depth)>(param_prefix + ".min_point_depth", config->min_point_depth);
  config->max_point_depth = node_->declare_parameter<decltype(config->max_point_depth)>(param_prefix + ".max_point_depth", config->max_point_depth);

  // trajectory stuff.
  config->save_trajectory = node_->declare_parameter<decltype(config->save_trajectory)>(param_prefix + ".save_trajectory", config->save_trajectory);
  config->trajectory_smoothing = node_->declare_parameter<decltype(config->trajectory_smoothing)>(param_prefix + ".trajectory_smoothing", config->trajectory_smoothing);
  config->lin_acc_std_dev_x = node_->declare_parameter<decltype(config->lin_acc_std_dev_x)>(param_prefix + ".lin_acc_std_dev_x", config->lin_acc_std_dev_x);
  config->lin_acc_std_dev_y = node_->declare_parameter<decltype(config->lin_acc_std_dev_y)>(param_prefix + ".lin_acc_std_dev_y", config->lin_acc_std_dev_y);
  config->lin_acc_std_dev_z = node_->declare_parameter<decltype(config->lin_acc_std_dev_z)>(param_prefix + ".lin_acc_std_dev_z", config->lin_acc_std_dev_z);
  config->ang_acc_std_dev_x = node_->declare_parameter<decltype(config->ang_acc_std_dev_x)>(param_prefix + ".ang_acc_std_dev_x", config->ang_acc_std_dev_x);
  config->ang_acc_std_dev_y = node_->declare_parameter<decltype(config->ang_acc_std_dev_y)>(param_prefix + ".ang_acc_std_dev_y", config->ang_acc_std_dev_y);
  config->ang_acc_std_dev_z = node_->declare_parameter<decltype(config->ang_acc_std_dev_z)>(param_prefix + ".ang_acc_std_dev_z", config->ang_acc_std_dev_z);
  config->disable_solver = node_->declare_parameter<decltype(config->disable_solver)>(param_prefix + ".disable_solver", config->disable_solver);
  // velocity prior
  config->velocity_prior = node_->declare_parameter<decltype(config->velocity_prior)>(param_prefix + ".velocity_prior", config->velocity_prior);
  config->lin_vel_mean_x = node_->declare_parameter<decltype(config->lin_vel_mean_x)>(param_prefix + ".lin_vel_mean_x", config->lin_vel_mean_x);
  config->lin_vel_mean_y = node_->declare_parameter<decltype(config->lin_vel_mean_y)>(param_prefix + ".lin_vel_mean_y", config->lin_vel_mean_y);
  config->lin_vel_mean_z = node_->declare_parameter<decltype(config->lin_vel_mean_z)>(param_prefix + ".lin_vel_mean_z", config->lin_vel_mean_z);
  config->ang_vel_mean_x = node_->declare_parameter<decltype(config->ang_vel_mean_x)>(param_prefix + ".ang_vel_mean_x", config->ang_vel_mean_x);
  config->ang_vel_mean_y = node_->declare_parameter<decltype(config->ang_vel_mean_y)>(param_prefix + ".ang_vel_mean_y", config->ang_vel_mean_y);
  config->ang_vel_mean_z = node_->declare_parameter<decltype(config->ang_vel_mean_z)>(param_prefix + ".ang_vel_mean_z", config->ang_vel_mean_z);

  config->lin_vel_std_dev_x = node_->declare_parameter<decltype(config->lin_vel_std_dev_x)>(param_prefix + ".lin_vel_std_dev_x", config->lin_vel_std_dev_x);
  config->lin_vel_std_dev_y = node_->declare_parameter<decltype(config->lin_vel_std_dev_y)>(param_prefix + ".lin_vel_std_dev_y", config->lin_vel_std_dev_y);
  config->lin_vel_std_dev_z = node_->declare_parameter<decltype(config->lin_vel_std_dev_z)>(param_prefix + ".lin_vel_std_dev_z", config->lin_vel_std_dev_z);
  config->ang_vel_std_dev_x = node_->declare_parameter<decltype(config->ang_vel_std_dev_x)>(param_prefix + ".ang_vel_std_dev_x", config->ang_vel_std_dev_x);
  config->ang_vel_std_dev_y = node_->declare_parameter<decltype(config->ang_vel_std_dev_y)>(param_prefix + ".ang_vel_std_dev_y", config->ang_vel_std_dev_y);
  config->ang_vel_std_dev_z = node_->declare_parameter<decltype(config->ang_vel_std_dev_z)>(param_prefix + ".ang_vel_std_dev_z", config->ang_vel_std_dev_z);
  // clang-format on
}

void ROSModuleFactory::configureSimpleVertexTest(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::SimpleVertexTestModule::Config>();
  // clang-format off
  config->min_creation_distance = node_->declare_parameter<decltype(config->min_creation_distance)>(param_prefix + ".min_creation_distance", config->min_creation_distance);
  config->max_creation_distance = node_->declare_parameter<decltype(config->max_creation_distance)>(param_prefix + ".max_creation_distance", config->max_creation_distance);
  config->min_distance = node_->declare_parameter<decltype(config->min_distance)>(param_prefix + ".min_distance", config->min_distance);
  config->rotation_threshold_min = node_->declare_parameter<decltype(config->rotation_threshold_min)>(param_prefix + ".rotation_threshold_min", config->rotation_threshold_min);
  config->rotation_threshold_max = node_->declare_parameter<decltype(config->rotation_threshold_max)>(param_prefix + ".rotation_threshold_max", config->rotation_threshold_max);
  config->match_threshold_min_count = node_->declare_parameter<decltype(config->match_threshold_min_count)>(param_prefix + ".match_threshold_min_count", config->match_threshold_min_count);
  config->match_threshold_fail_count = node_->declare_parameter<decltype(config->match_threshold_fail_count)>(param_prefix + ".match_threshold_fail_count", config->match_threshold_fail_count);
  // clang-format on
  std::dynamic_pointer_cast<stereo::SimpleVertexTestModule>(module)->setConfig(
      config);
}

void ROSModuleFactory::configureStereoWindowedRecallModule(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::StereoWindowedRecallModule::Config>();
  // clang-format off
  config->window_size = node_->declare_parameter<decltype(config->window_size)>(param_prefix + ".window_size", config->window_size);
  // clang-format on
  std::dynamic_pointer_cast<stereo::StereoWindowedRecallModule>(module)
      ->setConfig(config);
}

void ROSModuleFactory::configureSubMapExtraction(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::SubMapExtractionModule::Config>();
  // clang-format off
  config->sigma_scale = node_->declare_parameter<decltype(config->sigma_scale)>(param_prefix + ".sigma_scale", config->sigma_scale);
  config->temporal_min_depth = node_->declare_parameter<decltype(config->temporal_min_depth)>(param_prefix + ".temporal_min_depth", config->temporal_min_depth);
  config->temporal_max_depth = node_->declare_parameter<decltype(config->temporal_max_depth)>(param_prefix + ".temporal_max_depth", config->temporal_max_depth);
  config->search_spatially = node_->declare_parameter<decltype(config->search_spatially)>(param_prefix + ".search_spatially", config->search_spatially);
  config->angle_weight = node_->declare_parameter<decltype(config->angle_weight)>(param_prefix + ".angle_weight", config->angle_weight);
  // clang-format on
  std::dynamic_pointer_cast<stereo::SubMapExtractionModule>(module)->setConfig(
      config);
}

void ROSModuleFactory::configureExperienceTriage(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::ExperienceTriageModule::Config>();
  // clang-format off
  config->verbose = node_->declare_parameter<decltype(config->verbose)>(param_prefix + ".verbose", config->verbose);
  config->always_privileged = node_->declare_parameter<decltype(config->always_privileged)>(param_prefix + ".always_privileged", config->always_privileged);
  config->in_the_loop = node_->declare_parameter<decltype(config->in_the_loop)>(param_prefix + ".in_the_loop", config->in_the_loop);
  // clang-format on
  std::dynamic_pointer_cast<stereo::ExperienceTriageModule>(module)->setConfig(
      config);
}

void ROSModuleFactory::configureLandmarkMigration(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::LandmarkMigrationModule::Config>();
  (void)param_prefix;
  std::dynamic_pointer_cast<stereo::LandmarkMigrationModule>(module)->setConfig(
      config);
}

void ROSModuleFactory::configureTodRecog(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::TodRecognitionModule::Config>();
  // clang-format off
  config->verbose = node_->declare_parameter<decltype(config->verbose)>(param_prefix + ".verbose", config->verbose);
  config->num_exp = node_->declare_parameter<decltype(config->num_exp)>(param_prefix + ".num_desired_experiences", config->num_exp);
  config->in_the_loop = node_->declare_parameter<decltype(config->in_the_loop)>(param_prefix + ".in_the_loop", config->in_the_loop);
  config->time_of_day_weight = node_->declare_parameter<decltype(config->time_of_day_weight)>(param_prefix + ".time_of_day_weight", config->time_of_day_weight);
  config->total_time_weight = node_->declare_parameter<decltype(config->total_time_weight)>(param_prefix + ".total_time_weight", config->total_time_weight);
  // clang-format on
  std::dynamic_pointer_cast<stereo::TodRecognitionModule>(module)->setConfig(
      config);
}

void ROSModuleFactory::configureMelMatcher(
    ModulePtr &module, const std::string &param_prefix) const {
  auto config = std::make_shared<stereo::MelMatcherModule::Config>();
  // clang-format off
  config->target_match_count = node_->declare_parameter<decltype(config->target_match_count)>(param_prefix + ".target_match_count", config->target_match_count);
  config->min_match_count = node_->declare_parameter<decltype(config->min_match_count)>(param_prefix + ".min_match_count", config->min_match_count);
  config->time_allowance = node_->declare_parameter<decltype(config->time_allowance)>(param_prefix + ".time_allowance", config->time_allowance);
  config->matching_pixel_thresh = node_->declare_parameter<decltype(config->matching_pixel_thresh)>(param_prefix + ".matching_pixel_thresh", config->matching_pixel_thresh);
  config->tight_matching_pixel_thresh = node_->declare_parameter<decltype(config->tight_matching_pixel_thresh)>(param_prefix + ".tight_matching_pixel_thresh", config->tight_matching_pixel_thresh);
  config->tight_matching_x_sigma = node_->declare_parameter<decltype(config->tight_matching_x_sigma)>(param_prefix + ".tight_matching_x_sigma", config->tight_matching_x_sigma);
  config->tight_matching_y_sigma = node_->declare_parameter<decltype(config->tight_matching_y_sigma)>(param_prefix + ".tight_matching_y_sigma", config->tight_matching_y_sigma);
  config->tight_matching_theta_sigma = node_->declare_parameter<decltype(config->tight_matching_theta_sigma)>(param_prefix + ".tight_matching_theta_sigma", config->tight_matching_theta_sigma);
  config->min_response_ratio = node_->declare_parameter<decltype(config->min_response_ratio)>(param_prefix + ".min_response_ratio", config->min_response_ratio);
  config->descriptor_thresh_cpu = node_->declare_parameter<decltype(config->descriptor_thresh_cpu)>(param_prefix + ".descriptor_thresh_cpu", config->descriptor_thresh_cpu);
  config->descriptor_thresh_gpu = node_->declare_parameter<decltype(config->descriptor_thresh_gpu)>(param_prefix + ".descriptor_thresh_gpu", config->descriptor_thresh_gpu);
  config->min_track_length = node_->declare_parameter<decltype(config->min_track_length)>(param_prefix + ".min_track_length", config->min_track_length);
  config->max_landmark_depth = node_->declare_parameter<decltype(config->max_landmark_depth)>(param_prefix + ".max_landmark_depth", config->max_landmark_depth);
  config->max_depth_diff = node_->declare_parameter<decltype(config->max_depth_diff)>(param_prefix + ".max_depth_diff", config->max_depth_diff);
  config->visualize = node_->declare_parameter<decltype(config->visualize)>(param_prefix + ".visualize", config->visualize);
  config->screen_matched_landmarks = node_->declare_parameter<decltype(config->screen_matched_landmarks)>(param_prefix + ".screen_matched_landmarks", config->screen_matched_landmarks);
  config->parallel_threads = node_->declare_parameter<decltype(config->parallel_threads)>(param_prefix + ".parallel_threads", config->parallel_threads);
#ifdef DETERMINISTIC_VTR
  LOG_IF(config->parallel_threads != 1, WARNING) << "MEL matcher number of threads set to 1 in deterministic mode.";
  config->parallel_threads = 1;
#endif
  config->match_on_gpu = node_->declare_parameter<decltype(config->match_on_gpu)>(param_prefix + ".match_on_gpu", config->match_on_gpu);
  config->match_gpu_knn_match_num = node_->declare_parameter<decltype(config->match_gpu_knn_match_num)>(param_prefix + ".match_gpu_knn_match_num", config->match_gpu_knn_match_num);
  // clang-format on
  std::dynamic_pointer_cast<stereo::MelMatcherModule>(module)->setConfig(
      config);
}

}  // namespace tactic
}  // namespace vtr