#include <vtr_navigation/factories/module_factory.hpp>
#include <vtr_navigation/factories/ros_module_factory.hpp>
#include <vtr_navigation/modules.hpp>

namespace vtr {
namespace navigation {

ROSModuleFactory::mod_ptr ROSModuleFactory::make_str(
    const std::string &type_str) const {
  // Right now, just call the default factory...
  auto new_module = ModuleFactory(type_str).make();
  if (!new_module) return new_module;
  configureModule(new_module, type_str);
  return new_module;
}

void ROSModuleFactory::configureModule(std::shared_ptr<BaseModule> &new_module,
                                       const std::string &type_str) const {
  if (isType<ConversionExtractionModule>(type_str))
    configureConversionExtractor(new_module);
  else if (isType<ImageTriangulationModule>(type_str))
    configureImageTriangulator(new_module);
#if false
  else if (isType<ASRLStereoMatcherModule>(type_str))
    configureASRLStereoMatcher(new_module);
  else if (isType<SimpleVertexTestModule>(type_str))
    configureSimpleVertexCreationTestModule(new_module);
  else if (isType<WindowedRecallModule>(type_str))
    configureWindowedRecallModule(new_module);
  else if (isType<KeyframeOptimizationModule>(type_str))
    configureKeyframeOptimization(new_module);
  else if (isType<WindowOptimizationModule>(type_str))
    configureWindowOptimization(new_module);
  else if (isType<StereoRansacModule>(type_str))
    configureStereoRANSAC(new_module);
  else if (isType<LandmarkRecallModule>(type_str))
    configureLandmarkRecallModule(new_module);
  else if (isType<LandmarkMigrationModule>(type_str))
    configureLandmarkMigration(new_module);
  else if (isType<SubMapExtractionModule>(type_str))
    configureSubMapExtraction(new_module);
  else if (isType<MelMatcherModule>(type_str))
    configureMelMatcher(new_module);
  else if (isType<ResultsModule>(type_str))
    configureResults(new_module);
  else if (isType<CollaborativeLandmarksModule>(type_str))
    configureCollabLandmarks(new_module);
  else if (isType<ExperienceTriageModule>(type_str))
    configureExperienceTriage(new_module);
  else if (isType<RandomExperiencesModule>(type_str))
    configureRandomExperiences(new_module);
  else if (isType<TodRecognitionModule>(type_str))
    configureTodRecog(new_module);
  else
    throw std::runtime_error("Cannot configure requested module.");

  /*
  if (isType<NoopModule>(type_str)) {
  } else if (isType<StereoRansacModule>(type_str)) {
    configureStereoRANSAC(new_module);
  } else if (isType<MonoRansacModule>(type_str)) {
    configureMonoRANSAC(new_module);
  } else if (isType<InitMonoRansacModule>(type_str)) {
    configureInitMonoRANSAC(new_module);
  } else if (isType<OpenCVStereoMatcherModule>(type_str)) {
    configureOpenCVStereoMatcher(new_module);
  } else if (isType<ASRLMonoMatcherModule>(type_str)) {
    configureASRLMonoMatcher(new_module);
  } else if (isType<ASRLStereoMatcherModule>(type_str)) {
    configureASRLStereoMatcher(new_module);
  } else if (isType<KeyframeOptimizationModule>(type_str)) {
    configureKeyframeOptimization(new_module);
  } else if (isType<FeatureExtractionModule>(type_str)) {
    configureFeatureExtractor(new_module);
  } else if (isType<ImageConversionModule>(type_str)) {
    configureImageConverter(new_module);
  } else if (isType<ConversionExtractionModule>(type_str)) {
    configureConversionExtractor(new_module);
  } else if (isType<CVStereoBMModule>(type_str)) {
    configureCVStereoBM(new_module);
  } else if (isType<CVGpuStereoBMModule>(type_str)) {
    configureCVGpuStereoBM(new_module);
  } else if (isType<ElasModule>(type_str)) {
    configureElas(new_module);
  } else if (isType<CVReprojectorModule>(type_str)) {
    configureCVReprojector(new_module);
  } else if (isType<CVGpuReprojectorModule>(type_str)) {
    configureCVGpuReprojector(new_module);
  } else if (isType<ImageTriangulationModule>(type_str)) {
    configureImageTriangulator(new_module);
  } else if (isType<MonoTriangulationModule>(type_str)) {
    configureMonoTriangulator(new_module);
  } else if (isType<SequentialTriangulationModule>(type_str)) {
    configureSequentialTriangulator(new_module);
  } else if (isType<SimpleVertexTestModule>(type_str)) {
    configureSimpleVertexCreationTestModule(new_module);
  } else if (isType<LancasterVertexTestModule>(type_str)) {
    configureLancasterVertexCreationTestModule(new_module);
  } else if (isType<GimbalVertexTestModule>(type_str)) {
    configureGimbalVertexCreationTestModule(new_module);
  } else if (isType<LandmarkRecallModule>(type_str)) {
    configureLandmarkRecallModule(new_module);
  } else if (isType<WindowedRecallModule>(type_str)) {
    configureWindowedRecallModule(new_module);
  } else if (isType<WindowOptimizationModule>(type_str)) {
    configureWindowOptimization(new_module);
  } else if (isType<SubMapExtractionModule>(type_str)) {
    configureSubMapExtraction(new_module);
  } else if (isType<LandmarkMigrationModule>(type_str)) {
    configureLandmarkMigration(new_module);
  } else if (isType<MelMatcherModule>(type_str)) {
    configureMelMatcher(new_module);
  } else if (isType<MelRecognitionModule>(type_str)) {
    configureMelRecog(new_module);
  } else if (isType<TodRecognitionModule>(type_str)) {
    configureTodRecog(new_module);
  } else if (isType<CollaborativeLandmarksModule>(type_str)) {
    configureCollabLandmarks(new_module);
  } else if (isType<RandomExperiencesModule>(type_str)) {
    configureRandomExperiences(new_module);
  } else if (isType<ExperienceTriageModule>(type_str)) {
    configureExperienceTriage(new_module);
  } else if (isType<ResultsModule>(type_str)) {
    configureResults(new_module);
  } else if (isType<QuickVORosPublisherModule>(type_str)) {
    configureQuickVORosPublisher(new_module);
  } else if (isType<RefinedVORosPublisherModule>(type_str)) {
    configureRefinedVORosPublisher(new_module);
  } else if (isType<LocalizationRosPublisherModule>(type_str)) {
    configureLocalizationRosPublisher(new_module);
  } else if (isType<MonoPlanarScalingModule>(type_str)) {
    configureMonoScaling(new_module);
  } else if (isType<UnderfootSeparateModule>(type_str)) {
    configureUnderfootSeparate(new_module);
  } else if (isType<UnderfootAggregateModule>(type_str)) {
    configureUnderfootAggregate(new_module);
  } else if (isType<LookaheadPatchGenerationModule>(type_str)) {
    configureLookaheadPatchGeneration(new_module);
  } else if (isType<CDMaxMinModule>(type_str)) {
    configureCDMaxMin(new_module);
  } else if (isType<CDMinMaxModule>(type_str)) {
    configureCDMinMax(new_module);
  } else if (isType<CDGmmModule>(type_str)) {
    configureCDGmm(new_module);
  } else if (isType<CDGpcModule>(type_str)) {
    configureCDGpc(new_module);
  } else if (isType<TrainingModule>(type_str)) {
    configureTraining(new_module);
  }
  */
#endif
}
#if false
void ROSModuleFactory::configureORBDetector(
    vision::ORBConfiguration &config) const {
  nh_->param<bool>(param_prefix_ + "extractor/orb/use_STAR_detector",
                   config.use_STAR_detector_, true);
  nh_->param<bool>(param_prefix_ + "extractor/orb/use_GPU_descriptors",
                   config.use_GPU_descriptors_, false);
  nh_->param<int>(param_prefix_ + "extractor/orb/STAR_maxSize",
                  config.STAR_maxSize_, 5);
  nh_->param<int>(param_prefix_ + "extractor/orb/STAR_responseThreshold",
                  config.STAR_responseThreshold_, 10);
  nh_->param<int>(param_prefix_ + "extractor/orb/STAR_lineThresholdProjected",
                  config.STAR_lineThresholdProjected_, 10);
  nh_->param<int>(param_prefix_ + "extractor/orb/STAR_lineThresholdBinarized",
                  config.STAR_lineThresholdBinarized_, 8);
  nh_->param<int>(param_prefix_ + "extractor/orb/STAR_suppressNonmaxSize",
                  config.STAR_suppressNonmaxSize_, 5);
  nh_->param<int>(param_prefix_ + "extractor/orb/num_detector_features",
                  config.num_detector_features_, 7000);
  nh_->param<int>(param_prefix_ + "extractor/orb/num_binned_features",
                  config.num_binned_features_, 800);
  nh_->param<double>(param_prefix_ + "extractor/orb/scaleFactor",
                     config.scaleFactor_, 1.2);
  nh_->param<int>(param_prefix_ + "extractor/orb/nlevels", config.nlevels_, 8);
  nh_->param<int>(param_prefix_ + "extractor/orb/edgeThreshold",
                  config.edgeThreshold_, 31);
  nh_->param<int>(param_prefix_ + "extractor/orb/firstLevel",
                  config.firstLevel_, 0);
  nh_->param<int>(param_prefix_ + "extractor/orb/WTA_K", config.WTA_K_, 2);
  nh_->param<bool>(param_prefix_ + "extractor/orb/upright_flag",
                   config.upright_, false);
  nh_->param<int>(param_prefix_ + "extractor/orb/num_threads",
                  config.num_threads_, 8);

  std::string scoreType;
  nh_->param<std::string>(param_prefix_ + "extractor/orb/scoreType", scoreType,
                          "HARRIS");
  if (scoreType == "HARRIS") {
    config.scoreType_ = cv::ORB::HARRIS_SCORE;
  } else if (scoreType == "FAST") {
    config.scoreType_ = cv::ORB::FAST_SCORE;
  }
  nh_->param<int>(param_prefix_ + "extractor/orb/patchSize", config.patchSize_,
                  64);  // \todo: 64 gives an error in cuda::ORB, max 59
  nh_->param<int>(param_prefix_ + "extractor/orb/fastThreshold",
                  config.fastThreshold_, 20);
  nh_->param<int>(param_prefix_ + "extractor/orb/x_bins", config.x_bins_, 3);
  nh_->param<int>(param_prefix_ + "extractor/orb/y_bins", config.y_bins_, 2);
  double descriptor_match_thresh;
  nh_->param<double>(
      param_prefix_ + "extractor/orb/matcher/descriptor_match_thresh",
      descriptor_match_thresh, 0.55);
  config.stereo_matcher_config_.descriptor_match_thresh_ =
      descriptor_match_thresh;
  double stereo_descriptor_match_thresh;
  nh_->param<double>(
      param_prefix_ + "extractor/orb/matcher/stereo_descriptor_match_thresh",
      stereo_descriptor_match_thresh, 0.55);
  config.stereo_matcher_config_.stereo_descriptor_match_thresh_ =
      stereo_descriptor_match_thresh;
  double stereo_y_tolerance;
  nh_->param<double>(param_prefix_ + "extractor/orb/matcher/stereo_y_tolerance",
                     stereo_y_tolerance, 1.0f);
  config.stereo_matcher_config_.stereo_y_tolerance_ = stereo_y_tolerance;
  nh_->param<double>(
      param_prefix_ + "extractor/orb/matcher/stereo_x_tolerance_min",
      config.stereo_matcher_config_.stereo_x_tolerance_min_, 0);
  nh_->param<double>(
      param_prefix_ + "extractor/orb/matcher/stereo_x_tolerance_max",
      config.stereo_matcher_config_.stereo_x_tolerance_max_, 16);
  nh_->param<bool>(param_prefix_ + "extractor/orb/matcher/check_octave",
                   config.stereo_matcher_config_.check_octave_, true);
  nh_->param<bool>(param_prefix_ + "extractor/orb/matcher/check_response",
                   config.stereo_matcher_config_.check_response_, true);
  double min_response_ratio;
  nh_->param<double>(param_prefix_ + "extractor/orb/matcher/min_response_ratio",
                     min_response_ratio, 0.1);
  config.stereo_matcher_config_.min_response_ratio_ = min_response_ratio;
  nh_->param<bool>(
      param_prefix_ + "extractor/orb/matcher/scale_x_tolerance_by_y",
      config.stereo_matcher_config_.scale_x_tolerance_by_y_, true);
  double x_tolerance_scale;
  nh_->param<double>(param_prefix_ + "extractor/orb/matcher/x_tolerance_scale",
                     x_tolerance_scale, 768);
  config.stereo_matcher_config_.x_tolerance_scale_ = x_tolerance_scale;
}

#if GPUSURF_ENABLED
void ROSModuleFactory::configureSURFDetector(
    asrl::GpuSurfConfiguration &config) const {
  double val = 0;
  nh_->param<double>(param_prefix_ + "extractor/surf/threshold", val, 1e-7);
  config.threshold = val;
  nh_->param<bool>(param_prefix_ + "extractor/surf/upright_flag",
                   config.upright_flag, true);
  nh_->param<int>(param_prefix_ + "extractor/surf/nOctaves", config.nOctaves,
                  4);
  nh_->param<int>(param_prefix_ + "extractor/surf/nIntervals",
                  config.nIntervals, 4);
  nh_->param<double>(param_prefix_ + "extractor/surf/initialScale", val, 1.5);
  config.initialScale = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/edgeScale", val, 1.5);
  config.edgeScale = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/l1", val, 3.f / 1.5f);
  config.l1 = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/l2", val, 5.f / 1.5f);
  config.l2 = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/l3", val, 3.f / 1.5f);
  config.l3 = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/l4", val, 1.f / 1.5f);
  config.l4 = val;
  nh_->param<int>(param_prefix_ + "extractor/surf/initialStep",
                  config.initialStep, 1);
  nh_->param<int>(param_prefix_ + "extractor/surf/targetFeatures",
                  config.targetFeatures, 800);
  nh_->param<int>(param_prefix_ + "extractor/surf/detector_threads_x",
                  config.detector_threads_x, 16);
  nh_->param<int>(param_prefix_ + "extractor/surf/detector_threads_y",
                  config.detector_threads_y, 4);
  nh_->param<int>(param_prefix_ + "extractor/surf/nonmax_threads_x",
                  config.nonmax_threads_x, 16);
  nh_->param<int>(param_prefix_ + "extractor/surf/nonmax_threads_y",
                  config.nonmax_threads_y, 16);
  nh_->param<int>(param_prefix_ + "extractor/surf/regions_horizontal",
                  config.regions_horizontal, 8);
  nh_->param<int>(param_prefix_ + "extractor/surf/regions_vertical",
                  config.regions_vertical, 6);
  nh_->param<int>(param_prefix_ + "extractor/surf/regions_target",
                  config.regions_target, 800);
}

void ROSModuleFactory::configureSURFStereoDetector(
    asrl::GpuSurfStereoConfiguration &config) const {
  configureSURFDetector(config);
  double val = 0;
  nh_->param<double>(param_prefix_ + "extractor/surf/stereoDisparityMinimum",
                     val, 0.0);
  config.stereoDisparityMinimum = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/stereoDisparityMaximum",
                     val, 120.0);
  config.stereoDisparityMaximum = val;
  nh_->param<double>(
      param_prefix_ + "extractor/surf/stereoCorrelationThreshold", val, 0.79);
  config.stereoCorrelationThreshold = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/stereoYTolerance", val,
                     1.0);
  config.stereoYTolerance = val;
  nh_->param<double>(param_prefix_ + "extractor/surf/stereoScaleTolerance", val,
                     0.8);
  config.stereoScaleTolerance = val;
}
#endif
#endif
void ROSModuleFactory::configureConversionExtractor(
    std::shared_ptr<BaseModule> &new_module) const {
  /// std::shared_ptr<ConversionExtractionModule::Config> config;
  /// config.reset(new ConversionExtractionModule::Config());
  auto config = std::make_shared<ConversionExtractionModule::Config>();
  config->conversions = node_->declare_parameter<decltype(config->conversions)>(
      param_prefix_ + ".conversions", config->conversions);
  config->color_constant_weights =
      node_->declare_parameter<decltype(config->color_constant_weights)>(
          param_prefix_ + ".color_constant.weights",
          config->color_constant_weights);
  config->color_constant_histogram_equalization =
      node_->declare_parameter<decltype(
          config->color_constant_histogram_equalization)>(
          param_prefix_ + ".color_constant.histogram_equalization",
          config->color_constant_histogram_equalization);
  config->feature_type =
      node_->declare_parameter<decltype(config->feature_type)>(
          param_prefix_ + ".extractor.type", config->feature_type);
  config->visualize_raw_features =
      node_->declare_parameter<decltype(config->visualize_raw_features)>(
          param_prefix_ + ".extractor.visualize_raw_features",
          config->visualize_raw_features);

#if false
  // configure the detector
  if (config->feature_type == "OPENCV_ORB") {
    configureORBDetector(config->opencv_orb_params);
  } else if (config->feature_type == "ASRL_GPU_SURF") {
#if GPUSURF_ENABLED
    configureSURFDetector(config->gpu_surf_params);
    configureSURFStereoDetector(config->gpu_surf_stereo_params);
#else
    throw std::runtime_error(
        "ROSModuleFactory::configureFeatureExtractor: GPU SURF isn't enabled!");
#endif
  } else {
    throw std::runtime_error(
        "Couldn't determine feature type when building ConversionExtraction "
        "Module!");
  }
#endif
  std::dynamic_pointer_cast<ConversionExtractionModule>(new_module)
      ->setConfig(config);
}
#if false
#if 0
void ROSModuleFactory::configureFeatureExtractor(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<FeatureExtractionModule::Config> config;
  config.reset(new FeatureExtractionModule::Config());

  nh_->param<std::string>(param_prefix_ + "extractor/type",
                          config->feature_type, "ASRL_GPU_SURF");
  nh_->param<bool>(param_prefix_ + "extractor/visualize_raw_features",
                   config->visualize_raw_features, false);
  nh_->getParam(param_prefix_ + "extractor/channels", config->channels);

  // configure the detector
  if (config->feature_type == "OPENCV_ORB") {
    configureORBDetector(config->opencv_orb_params);
  } else if (config->feature_type == "ASRL_GPU_SURF") {
#if GPUSURF_ENABLED
    configureSURFDetector(config->gpu_surf_params);
    configureSURFStereoDetector(config->gpu_surf_stereo_params);
#else
    throw std::runtime_error(
        "ROSModuleFactory::configureFeatureExtractor: GPU SURF isn't enabled!");
#endif
  } else {
    throw std::runtime_error(
        "Couldn't determine feature type when building ConversionExtraction "
        "Module!");
  }

  std::dynamic_pointer_cast<FeatureExtractionModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureImageConverter(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<ImageConversionModule::Config> config;
  config.reset(new ImageConversionModule::Config());

  nh_->getParam(param_prefix_ + "/conversions", config->conversions);
  nh_->getParam(param_prefix_ + "/color_constant/weights",
                config->color_constant_weights);
  nh_->getParam(param_prefix_ + "/color_constant/histogram_equalization",
                config->color_constant_histogram_equalization);
  std::dynamic_pointer_cast<ImageConversionModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureCVStereoBM(
    std::shared_ptr<BaseModule> &new_module) const {
  CVStereoBMModule::Config config;

  nh_->getParam(param_prefix_ + "/max_fps", config.max_fps);
  nh_->getParam(param_prefix_ + "/visualize", config.visualize);

  std::dynamic_pointer_cast<CVStereoBMModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureCVStereoSgbm(
    std::shared_ptr<BaseModule> &new_module) const {
  CVStereoSgbmModule::Config config;

  nh_->getParam(param_prefix_ + "/max_fps", config.max_fps);
  nh_->getParam(param_prefix_ + "/visualize", config.visualize);

  config.cv_stereo_sgbm_params =
      std::make_shared<asrl::vision::CVStereoSGBM::Params>();
  nh_->getParam(param_prefix_ + "/minDisparity",
                config.cv_stereo_sgbm_params->minDisparity);
  nh_->getParam(param_prefix_ + "/numberOfDisparities",
                config.cv_stereo_sgbm_params->numberOfDisparities);
  nh_->getParam(param_prefix_ + "/SADWindowSize",
                config.cv_stereo_sgbm_params->SADWindowSize);
  nh_->getParam(param_prefix_ + "/preFilterCap",
                config.cv_stereo_sgbm_params->preFilterCap);
  nh_->getParam(param_prefix_ + "/uniquenessRatio",
                config.cv_stereo_sgbm_params->uniquenessRatio);
  nh_->getParam(param_prefix_ + "/P1", config.cv_stereo_sgbm_params->P1);
  nh_->getParam(param_prefix_ + "/P2", config.cv_stereo_sgbm_params->P2);
  nh_->getParam(param_prefix_ + "/speckleWindowSize",
                config.cv_stereo_sgbm_params->speckleWindowSize);
  nh_->getParam(param_prefix_ + "/speckleRange",
                config.cv_stereo_sgbm_params->speckleRange);
  nh_->getParam(param_prefix_ + "/disp12MaxDiff",
                config.cv_stereo_sgbm_params->disp12MaxDiff);
  nh_->getParam(param_prefix_ + "/fullDP",
                config.cv_stereo_sgbm_params->fullDP);
  nh_->getParam(param_prefix_ + "/use_left_right_consistency",
                config.cv_stereo_sgbm_params->use_left_right_consistency);
  nh_->getParam(param_prefix_ + "/left_right_tolerance",
                config.cv_stereo_sgbm_params->left_right_tolerance);

  std::dynamic_pointer_cast<CVStereoSgbmModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureCVGpuStereoBM(
    std::shared_ptr<BaseModule> &new_module) const {
  CVGpuStereoBMModule::Config config;

  nh_->getParam(param_prefix_ + "/max_fps", config.max_fps);
  nh_->getParam(param_prefix_ + "/visualize", config.visualize);

#if defined(HAVE_OPENCV_CUDASTEREO) || defined(HAVE_OPENCV_GPU)
  config.gpu_bm_params = std::make_shared<asrl::vision::CVGpuStereoBM::Params>();
  nh_->getParam(param_prefix_ + "/preset", config.gpu_bm_params->preset);
  nh_->getParam(param_prefix_ + "/ndisp", config.gpu_bm_params->ndisp);
  nh_->getParam(param_prefix_ + "/winSize", config.gpu_bm_params->winSize);
  nh_->getParam(param_prefix_ + "/avergeTexThreshold",
                config.gpu_bm_params->avergeTexThreshold);
  nh_->getParam(param_prefix_ + "/use_left_right_consistency",
                config.gpu_bm_params->use_left_right_consistency);
  nh_->getParam(param_prefix_ + "/left_right_tolerance",
                config.gpu_bm_params->left_right_tolerance);

  nh_->getParam(param_prefix_ + "/postfilters/gpu_bilateral_filter/k",
                config.gpu_bilateral_filter_config.k);
  nh_->getParam(param_prefix_ + "/postfilters/gpu_bilateral_filter/sigma_color",
                config.gpu_bilateral_filter_config.sigma_color);
  nh_->getParam(
      param_prefix_ + "/postfilters/gpu_bilateral_filter/sigma_spatial",
      config.gpu_bilateral_filter_config.sigma_spatial);
#endif

  std::dynamic_pointer_cast<CVGpuStereoBMModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureElas(
    std::shared_ptr<BaseModule> &new_module) const {
  ElasModule::Config config;

  nh_->getParam(param_prefix_ + "/max_fps", config.max_fps);
  nh_->getParam(param_prefix_ + "/visualize", config.visualize);
#if ELAS_ENABLED
  nh_->getParam(param_prefix_ + "/disp_min", config.elas_params.disp_min);
  nh_->getParam(param_prefix_ + "/disp_max", config.elas_params.disp_max);
  nh_->getParam(param_prefix_ + "/support_threshold",
                config.elas_params.support_threshold);
  nh_->getParam(param_prefix_ + "/support_texture",
                config.elas_params.support_texture);
  nh_->getParam(param_prefix_ + "/candidate_stepsize",
                config.elas_params.candidate_stepsize);
  nh_->getParam(param_prefix_ + "/incon_window_size",
                config.elas_params.incon_window_size);
  nh_->getParam(param_prefix_ + "/incon_threshold",
                config.elas_params.incon_threshold);
  nh_->getParam(param_prefix_ + "/incon_min_support",
                config.elas_params.incon_min_support);
  nh_->getParam(param_prefix_ + "/add_corners", config.elas_params.add_corners);
  nh_->getParam(param_prefix_ + "/grid_size", config.elas_params.grid_size);
  nh_->getParam(param_prefix_ + "/beta", config.elas_params.beta);
  nh_->getParam(param_prefix_ + "/gamma", config.elas_params.gamma);
  nh_->getParam(param_prefix_ + "/sigma", config.elas_params.sigma);
  nh_->getParam(param_prefix_ + "/sradius", config.elas_params.sradius);
  nh_->getParam(param_prefix_ + "/match_texture",
                config.elas_params.match_texture);
  nh_->getParam(param_prefix_ + "/lr_threshold",
                config.elas_params.lr_threshold);
  nh_->getParam(param_prefix_ + "/speckle_sim_threshold",
                config.elas_params.speckle_sim_threshold);
  nh_->getParam(param_prefix_ + "/speckle_size",
                config.elas_params.speckle_size);
  nh_->getParam(param_prefix_ + "/ipol_gap_width",
                config.elas_params.ipol_gap_width);
  nh_->getParam(param_prefix_ + "/filter_median",
                config.elas_params.filter_median);
  nh_->getParam(param_prefix_ + "/filter_adaptive_mean",
                config.elas_params.filter_adaptive_mean);
  nh_->getParam(param_prefix_ + "/postprocess_only_left",
                config.elas_params.postprocess_only_left);
  nh_->getParam(param_prefix_ + "/subsampling", config.elas_params.subsampling);
#endif
  std::dynamic_pointer_cast<ElasModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureCVReprojector(
    std::shared_ptr<BaseModule> &new_module) const {
  CVReprojectorModule::Config config;

  nh_->getParam(param_prefix_ + "/max_depth", config.max_depth);
  nh_->getParam(param_prefix_ + "/sample_rate", config.sample_rate);

  std::dynamic_pointer_cast<CVReprojectorModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureCVGpuReprojector(
    std::shared_ptr<BaseModule> &new_module) const {
  CVGpuReprojectorModule::Config config;

  nh_->getParam(param_prefix_ + "/max_depth", config.max_depth);
  nh_->getParam(param_prefix_ + "/sample_rate", config.sample_rate);

  std::dynamic_pointer_cast<CVGpuReprojectorModule>(new_module)
      ->setConfig(config);
}
#endif
#endif
void ROSModuleFactory::configureImageTriangulator(
    std::shared_ptr<BaseModule> &new_module) const {
  /// std::shared_ptr<ImageTriangulationModule::Config> config;
  /// config.reset(new ImageTriangulationModule::Config());
  auto config = std::make_shared<ImageTriangulationModule::Config>();
  config->visualize_features =
      node_->declare_parameter<decltype(config->visualize_features)>(
          param_prefix_ + ".visualize_features", config->visualize_features);
  config->visualize_stereo_features =
      node_->declare_parameter<decltype(config->visualize_stereo_features)>(
          param_prefix_ + ".visualize_stereo_features",
          config->visualize_stereo_features);
  config->min_triangulation_depth =
      node_->declare_parameter<decltype(config->min_triangulation_depth)>(
          param_prefix_ + ".min_triangulation_depth",
          config->min_triangulation_depth);
  config->max_triangulation_depth =
      node_->declare_parameter<decltype(config->max_triangulation_depth)>(
          param_prefix_ + ".max_triangulation_depth",
          config->max_triangulation_depth);
  std::dynamic_pointer_cast<ImageTriangulationModule>(new_module)
      ->setConfig(config);
}
#if false
#if 0
void ROSModuleFactory::configureMonoTriangulator(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<MonoTriangulationModule::Config> config;
  config.reset(new MonoTriangulationModule::Config());

  nh_->getParam(param_prefix_ + "visualize_features",
                config->visualize_features);
  nh_->getParam(param_prefix_ + "min_new_points", config->min_new_points);
  nh_->getParam(param_prefix_ + "min_new_points_init",
                config->min_new_points_init);
  nh_->getParam(param_prefix_ + "min_depth", config->min_depth);
  nh_->getParam(param_prefix_ + "max_depth", config->max_depth);
  nh_->getParam(param_prefix_ + "plane_distance", config->plane_distance);
  nh_->getParam(param_prefix_ + "max_reprojection_error",
                config->max_reprojection_error);

  std::dynamic_pointer_cast<MonoTriangulationModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureSequentialTriangulator(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<SequentialTriangulationModule::Config> config;
  config.reset(new SequentialTriangulationModule::Config());

  nh_->getParam(param_prefix_ + "visualize_features",
                config->visualize_features);
  nh_->getParam(param_prefix_ + "min_new_points", config->min_new_points);
  nh_->getParam(param_prefix_ + "min_depth", config->min_depth);
  nh_->getParam(param_prefix_ + "max_depth", config->max_depth);
  nh_->getParam(param_prefix_ + "max_reprojection_error",
                config->max_reprojection_error);

  std::dynamic_pointer_cast<SequentialTriangulationModule>(new_module)
      ->setConfig(config);
}
#endif

void ROSModuleFactory::configureSteam(
    std::shared_ptr<SteamModule::Config> &config) const {
  // TODO: change to ros::param::param<type> to use default values.
  nh_->getParam(param_prefix_ + "solver_type", config->solver_type);
  nh_->getParam(param_prefix_ + "loss_function", config->loss_function);
  nh_->getParam(param_prefix_ + "verbose", config->verbose);
  nh_->getParam(param_prefix_ + "use_T_q_m_prior", config->use_T_q_m_prior);

  nh_->getParam(param_prefix_ + "iterations", config->iterations);
  nh_->getParam(param_prefix_ + "absoluteCostThreshold",
                config->absoluteCostThreshold);
  nh_->getParam(param_prefix_ + "absoluteCostChangeThreshold",
                config->absoluteCostChangeThreshold);
  nh_->getParam(param_prefix_ + "relativeCostChangeThreshold",
                config->relativeCostChangeThreshold);

  nh_->getParam(param_prefix_ + "ratioThresholdShrink",
                config->ratioThresholdShrink);
  nh_->getParam(param_prefix_ + "ratioThresholdGrow",
                config->ratioThresholdGrow);
  nh_->getParam(param_prefix_ + "shrinkCoeff", config->shrinkCoeff);
  nh_->getParam(param_prefix_ + "growCoeff", config->growCoeff);
  nh_->getParam(param_prefix_ + "maxShrinkSteps", config->maxShrinkSteps);
  nh_->getParam(param_prefix_ + "backtrackMultiplier",
                config->backtrackMultiplier);
  nh_->getParam(param_prefix_ + "maxBacktrackSteps", config->maxBacktrackSteps);

  // validity checking
  nh_->getParam(param_prefix_ + "perform_planarity_check",
                config->perform_planarity_check);
  nh_->getParam(param_prefix_ + "plane_distance", config->plane_distance);
  nh_->getParam(param_prefix_ + "min_point_depth", config->min_point_depth);
  nh_->getParam(param_prefix_ + "max_point_depth", config->max_point_depth);

  // trajectory stuff.
  nh_->getParam(param_prefix_ + "save_trajectory", config->save_trajectory);
  nh_->getParam(param_prefix_ + "trajectory_smoothing",
                config->trajectory_smoothing);
  nh_->getParam(param_prefix_ + "lin_acc_std_dev_x", config->lin_acc_std_dev_x);
  nh_->getParam(param_prefix_ + "lin_acc_std_dev_y", config->lin_acc_std_dev_y);
  nh_->getParam(param_prefix_ + "lin_acc_std_dev_z", config->lin_acc_std_dev_z);
  nh_->getParam(param_prefix_ + "ang_acc_std_dev_x", config->ang_acc_std_dev_x);
  nh_->getParam(param_prefix_ + "ang_acc_std_dev_y", config->ang_acc_std_dev_y);
  nh_->getParam(param_prefix_ + "ang_acc_std_dev_z", config->ang_acc_std_dev_z);
  nh_->getParam(param_prefix_ + "disable_solver", config->disable_solver);
  // velocity prior
  nh_->getParam(param_prefix_ + "velocity_prior", config->velocity_prior);
  nh_->getParam(param_prefix_ + "lin_vel_mean_x", config->lin_vel_mean_x);
  nh_->getParam(param_prefix_ + "lin_vel_mean_y", config->lin_vel_mean_y);
  nh_->getParam(param_prefix_ + "lin_vel_mean_z", config->lin_vel_mean_z);
  nh_->getParam(param_prefix_ + "ang_vel_mean_x", config->ang_vel_mean_x);
  nh_->getParam(param_prefix_ + "ang_vel_mean_y", config->ang_vel_mean_y);
  nh_->getParam(param_prefix_ + "ang_vel_mean_z", config->ang_vel_mean_z);

  nh_->getParam(param_prefix_ + "lin_vel_std_dev_x", config->lin_vel_std_dev_x);
  nh_->getParam(param_prefix_ + "lin_vel_std_dev_y", config->lin_vel_std_dev_y);
  nh_->getParam(param_prefix_ + "lin_vel_std_dev_z", config->lin_vel_std_dev_z);
  nh_->getParam(param_prefix_ + "ang_vel_std_dev_x", config->ang_vel_std_dev_x);
  nh_->getParam(param_prefix_ + "ang_vel_std_dev_y", config->ang_vel_std_dev_y);
  nh_->getParam(param_prefix_ + "ang_vel_std_dev_z", config->ang_vel_std_dev_z);
}

void ROSModuleFactory::configureKeyframeOptimization(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<KeyframeOptimizationModule::Config> config;
  config.reset(new KeyframeOptimizationModule::Config());

  // Base Config
  auto base_config = std::dynamic_pointer_cast<SteamModule::Config>(config);
  configureSteam(base_config);

  nh_->getParam(param_prefix_ + "pose_prior_enable", config->pose_prior_enable);
  nh_->getParam(param_prefix_ + "depth_prior_enable",
                config->depth_prior_enable);
  nh_->getParam(param_prefix_ + "depth_prior_weight",
                config->depth_prior_weight);
  nh_->getParam(param_prefix_ + "max_point_depth", config->max_point_depth);
  nh_->getParam(param_prefix_ + "use_migrated_points",
                config->use_migrated_points);

  std::dynamic_pointer_cast<KeyframeOptimizationModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureWindowOptimization(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<WindowOptimizationModule::Config> config;
  config.reset(new WindowOptimizationModule::Config());

  // Base Config
  auto base_config = std::dynamic_pointer_cast<SteamModule::Config>(config);
  configureSteam(base_config);

  nh_->getParam(param_prefix_ + "depth_prior_enable",
                config->depth_prior_enable);
  nh_->getParam(param_prefix_ + "depth_prior_weight",
                config->depth_prior_weight);
  nh_->getParam(param_prefix_ + "max_point_depth", config->max_point_depth);

  std::dynamic_pointer_cast<WindowOptimizationModule>(new_module)
      ->setConfig(config);
}

#if 0
void ROSModuleFactory::configureOpenCVStereoMatcher(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<OpenCVStereoMatcherModule::Config> config;
  config.reset(new OpenCVStereoMatcherModule::Config());

  nh_->getParam(param_prefix_ + "nn_match_ratio", config->nn_match_ratio);
  nh_->getParam(param_prefix_ + "forward_matching_pixel_thresh",
                config->forward_matching_pixel_thresh);
  nh_->getParam(param_prefix_ + "max_point_depth", config->max_point_depth);
  nh_->getParam(param_prefix_ + "descriptor_thresh", config->descriptor_thresh);

  std::dynamic_pointer_cast<OpenCVStereoMatcherModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureASRLMonoMatcher(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<ASRLMonoMatcherModule::Config> config;
  config.reset(new ASRLMonoMatcherModule::Config());

  nh_->getParam(param_prefix_ + "check_laplacian_bit",
                config->check_laplacian_bit);
  nh_->getParam(param_prefix_ + "check_octave", config->check_octave);
  nh_->getParam(param_prefix_ + "check_response", config->check_response);
  nh_->getParam(param_prefix_ + "min_response_ratio",
                config->min_response_ratio);
  nh_->getParam(param_prefix_ + "matching_pixel_thresh",
                config->matching_pixel_thresh);
  nh_->getParam(param_prefix_ + "tight_matching_pixel_thresh",
                config->tight_matching_pixel_thresh);
  nh_->getParam(param_prefix_ + "tight_matching_x_sigma",
                config->tight_matching_x_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_y_sigma",
                config->tight_matching_y_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_phi_sigma",
                config->tight_matching_phi_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_theta_sigma",
                config->tight_matching_theta_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_psi_sigma",
                config->tight_matching_psi_sigma);
  nh_->getParam(param_prefix_ + "use_pixel_variance",
                config->use_pixel_variance);
  std::string prediction_method;
  nh_->getParam(param_prefix_ + "prediction_method", prediction_method);
  if (!prediction_method.compare("se3"))
    config->prediction_method = ASRLMonoMatcherModule::se3;
  else if (!prediction_method.compare("none"))
    config->prediction_method = ASRLMonoMatcherModule::none;
  else
    config->prediction_method = ASRLMonoMatcherModule::none;
  nh_->getParam(param_prefix_ + "max_point_depth", config->max_point_depth);
  nh_->getParam(param_prefix_ + "descriptor_thresh_cpu",
                config->descriptor_thresh_cpu);
  nh_->getParam(param_prefix_ + "descriptor_thresh_gpu",
                config->descriptor_thresh_gpu);
  nh_->getParam(param_prefix_ + "parallel_threads", config->parallel_threads);
  nh_->getParam(param_prefix_ + "visualize_feature_matches",
                config->visualize_feature_matches);
  nh_->getParam(param_prefix_ + "min_matches", config->min_matches);
  nh_->getParam(param_prefix_ + "min_window_size", config->min_window_size);
  nh_->getParam(param_prefix_ + "max_window_size", config->max_window_size);
  nh_->getParam(param_prefix_ + "match_on_gpu", config->match_on_gpu);
  nh_->getParam(param_prefix_ + "match_gpu_tight_knn_match_num",
                config->match_gpu_tight_knn_match_num);
  nh_->getParam(param_prefix_ + "match_gpu_loose_knn_match_num",
                config->match_gpu_loose_knn_match_num);
  std::dynamic_pointer_cast<ASRLMonoMatcherModule>(new_module)
      ->setConfig(config);
}

#endif
void ROSModuleFactory::configureASRLStereoMatcher(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<ASRLStereoMatcherModule::Config> config;
  config.reset(new ASRLStereoMatcherModule::Config());

  nh_->getParam(param_prefix_ + "check_laplacian_bit",
                config->check_laplacian_bit);
  nh_->getParam(param_prefix_ + "check_octave", config->check_octave);
  nh_->getParam(param_prefix_ + "check_response", config->check_response);
  nh_->getParam(param_prefix_ + "min_response_ratio",
                config->min_response_ratio);
  nh_->getParam(param_prefix_ + "matching_pixel_thresh",
                config->matching_pixel_thresh);
  nh_->getParam(param_prefix_ + "tight_matching_pixel_thresh",
                config->tight_matching_pixel_thresh);
  nh_->getParam(param_prefix_ + "tight_matching_x_sigma",
                config->tight_matching_x_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_y_sigma",
                config->tight_matching_y_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_theta_sigma",
                config->tight_matching_theta_sigma);
  nh_->getParam(param_prefix_ + "use_pixel_variance",
                config->use_pixel_variance);
  std::string prediction_method;
  nh_->getParam(param_prefix_ + "prediction_method", prediction_method);
  if (!prediction_method.compare("se3"))
    config->prediction_method = ASRLStereoMatcherModule::se3;
  else if (!prediction_method.compare("none"))
    config->prediction_method = ASRLStereoMatcherModule::none;
  else
    config->prediction_method = ASRLStereoMatcherModule::none;
  nh_->getParam(param_prefix_ + "max_point_depth", config->max_point_depth);
  nh_->getParam(param_prefix_ + "descriptor_thresh", config->descriptor_thresh);
  nh_->getParam(param_prefix_ + "parallel_threads", config->parallel_threads);
  nh_->getParam(param_prefix_ + "visualize_feature_matches",
                config->visualize_feature_matches);

  std::dynamic_pointer_cast<ASRLStereoMatcherModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureRANSAC(
    std::shared_ptr<RansacModule::Config> &config) const {
  // Base RANSAC
  nh_->getParam(param_prefix_ + "enable", config->enable);
  nh_->getParam(param_prefix_ + "iterations", config->iterations);
  nh_->getParam(param_prefix_ + "flavor", config->flavor);
  nh_->getParam(param_prefix_ + "sigma", config->sigma);
  nh_->getParam(param_prefix_ + "threshold", config->threshold);
  nh_->getParam(param_prefix_ + "early_stop_ratio", config->early_stop_ratio);
  nh_->getParam(param_prefix_ + "early_stop_min_inliers",
                config->early_stop_min_inliers);
  nh_->getParam(param_prefix_ + "visualize_ransac_inliers",
                config->visualize_ransac_inliers);
  nh_->getParam(param_prefix_ + "use_migrated_points",
                config->use_migrated_points);
  nh_->getParam(param_prefix_ + "min_inliers", config->min_inliers);
  nh_->getParam(param_prefix_ + "enable_local_opt", config->enable_local_opt);
  nh_->getParam(param_prefix_ + "num_threads", config->num_threads);
  // sanity check
  if (config->num_threads < 1) {
    config->num_threads = 1;
  }
}

void ROSModuleFactory::configureStereoRANSAC(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<StereoRansacModule::Config> config;
  config.reset(new StereoRansacModule::Config());

  // Base Config
  auto base_config = std::dynamic_pointer_cast<RansacModule::Config>(config);
  configureRANSAC(base_config);

  // Stereo RANSAC Config
  nh_->getParam(param_prefix_ + "mask_depth", config->mask_depth);
  nh_->getParam(param_prefix_ + "mask_depth_inlier_count",
                config->mask_depth_inlier_count);
  nh_->getParam(param_prefix_ + "visualize_ransac_inliers",
                config->visualize_ransac_inliers);
  nh_->getParam(param_prefix_ + "use_covariance", config->use_covariance);

  std::dynamic_pointer_cast<StereoRansacModule>(new_module)->setConfig(config);
}

#if 0
void ROSModuleFactory::configureInitMonoRANSAC(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<InitMonoRansacModule::Config> config;
  config.reset(new InitMonoRansacModule::Config());

  // Init Mono RANSAC Config
  nh_->getParam(param_prefix_ + "ransac_prob", config->ransac_prob);
  nh_->getParam(param_prefix_ + "ransac_thresh", config->ransac_thresh);
  nh_->getParam(param_prefix_ + "min_inliers", config->min_inliers);
  nh_->getParam(param_prefix_ + "min_inlier_ratio", config->min_inlier_ratio);
  nh_->getParam(param_prefix_ + "visualize_ransac_inliers",
                config->visualize_ransac_inliers);

  std::dynamic_pointer_cast<InitMonoRansacModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureMonoRANSAC(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<MonoRansacModule::Config> config;
  config.reset(new MonoRansacModule::Config());

  // Base Config
  auto base_config = std::dynamic_pointer_cast<RansacModule::Config>(config);
  configureRANSAC(base_config);

  // Mono RANSAC Config
  nh_->getParam(param_prefix_ + "mask_inlier_count", config->mask_inlier_count);
  nh_->getParam(param_prefix_ + "visualize_ransac_inliers",
                config->visualize_ransac_inliers);

  std::dynamic_pointer_cast<MonoRansacModule>(new_module)->setConfig(config);
}
#endif

void ROSModuleFactory::configureSimpleVertexCreationTestModule(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<SimpleVertexTestModule::Config> config;
  config.reset(new SimpleVertexTestModule::Config());

  nh_->getParam(param_prefix_ + "min_creation_distance",
                config->min_creation_distance);
  nh_->getParam(param_prefix_ + "max_creation_distance",
                config->max_creation_distance);
  nh_->getParam(param_prefix_ + "min_distance", config->min_distance);
  nh_->getParam(param_prefix_ + "rotation_threshold_min",
                config->rotation_threshold_min);
  nh_->getParam(param_prefix_ + "rotation_threshold_max",
                config->rotation_threshold_max);
  nh_->getParam(param_prefix_ + "match_threshold_min_count",
                config->match_threshold_min_count);
  nh_->getParam(param_prefix_ + "match_threshold_fail_count",
                config->match_threshold_fail_count);

  std::dynamic_pointer_cast<SimpleVertexTestModule>(new_module)
      ->setConfig(config);
}

#if 0
void ROSModuleFactory::configureLancasterVertexCreationTestModule(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<LancasterVertexTestModule::Config> config;
  config.reset(new LancasterVertexTestModule::Config());

  nh_->getParam(param_prefix_ + "min_creation_distance",
                config->min_creation_distance);
  nh_->getParam(param_prefix_ + "max_creation_distance",
                config->max_creation_distance);
  nh_->getParam(param_prefix_ + "min_distance", config->min_distance);
  nh_->getParam(param_prefix_ + "rotation_threshold_min",
                config->rotation_threshold_min);
  nh_->getParam(param_prefix_ + "rotation_threshold_max",
                config->rotation_threshold_max);
  nh_->getParam(param_prefix_ + "match_threshold_min_count",
                config->match_threshold_min_count);
  nh_->getParam(param_prefix_ + "match_threshold_fail_count",
                config->match_threshold_fail_count);
  nh_->getParam(param_prefix_ + "tri_threshold_min_count",
                config->tri_threshold_min_count);
  nh_->getParam(param_prefix_ + "tri_threshold_fail_count",
                config->tri_threshold_fail_count);
  nh_->getParam(param_prefix_ + "tri_threshold_init_min_count",
                config->tri_threshold_init_min_count);
  nh_->getParam(param_prefix_ + "tri_threshold_init_fail_count",
                config->tri_threshold_init_fail_count);

  std::dynamic_pointer_cast<LancasterVertexTestModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureGimbalVertexCreationTestModule(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<GimbalVertexTestModule::Config> config;
  config.reset(new GimbalVertexTestModule::Config());

  nh_->getParam(param_prefix_ + "min_creation_distance",
                config->min_creation_distance);
  nh_->getParam(param_prefix_ + "max_creation_distance",
                config->max_creation_distance);
  nh_->getParam(param_prefix_ + "min_distance", config->min_distance);
  nh_->getParam(param_prefix_ + "rotation_threshold_min",
                config->rotation_threshold_min);
  nh_->getParam(param_prefix_ + "rotation_threshold_max",
                config->rotation_threshold_max);
  nh_->getParam(param_prefix_ + "match_threshold_min_count",
                config->match_threshold_min_count);
  nh_->getParam(param_prefix_ + "match_threshold_fail_count",
                config->match_threshold_fail_count);
  nh_->getParam(param_prefix_ + "tri_threshold_min_count",
                config->tri_threshold_min_count);
  nh_->getParam(param_prefix_ + "tri_threshold_fail_count",
                config->tri_threshold_fail_count);
  nh_->getParam(param_prefix_ + "tri_threshold_init_min_count",
                config->tri_threshold_init_min_count);
  nh_->getParam(param_prefix_ + "tri_threshold_init_fail_count",
                config->tri_threshold_init_fail_count);

  std::dynamic_pointer_cast<GimbalVertexTestModule>(new_module)
      ->setConfig(config);
}
#endif

void ROSModuleFactory::configureLandmarkRecallModule(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<LandmarkRecallModule::Config> config;
  config.reset(new LandmarkRecallModule::Config());

  nh_->getParam(param_prefix_ + "landmark_source", config->landmark_source);
  nh_->getParam(param_prefix_ + "landmark_matches", config->landmark_matches);

  std::dynamic_pointer_cast<LandmarkRecallModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureWindowedRecallModule(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<WindowedRecallModule::Config> config;
  config.reset(new WindowedRecallModule::Config());

  nh_->getParam(param_prefix_ + "window_size", config->window_size);

  std::dynamic_pointer_cast<WindowedRecallModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureSubMapExtraction(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<SubMapExtractionModule::Config> config;
  config.reset(new SubMapExtractionModule::Config());

  nh_->getParam(param_prefix_ + "sigma_scale", config->sigma_scale);
  nh_->getParam(param_prefix_ + "temporal_min_depth",
                config->temporal_min_depth);
  nh_->getParam(param_prefix_ + "temporal_max_depth",
                config->temporal_max_depth);
  nh_->getParam(param_prefix_ + "search_spatially", config->search_spatially);
  nh_->getParam(param_prefix_ + "angle_weight", config->angle_weight);

  std::dynamic_pointer_cast<SubMapExtractionModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureLandmarkMigration(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<LandmarkMigrationModule::Config> config;
  config.reset(new LandmarkMigrationModule::Config());

  std::dynamic_pointer_cast<LandmarkMigrationModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureMelMatcher(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<MelMatcherModule::Config> config;
  config.reset(new MelMatcherModule::Config());

  nh_->getParam(param_prefix_ + "target_match_count",
                config->target_match_count);
  nh_->getParam(param_prefix_ + "min_match_count", config->min_match_count);
  nh_->getParam(param_prefix_ + "time_allowance", config->time_allowance);
  nh_->getParam(param_prefix_ + "matching_pixel_thresh",
                config->matching_pixel_thresh);
  nh_->getParam(param_prefix_ + "tight_matching_pixel_thresh",
                config->tight_matching_pixel_thresh);
  nh_->getParam(param_prefix_ + "tight_matching_x_sigma",
                config->tight_matching_x_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_y_sigma",
                config->tight_matching_y_sigma);
  nh_->getParam(param_prefix_ + "tight_matching_theta_sigma",
                config->tight_matching_theta_sigma);
  nh_->getParam(param_prefix_ + "min_response_ratio",
                config->min_response_ratio);
  nh_->getParam(param_prefix_ + "descriptor_thresh_cpu",
                config->descriptor_thresh_cpu);
  nh_->getParam(param_prefix_ + "descriptor_thresh_gpu",
                config->descriptor_thresh_gpu);
  nh_->getParam(param_prefix_ + "min_track_length", config->min_track_length);
  nh_->getParam(param_prefix_ + "max_landmark_depth",
                config->max_landmark_depth);
  nh_->getParam(param_prefix_ + "max_depth_diff", config->max_depth_diff);
  nh_->getParam(param_prefix_ + "visualize", config->visualize);
  nh_->getParam(param_prefix_ + "screen_matched_landmarks",
                config->screen_matched_landmarks);
  nh_->getParam(param_prefix_ + "parallel_threads", config->parallel_threads);
  nh_->getParam(param_prefix_ + "match_on_gpu", config->match_on_gpu);
  nh_->getParam(param_prefix_ + "match_gpu_knn_match_num",
                config->match_gpu_knn_match_num);

  std::dynamic_pointer_cast<MelMatcherModule>(new_module)->setConfig(config);
}

#if 0
void ROSModuleFactory::configureMelRecog(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<MelRecognitionModule::Config> config;
  config.reset(new MelRecognitionModule::Config());

  nh_->getParam(param_prefix_ + "temporal_depth", config->temporal_depth);
  nh_->getParam(param_prefix_ + "verbose", config->verbose);
  nh_->getParam(param_prefix_ + "sliding_window", config->sliding_window);
  nh_->getParam(param_prefix_ + "cluster_size", config->cluster_size);
  nh_->getParam(param_prefix_ + "compare_octave", config->compare_octave);
  nh_->getParam(param_prefix_ + "compare_laplacian", config->compare_laplacian);
  nh_->getParam(param_prefix_ + "num_desired_experiences",
                config->num_desired_experiences);
  nh_->getParam(param_prefix_ + "in_the_loop", config->in_the_loop);

  std::dynamic_pointer_cast<MelRecognitionModule>(new_module)
      ->setConfig(config);
}
#endif

void ROSModuleFactory::configureTodRecog(
    std::shared_ptr<BaseModule> &new_module) const {
  TodRecognitionModule::Config config;

  nh_->getParam(param_prefix_ + "verbose", config.verbose);
  nh_->getParam(param_prefix_ + "num_desired_experiences", config.num_exp);
  nh_->getParam(param_prefix_ + "in_the_loop", config.in_the_loop);
  nh_->getParam(param_prefix_ + "time_of_day_weight",
                config.time_of_day_weight);
  nh_->getParam(param_prefix_ + "total_time_weight", config.total_time_weight);

  std::dynamic_pointer_cast<TodRecognitionModule>(new_module)
      ->setConfig(std::move(config));
}

void ROSModuleFactory::configureCollabLandmarks(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<CollaborativeLandmarksModule::Config> config;
  config.reset(new CollaborativeLandmarksModule::Config());

  nh_->getParam(param_prefix_ + "verbose", config->verbose);
  nh_->getParam(param_prefix_ + "num_desired_experiences", config->num_exp);
  nh_->getParam(param_prefix_ + "in_the_loop", config->in_the_loop);
  nh_->getParam(param_prefix_ + "similarity_decay", config->similarity_decay);
  nh_->getParam(param_prefix_ + "prediction_decay", config->prediction_decay);
  nh_->param(param_prefix_ + "recommend_landmarks", config->recommend_landmarks,
             config->recommend_landmarks);

  std::dynamic_pointer_cast<CollaborativeLandmarksModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureRandomExperiences(
    std::shared_ptr<BaseModule> &new_module) const {
  RandomExperiencesModule::Config config;

  nh_->param(param_prefix_ + "verbose", config.verbose, false);
  nh_->param(param_prefix_ + "in_the_loop", config.in_the_loop, true);
  nh_->getParam(param_prefix_ + "num_desired_experiences", config.num_exp);

  std::dynamic_pointer_cast<RandomExperiencesModule>(new_module)
      ->setConfig(std::move(config));
}

void ROSModuleFactory::configureExperienceTriage(
    std::shared_ptr<BaseModule> &new_module) const {
  ExperienceTriageModule::Config config;

  nh_->param(param_prefix_ + "verbose", config.verbose, false);
  nh_->param(param_prefix_ + "always_privileged", config.always_privileged,
             true);
  nh_->param(param_prefix_ + "in_the_loop", config.in_the_loop, true);

  std::dynamic_pointer_cast<ExperienceTriageModule>(new_module)
      ->setConfig(std::move(config));
}

void ROSModuleFactory::configureResults(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<ResultsModule::Config> config;
  config.reset(new ResultsModule::Config());

  nh_->getParam(param_prefix_ + "directory", config->directory);

  std::dynamic_pointer_cast<ResultsModule>(new_module)->setConfig(config);
}

#if 0
void ROSModuleFactory::configureMonoScaling(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<MonoPlanarScalingModule::Config> config;
  config.reset(new MonoPlanarScalingModule::Config());

  // Mono Scaling Module Config
  nh_->getParam(param_prefix_ + "base_height", config->base_height);
  nh_->getParam(param_prefix_ + "init_distance", config->init_distance);
  nh_->getParam(param_prefix_ + "use_position_as_lla",
                config->use_position_as_lla);

  std::dynamic_pointer_cast<MonoPlanarScalingModule>(new_module)
      ->setConfig(config);
}
void ROSModuleFactory::configureUnderfootSeparate(
    std::shared_ptr<BaseModule> &new_module) const {
  UnderfootSeparateModule::Config config;

  config.node_handle = const_cast<ros::NodeHandle *>(nh_);
  config.param_prefix = param_prefix_;

  nh_->getParam(param_prefix_ + "/underfoot_distance",
                config.underfoot_distance);
  nh_->getParam(param_prefix_ + "/occlusion_distance",
                config.occlusion_distance);
  nh_->getParam(param_prefix_ + "/max_vertices", config.max_vertices);
  nh_->getParam(param_prefix_ + "/underfoot_set_skip",
                config.underfoot_set_skip);
  nh_->getParam(param_prefix_ + "/obstacle_button_idx",
                config.obstacle_button_idx);
  nh_->getParam(param_prefix_ + "/verbose", config.verbose);

  config.patch_gen_params =
      std::make_shared<terrain_assessment::PatchGenParams>();
  nh_->getParam(param_prefix_ + "/patch_gen_params/num_cells",
                config.patch_gen_params->num_cells);
  nh_->getParam(param_prefix_ + "/patch_gen_params/cell_sizes",
                config.patch_gen_params->cell_sizes);
  nh_->getParam(param_prefix_ + "/patch_gen_params/points_per_cell",
                config.patch_gen_params->points_per_cell);
  nh_->getParam(param_prefix_ + "/patch_gen_params/imagelike_pointcloud",
                config.patch_gen_params->imagelike_pointcloud);

  std::dynamic_pointer_cast<UnderfootSeparateModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureUnderfootAggregate(
    std::shared_ptr<BaseModule> &new_module) const {
  UnderfootAggregateModule::Config config;

  config.node_handle = const_cast<ros::NodeHandle *>(nh_);
  config.param_prefix = param_prefix_;

  nh_->getParam(param_prefix_ + "/underfoot_distance",
                config.underfoot_distance);
  nh_->getParam(param_prefix_ + "/occlusion_distance",
                config.occlusion_distance);
  nh_->getParam(param_prefix_ + "/max_vertices", config.max_vertices);
  nh_->getParam(param_prefix_ + "/underfoot_set_skip",
                config.underfoot_set_skip);
  nh_->getParam(param_prefix_ + "/obstacle_button_idx",
                config.obstacle_button_idx);
  nh_->getParam(param_prefix_ + "/verbose", config.verbose);

  config.patch_gen_params =
      std::make_shared<terrain_assessment::PatchGenParams>();
  nh_->getParam(param_prefix_ + "/patch_gen_params/num_cells",
                config.patch_gen_params->num_cells);
  nh_->getParam(param_prefix_ + "/patch_gen_params/cell_sizes",
                config.patch_gen_params->cell_sizes);
  nh_->getParam(param_prefix_ + "/patch_gen_params/points_per_cell",
                config.patch_gen_params->points_per_cell);
  nh_->getParam(param_prefix_ + "/patch_gen_params/imagelike_pointcloud",
                config.patch_gen_params->imagelike_pointcloud);

  std::dynamic_pointer_cast<UnderfootAggregateModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureQuickVORosPublisher(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<QuickVORosPublisherModule::Config> config;
  config.reset(new QuickVORosPublisherModule::Config());

  nh_->getParam(param_prefix_ + "use_position_as_lla",
                config->use_position_as_lla);
  nh_->getParam(param_prefix_ + "line_strip_scale", config->line_strip_scale);
  nh_->getParam(param_prefix_ + "robot_base_frame", config->robot_base_frame);

  std::dynamic_pointer_cast<QuickVORosPublisherModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureRefinedVORosPublisher(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<RefinedVORosPublisherModule::Config> config;
  config.reset(new RefinedVORosPublisherModule::Config());

  nh_->getParam(param_prefix_ + "use_position_as_lla",
                config->use_position_as_lla);
  nh_->getParam(param_prefix_ + "line_strip_scale", config->line_strip_scale);

  std::dynamic_pointer_cast<RefinedVORosPublisherModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureLocalizationRosPublisher(
    std::shared_ptr<BaseModule> &new_module) const {
  std::shared_ptr<LocalizationRosPublisherModule::Config> config;
  config.reset(new LocalizationRosPublisherModule::Config());

  nh_->getParam(param_prefix_ + "use_position_as_lla",
                config->use_position_as_lla);
  nh_->getParam(param_prefix_ + "line_strip_scale", config->line_strip_scale);

  std::dynamic_pointer_cast<LocalizationRosPublisherModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureLookaheadPatchGeneration(
    std::shared_ptr<BaseModule> &new_module) const {
  LookaheadPatchGenerationModule::Config config;

  config.node_handle = const_cast<ros::NodeHandle *>(nh_);
  config.param_prefix = param_prefix_;

  nh_->getParam(param_prefix_ + "/max_fps", config.max_fps);
  nh_->getParam(param_prefix_ + "/lookahead_distance",
                config.lookahead_distance);
  nh_->getParam(param_prefix_ + "/occlusion_distance",
                config.occlusion_distance);
  nh_->getParam(param_prefix_ + "/max_vertices", config.max_lookahead_vertices);
  nh_->getParam(param_prefix_ + "/lookahead_set_skip",
                config.lookahead_set_skip);
  nh_->getParam(param_prefix_ + "/save_to_disk", config.save_to_disk);
  nh_->getParam(param_prefix_ + "/verbose", config.verbose);

  config.patch_gen_params =
      std::make_shared<terrain_assessment::PatchGenParams>();
  nh_->getParam(param_prefix_ + "/patch_gen_params/num_cells",
                config.patch_gen_params->num_cells);
  nh_->getParam(param_prefix_ + "/patch_gen_params/cell_sizes",
                config.patch_gen_params->cell_sizes);
  nh_->getParam(param_prefix_ + "/patch_gen_params/points_per_cell",
                config.patch_gen_params->points_per_cell);
  nh_->getParam(param_prefix_ + "/patch_gen_params/imagelike_pointcloud",
                config.patch_gen_params->imagelike_pointcloud);

  nh_->getParam(param_prefix_ + "/matched_feature_mask/enabled",
                config.matched_feature_mask_config.enabled);
  nh_->getParam(param_prefix_ + "/matched_feature_mask/surf_base_window",
                config.matched_feature_mask_config.surf_base_window);

  std::dynamic_pointer_cast<LookaheadPatchGenerationModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureTraining(
    std::shared_ptr<BaseModule> &new_module) const {
  TrainingModule::Config config;

  nh_->getParam(param_prefix_ + "/training_distance", config.training_distance);
  nh_->getParam(param_prefix_ + "/max_vertices", config.max_training_vertices);
  nh_->getParam(param_prefix_ + "/training_runs", config.n_training_runs);
  nh_->getParam(param_prefix_ + "/min_cell_fraction", config.min_cell_fraction);

  std::dynamic_pointer_cast<TrainingModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureGpcTraining(
    std::shared_ptr<BaseModule> &new_module) const {
  GpcTrainingModule::Config config;

  std::dynamic_pointer_cast<GpcTrainingModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureChangeDetection(
    std::shared_ptr<BaseModule> &new_module) const {
  ChangeDetectionModule::Config config;

  config.node_handle = const_cast<ros::NodeHandle *>(nh_);
  config.param_prefix = param_prefix_;

  nh_->getParam(param_prefix_ + "/max_fps", config.max_fps);
  nh_->getParam(param_prefix_ + "/max_diff", config.max_label);
  nh_->getParam(param_prefix_ + "/n_consec_inf", config.n_consec_inf);
  nh_->getParam(param_prefix_ + "/slow_distance", config.slow_distance);
  nh_->getParam(param_prefix_ + "/stop_distance", config.stop_distance);

  nh_->getParam(param_prefix_ + "/lookahead/lookahead_distance",
                config.lookahead_distance);
  nh_->getParam(param_prefix_ + "/lookahead/occlusion_distance",
                config.occlusion_distance);
  nh_->getParam(param_prefix_ + "/lookahead/max_vertices",
                config.max_lookahead_vertices);
  nh_->getParam(param_prefix_ + "/lookahead/lookahead_set_skip",
                config.lookahead_set_skip);

  nh_->getParam(param_prefix_ + "/training/training_distance",
                config.training_distance);
  nh_->getParam(param_prefix_ + "/training/max_vertices",
                config.max_training_vertices);
  nh_->getParam(param_prefix_ + "/training/training_runs",
                config.n_training_runs);
  nh_->getParam(param_prefix_ + "/training/min_cell_fraction",
                config.min_cell_fraction);

  std::string smoothing_type;
  nh_->getParam(param_prefix_ + "/smoothing_type", smoothing_type);
  if (smoothing_type == "none") {
    config.smoothing_config.smoothing_type =
        terrain_assessment::SmoothingType::None;
  } else if (smoothing_type == "median") {
    config.smoothing_config.smoothing_type =
        terrain_assessment::SmoothingType::Median;
  } else if (smoothing_type == "connected_component") {
    config.smoothing_config.smoothing_type =
        terrain_assessment::SmoothingType::ConnectedComponent;
  }
  nh_->param<int>(param_prefix_ + "/nth_best", config.smoothing_config.nth_best,
                  0);
  nh_->param<int>(param_prefix_ + "/n_components",
                  config.smoothing_config.n_components, 0);

  config.patch_gen_params =
      std::make_shared<terrain_assessment::PatchGenParams>();
  nh_->getParam(param_prefix_ + "/patch_gen_params/num_cells",
                config.patch_gen_params->num_cells);
  nh_->getParam(param_prefix_ + "/patch_gen_params/cell_sizes",
                config.patch_gen_params->cell_sizes);
  nh_->getParam(param_prefix_ + "/patch_gen_params/points_per_cell",
                config.patch_gen_params->points_per_cell);
  nh_->getParam(param_prefix_ + "/patch_gen_params/imagelike_pointcloud",
                config.patch_gen_params->imagelike_pointcloud);

  std::dynamic_pointer_cast<ChangeDetectionModule>(new_module)
      ->setConfig(config);
}

void ROSModuleFactory::configureCDMaxMin(
    std::shared_ptr<BaseModule> &new_module) const {
  configureChangeDetection(new_module);
}

void ROSModuleFactory::configureCDMinMax(
    std::shared_ptr<BaseModule> &new_module) const {
  configureChangeDetection(new_module);
}

void ROSModuleFactory::configureCDGmm(
    std::shared_ptr<BaseModule> &new_module) const {
  configureChangeDetection(new_module);

  CDGmmModule::Config config;

  config.dpgmm_config = std::make_shared<terrain_assessment::DPGmm::Config>();
  nh_->getParam(param_prefix_ + "/gmm/k", config.dpgmm_config->base_config->K);
  nh_->getParam(param_prefix_ + "/gmm/max_iter", config.dpgmm_config->max_iter);
  nh_->getParam(param_prefix_ + "/gmm/tol", config.dpgmm_config->tol);

  std::dynamic_pointer_cast<CDGmmModule>(new_module)->setConfig(config);
}

void ROSModuleFactory::configureCDGpc(
    std::shared_ptr<BaseModule> &new_module) const {
  configureChangeDetection(new_module);

  CDGpcModule::Config config;

  nh_->getParam(param_prefix_ + "/gp/mean", config.mean);
  nh_->getParam(param_prefix_ + "/gp/l", config.l);
  nh_->getParam(param_prefix_ + "/gp/sf", config.sf);

  std::dynamic_pointer_cast<CDGpcModule>(new_module)->setConfig(config);
}
#endif
#endif
}  // namespace navigation
}  // namespace vtr