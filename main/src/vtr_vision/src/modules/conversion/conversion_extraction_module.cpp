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
 * \file conversion_extraction_module.cpp
 * \brief ConversionExtractionModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/features/extractor/feature_extractor_factory.hpp>
#include <vtr_vision/image_conversions.hpp>
#include <vtr_vision/modules/conversion/conversion_extraction_module.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

namespace {

void configureORBDetector(const rclcpp::Node::SharedPtr &node,
                          vision::ORBConfiguration &config,
                          const std::string &param_prefix) {
  // clang-format off
  config.use_STAR_detector_  = node->declare_parameter<bool>(param_prefix + ".extractor.orb.use_STAR_detector", true);
  config.use_GPU_descriptors_  = node->declare_parameter<bool>(param_prefix + ".extractor.orb.use_GPU_descriptors", false);
  config.STAR_maxSize_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.STAR_maxSize", 5);
  config.STAR_responseThreshold_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.STAR_responseThreshold", 10);
  config.STAR_lineThresholdProjected_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.STAR_lineThresholdProjected", 10);
  config.STAR_lineThresholdBinarized_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.STAR_lineThresholdBinarized", 8);
  config.STAR_suppressNonmaxSize_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.STAR_suppressNonmaxSize", 5);
  config.num_detector_features_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.num_detector_features", 15000);
  config.num_binned_features_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.num_binned_features", 2000);
  config.scaleFactor_  = node->declare_parameter<double>(param_prefix + ".extractor.orb.scaleFactor", 2.0);
  config.nlevels_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.nlevels", 4);
  config.edgeThreshold_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.edgeThreshold", 8);
  config.firstLevel_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.firstLevel", 0);
  config.WTA_K_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.WTA_K", 2);
  config.upright_  = node->declare_parameter<bool>(param_prefix + ".extractor.orb.upright_flag", false);
  config.num_threads_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.num_threads", 8);

  auto scoreType = node->declare_parameter<std::string>(param_prefix + ".extractor.orb.scoreType", "HARRIS");
  if (scoreType == "HARRIS")
    config.scoreType_ = cv::ORB::HARRIS_SCORE;
  else if (scoreType == "FAST")
    config.scoreType_ = cv::ORB::FAST_SCORE;

  config.patchSize_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.patchSize", 64); // \todo: 64 gives an error in cuda::ORB, max 59
  config.fastThreshold_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.fastThreshold", 20);
  config.x_bins_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.x_bins", 6);
  config.y_bins_  = node->declare_parameter<int>(param_prefix + ".extractor.orb.y_bins", 4);
  config.stereo_matcher_config_.descriptor_match_thresh_ = node->declare_parameter<double>(param_prefix + ".extractor.orb.matcher.descriptor_match_thresh", 0.55);
  config.stereo_matcher_config_.stereo_descriptor_match_thresh_ = node->declare_parameter<double>(param_prefix + ".extractor.orb.matcher.stereo_descriptor_match_thresh", 0.55);
  config.stereo_matcher_config_.stereo_y_tolerance_ = node->declare_parameter<double>(param_prefix + ".extractor.orb.matcher.stereo_y_tolerance", 2.0f);
  config.stereo_matcher_config_.stereo_x_tolerance_min_  = node->declare_parameter<double>(param_prefix + ".extractor.orb.matcher.stereo_x_tolerance_min", 0.0);
  config.stereo_matcher_config_.stereo_x_tolerance_max_  = node->declare_parameter<double>(param_prefix + ".extractor.orb.matcher.stereo_x_tolerance_max", 250.0);
  config.stereo_matcher_config_.check_octave_  = node->declare_parameter<bool>(param_prefix + ".extractor.orb.matcher.check_octave", true);
  config.stereo_matcher_config_.check_response_  = node->declare_parameter<bool>(param_prefix + ".extractor.orb.matcher.check_response", true);
  config.stereo_matcher_config_.min_response_ratio_  = node->declare_parameter<double>(param_prefix + ".extractor.orb.matcher.min_response_ratio", 0.2);
  config.stereo_matcher_config_.scale_x_tolerance_by_y_  = node->declare_parameter<bool>(param_prefix + ".extractor.orb.matcher.scale_x_tolerance_by_y", true);
  config.stereo_matcher_config_.x_tolerance_scale_ = node->declare_parameter<double>(param_prefix + ".extractor.orb.matcher.x_tolerance_scale", 768);
  // clang-format on
}

#ifdef VTR_ENABLE_GPUSURF
void configureSURFDetector(const rclcpp::Node::SharedPtr &node,
                           asrl::GpuSurfConfiguration &config,
                           const std::string &param_prefix) {
  // clang-format off
  config.threshold = node->declare_parameter<double>(param_prefix + ".extractor.surf.threshold", 1e-6);
  config.upright_flag = node->declare_parameter<bool>(param_prefix + ".extractor.surf.upright_flag", true);
#ifdef VTR_DETERMINISTIC
  LOG_IF(config.upright_flag, WARNING) << "SURF upright flag set to FALSE in deterministic mode.";
  config.upright_flag = false;
#endif
  config.nOctaves = node->declare_parameter<int>(param_prefix + ".extractor.surf.nOctaves", 4);
  config.nIntervals = node->declare_parameter<int>(param_prefix + ".extractor.surf.nIntervals", 4);
  config.initialScale = node->declare_parameter<double>(param_prefix + ".extractor.surf.initialScale", 1.5);
  config.edgeScale = node->declare_parameter<double>(param_prefix + ".extractor.surf.edgeScale", 1.5);
  config.l1 = node->declare_parameter<double>(param_prefix + ".extractor.surf.l1", 3.f / 1.5f);
  config.l2 = node->declare_parameter<double>(param_prefix + ".extractor.surf.l2", 5.f / 1.5f);
  config.l3 = node->declare_parameter<double>(param_prefix + ".extractor.surf.l3", 3.f / 1.5f);
  config.l4 = node->declare_parameter<double>(param_prefix + ".extractor.surf.l4", 1.f / 1.5f);
  config.initialStep = node->declare_parameter<int>(param_prefix + ".extractor.surf.initialStep", 1);
  config.targetFeatures = node->declare_parameter<int>(param_prefix + ".extractor.surf.targetFeatures", 600);
  config.detector_threads_x = node->declare_parameter<int>(param_prefix + ".extractor.surf.detector_threads_x", 16);
  config.detector_threads_y = node->declare_parameter<int>(param_prefix + ".extractor.surf.detector_threads_y", 16);
  config.nonmax_threads_x = node->declare_parameter<int>(param_prefix + ".extractor.surf.nonmax_threads_x", 16);
  config.nonmax_threads_y = node->declare_parameter<int>(param_prefix + ".extractor.surf.nonmax_threads_y", 16);
  config.regions_horizontal = node->declare_parameter<int>(param_prefix + ".extractor.surf.regions_horizontal", 16);
  config.regions_vertical = node->declare_parameter<int>(param_prefix + ".extractor.surf.regions_vertical", 16);
  config.regions_target = node->declare_parameter<int>(param_prefix + ".extractor.surf.regions_target", 600);
  // clang-format on
}

void configureSURFStereoDetector(const rclcpp::Node::SharedPtr &node,
                                 asrl::GpuSurfStereoConfiguration &config,
                                 const std::string &param_prefix) {
  // clang-format off
  config.stereoDisparityMinimum = node->declare_parameter<double>(param_prefix + ".extractor.surf.stereoDisparityMinimum", 0.0);
  config.stereoDisparityMaximum = node->declare_parameter<double>(param_prefix + ".extractor.surf.stereoDisparityMaximum", 64.0);
  config.stereoCorrelationThreshold = node->declare_parameter<double>(param_prefix + ".extractor.surf.stereoCorrelationThreshold", 0.79);
  config.stereoYTolerance = node->declare_parameter<double>(param_prefix + ".extractor.surf.stereoYTolerance", 0.5);
  config.stereoScaleTolerance = node->declare_parameter<double>(param_prefix + ".extractor.surf.stereoScaleTolerance", 0.9);
  // clang-format on
}
#endif

void configureLearnedFeatureDetector(const rclcpp::Node::SharedPtr &node,
                                     vision::LearnedFeatureConfiguration &config,
                                     const std::string &param_prefix) {
  // clang-format off
  config.model_path = node->declare_parameter<std::string>(param_prefix + ".extractor.learned.modelPath", "");
  // clang-format on
}

void configureLearnedFeatureStereoDetector(const rclcpp::Node::SharedPtr &node,
                                           vision::LearnedFeatureStereoConfiguration &config,
                                           const std::string &param_prefix) {
  // clang-format off
  config.stereoDisparityMinimum = node->declare_parameter<double>(param_prefix + ".extractor.learned.stereoDisparityMinimum", 0.0);
  config.stereoDisparityMaximum = node->declare_parameter<double>(param_prefix + ".extractor.learned.stereoDisparityMaximum", 100.0);
  // config.minDisparity = node->declare_parameter<int>(param_prefix + ".extractor.learned.minDisparity", 0);
  // config.numDisparities = node->declare_parameter<int>(param_prefix + ".extractor.learned.numDisparities", 48);
  // config.blockSize = node->declare_parameter<int>(param_prefix + ".extractor.learned.blockSize", 5);
  // config.preFilterCap = node->declare_parameter<int>(param_prefix + ".extractor.learned.preFilterCap", 30);
  // config.uniquenessRatio = node->declare_parameter<int>(param_prefix + ".extractor.learned.uniquenessRatio", 20);
  // config.P1 = node->declare_parameter<int>(param_prefix + ".extractor.learned.P1", 200);
  // config.P2 = node->declare_parameter<int>(param_prefix + ".extractor.learned.P2", 800);
  // config.speckleWindowSize = node->declare_parameter<int>(param_prefix + ".extractor.learned.speckleWindowSize", 200);
  // config.speckleRange = node->declare_parameter<int>(param_prefix + ".extractor.learned.speckleRange", 1);
  // config.disp12MaxDiff = node->declare_parameter<int>(param_prefix + ".extractor.learned.disp12MaxDiff", -1);
  // config.fullDP = node->declare_parameter<bool>(param_prefix + ".extractor.learned.fullDP", false);
  // config.minDisparity = 0;
  // config.numDisparities = 48;
  // config.blockSize = 5;
  // config.preFilterCap = 30;
  // config.uniquenessRatio = 20;
  // config.P1 = 200;
  // config.P2 = 800;
  // config.speckleWindowSize = 200;
  // config.speckleRange = 1;
  // config.disp12MaxDiff = -1;
  // config.fullDP = false;
  // clang-format on
}

}  // namespace

void ConversionExtractionModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->conversions = node->declare_parameter<std::vector<std::string>>(param_prefix + ".conversions", config_->conversions);
  config_->color_constant_weights = node->declare_parameter<std::vector<double>>(param_prefix + ".color_constant.weights", config_->color_constant_weights);
  config_->color_constant_histogram_equalization = node->declare_parameter<bool>(param_prefix + ".color_constant.histogram_equalization", config_->color_constant_histogram_equalization);
  config_->visualize_raw_features = node->declare_parameter<bool>(param_prefix + ".visualize_raw_features", config_->visualize_raw_features);
  config_->feature_type = node->declare_parameter<std::string>(param_prefix + ".extractor.type", config_->feature_type);
  config_->visualize_disparity = node->declare_parameter<bool>(param_prefix + ".extractor.visualize_disparity", config_->visualize_disparity);
  config_->use_learned = node->declare_parameter<bool>(param_prefix + ".extractor.use_learned", config_->use_learned);

  // configure the detector
  if (config_->feature_type == "OPENCV_ORB") {
    configureORBDetector(node, config_->opencv_orb_params, param_prefix);
  } else if (config_->feature_type == "ASRL_GPU_SURF") {
#ifdef VTR_ENABLE_GPUSURF
    configureSURFDetector(node, config_->gpu_surf_params, param_prefix);
    configureSURFStereoDetector(node, config_->gpu_surf_stereo_params, param_prefix);
    config_->gpu_surf_stereo_params.threshold = config_->gpu_surf_params.threshold;
    config_->gpu_surf_stereo_params.upright_flag = config_->gpu_surf_params.upright_flag;
    config_->gpu_surf_stereo_params.initialScale = config_->gpu_surf_params.initialScale;
    config_->gpu_surf_stereo_params.edgeScale = config_->gpu_surf_params.edgeScale;
    config_->gpu_surf_stereo_params.detector_threads_x = config_->gpu_surf_params.detector_threads_x;
    config_->gpu_surf_stereo_params.detector_threads_y = config_->gpu_surf_params.detector_threads_y;
    config_->gpu_surf_stereo_params.regions_horizontal = config_->gpu_surf_params.regions_horizontal;
    config_->gpu_surf_stereo_params.regions_vertical = config_->gpu_surf_params.regions_vertical;
    config_->gpu_surf_stereo_params.regions_target = config_->gpu_surf_params.regions_target;
#else
    throw std::runtime_error(
        "ROSModuleFactory::configureFeatureExtractor: GPU SURF isn't enabled!");
#endif
  } else {
    throw std::runtime_error(
        "Couldn't determine feature type when building ConversionExtraction "
        "Module!");
  }

  if (config_->use_learned) {
    configureLearnedFeatureDetector(node, config_->learned_feature_params, param_prefix);
    configureLearnedFeatureStereoDetector(node, config_->learned_feature_stereo_params, param_prefix);
  }

  // clang-format on
  createExtractor();
}

void ConversionExtractionModule::createExtractor() {
  extractor_ =
      vision::FeatureExtractorFactory::createExtractor(config_->feature_type);
  if (config_->feature_type == "ASRL_GPU_SURF") {
#ifdef VTR_ENABLE_GPUSURF
    vision::GpuSurfFeatureExtractor *dextractor =
        dynamic_cast<vision::GpuSurfFeatureExtractor *>(extractor_.get());
    dextractor->initialize(config_->gpu_surf_params);
    dextractor->initialize(config_->gpu_surf_stereo_params);
#else
    LOG(ERROR) << "GPU SURF isn't enabled!";
#endif
  } else if (config_->feature_type == "OPENCV_ORB") {
    vision::OrbFeatureExtractor *dextractor =
        dynamic_cast<vision::OrbFeatureExtractor *>(extractor_.get());
    dextractor->initialize(config_->opencv_orb_params);
  } else {
    LOG(ERROR) << "Couldn't determine feature type!";
  }

  if (config_->use_learned) {
    extractor_learned_ =
        vision::FeatureExtractorFactory::createExtractor("LEARNED_FEATURE");
    vision::LearnedFeatureExtractor *dextractor_learned =
      dynamic_cast<vision::LearnedFeatureExtractor *>(extractor_learned_.get());
    dextractor_learned->initialize(config_->learned_feature_params);
    dextractor_learned->initialize(config_->learned_feature_stereo_params);
  }
}

void ConversionExtractionModule::runImpl(QueryCache &qdata0,
                                         const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  // check if the required data is in this cache
  if (!qdata.rig_images.is_valid() || !qdata.rig_calibrations.is_valid())
    return;

  // Inputs, images
  auto &rigs = *qdata.rig_images;
  // Outputs, frames
  auto &rig_feature_list = qdata.rig_features.fallback();
  // Extra 
  auto &rig_extra_list = qdata.rig_extra.fallback();
  if (extractor_ == nullptr) {
    LOG(ERROR) << " Our extractor is null!";
    return;
  }
  if ((config_->use_learned) && (extractor_learned_ == nullptr)) {
    LOG(ERROR) << " Our learned extractor is null!";
    return;
  }

  for (auto &rig : rigs) {
    rig_feature_list->emplace_back(vision::RigFeatures());
    rig_extra_list->emplace_back(vision::RigExtra());
    auto num_input_channels = rig.channels.size();

    auto &rig_features = rig_feature_list->back();
    rig_features.name = rig.name;

    auto &rig_extra = rig_extra_list->back();
    rig_extra.name = rig.name;
    
    vision::ChannelFeatures (vision::BaseFeatureExtractor::*doit)(
        const vision::ChannelImages &, bool) =
        &vision::BaseFeatureExtractor::extractChannelFeatures;

    vision::ChannelFeatures (vision::BaseFeatureExtractor::*doit_learned)(
        const vision::ChannelImages &, const vision::ChannelImages &, bool) =
        &vision::BaseFeatureExtractor::extractChannelFeaturesDisp;

    // vision::ChannelExtra (vision::BaseFeatureExtractor::*doit_extra)(
    //     const vision::ChannelImages &) =
    //     &vision::BaseFeatureExtractor::extractChannelFeaturesExtra;

    for (unsigned channel_idx = 0; channel_idx < num_input_channels;
         ++channel_idx) {
      auto cc_weight_idx = 0;

      rig.channels[channel_idx].name = "RGB";

      std::vector<std::future<vision::ChannelFeatures>> feature_futures;

      // extract features on this channel. The extractor config selects if the
      // channel requires feature extraction, otherwise it inserts an empty set
      // of channel features

      // make the appropriate conversions.
      for (unsigned conversion_idx = 0;
           conversion_idx < config_->conversions.size(); ++conversion_idx) {
        const auto &input_channel = rig.channels[channel_idx];
        const auto &conversion = vision::StringToImageConversion(
            config_->conversions[conversion_idx]);
        // convert
        if (conversion == vision::ImageConversion::RGB_TO_GRAYSCALE) {
          rig.channels.emplace_back(vision::RGB2Grayscale(input_channel));
        } else if (conversion ==
                   vision::ImageConversion::RGB_TO_COLOR_CONSTANT) {
          // move the new channel onto the rig.
          rig.channels.emplace_back(vision::RGB2ColorConstant(
              rig.channels[channel_idx],
              config_->color_constant_weights[cc_weight_idx],
              config_->color_constant_histogram_equalization));
          cc_weight_idx++;    
        } else if (conversion == vision::ImageConversion::UNKNOWN) {
          throw std::runtime_error("ERROR: Image conversion " +
                                   config_->conversions[conversion_idx] +
                                   " unknown!");
        }

        // extract
        feature_futures.emplace_back(std::async(std::launch::async, doit,
                                                extractor_.get(),
                                                rig.channels.back(), true));
        
      }  // finish the conversionse


      if (config_->use_learned) {
        
        const auto &disp_channel = rig.channels[1]; // Grayscale
        rig.channels.emplace_back(vision::Disparity(disp_channel));

        // Get the dense descriptors and scores
        // vision::ChannelExtra extra = 
        //                         extractor_learned_->extractChannelFeaturesExtra(
        //                                              rig.channels[channel_idx]);

        // rig_extra.channels.emplace_back(extra);

        // Get keypoints features with associated info.
        // feature_futures.emplace(feature_futures.begin(),
        //                         std::async(std::launch::async, doit_learned,
        //                                    extractor_learned_.get(),
        //                                    rig.channels[channel_idx], 
        //                                    rig.channels.back(), extra, true));

        feature_futures.emplace(feature_futures.begin(),
                                std::async(std::launch::async, doit_learned,
                                           extractor_learned_.get(),
                                           rig.channels[channel_idx], 
                                           rig.channels.back(), true));


      }

      // get the futures.
      for (auto &future : feature_futures) {
        rig_features.channels.emplace_back(future.get());
      }
    }
  }
}

void ConversionExtractionModule::visualizeImpl(QueryCache &qdata0,
                                               const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);
  if (config_->visualize_raw_features)  // check if visualization is enabled
    visualize::showRawFeatures(*qdata.vis_mutex, qdata, " raw features");

  if (config_->visualize_disparity)  // check if visualization is enabled
    visualize::showDisparity(*qdata.vis_mutex, qdata, " disparity");
}

}  // namespace vision
}  // namespace vtr