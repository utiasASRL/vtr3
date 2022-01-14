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
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_vision/modules/conversion/conversion_extraction_module.hpp"

#include "vtr_vision/features/extractor/feature_extractor_factory.hpp"
#include "vtr_vision/types.hpp"

namespace vtr {
namespace vision {

using namespace tactic;

namespace {

#ifdef VTR_ENABLE_GPUSURF
void configureSURFDetector(const rclcpp::Node::SharedPtr &node,
                           asrl::GpuSurfConfiguration &config,
                           const std::string &param_prefix) {
  // clang-format off
  config.threshold = node->declare_parameter<double>(param_prefix + ".extractor.surf.threshold", 1e-6);
  config.upright_flag = node->declare_parameter<bool>(param_prefix + ".extractor.surf.upright_flag", true);
#ifdef VTR_DETERMINISTIC
  CLOG_IF(config.upright_flag, WARNING, "vision.conversion_extraction") << "SURF upright flag set to FALSE in deterministic mode.";
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

}  // namespace

auto ConversionExtractionModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->conversions = node->declare_parameter<std::vector<std::string>>(param_prefix + ".conversions", config->conversions);
  config->color_constant_weights = node->declare_parameter<std::vector<double>>(param_prefix + ".color_constant.weights", config->color_constant_weights);
  config->color_constant_histogram_equalization = node->declare_parameter<bool>(param_prefix + ".color_constant.histogram_equalization", config->color_constant_histogram_equalization);
  config->visualize_raw_features = node->declare_parameter<bool>(param_prefix + ".visualize_raw_features", config->visualize_raw_features);
  config->feature_type = node->declare_parameter<std::string>(param_prefix + ".extractor.type", config->feature_type);
  // configure the detector
  if (config->feature_type == "ASRL_GPU_SURF") {
#ifdef VTR_ENABLE_GPUSURF
    configureSURFDetector(node, config->gpu_surf_params, param_prefix);
    configureSURFStereoDetector(node, config->gpu_surf_stereo_params, param_prefix);
    config->gpu_surf_stereo_params.threshold = config->gpu_surf_params.threshold;
    config->gpu_surf_stereo_params.upright_flag = config->gpu_surf_params.upright_flag;
    config->gpu_surf_stereo_params.initialScale = config->gpu_surf_params.initialScale;
    config->gpu_surf_stereo_params.edgeScale = config->gpu_surf_params.edgeScale;
    config->gpu_surf_stereo_params.detector_threads_x = config->gpu_surf_params.detector_threads_x;
    config->gpu_surf_stereo_params.detector_threads_y = config->gpu_surf_params.detector_threads_y;
    config->gpu_surf_stereo_params.regions_horizontal = config->gpu_surf_params.regions_horizontal;
    config->gpu_surf_stereo_params.regions_vertical = config->gpu_surf_params.regions_vertical;
    config->gpu_surf_stereo_params.regions_target = config->gpu_surf_params.regions_target;
#else
    throw std::runtime_error(
        "ROSModuleFactory::configureFeatureExtractor: GPU SURF isn't enabled!");
#endif
  } else {
    throw std::runtime_error(
        "Couldn't determine feature type when building ConversionExtraction "
        "Module!");
  }
  // clang-format on
}

void ConversionExtractionModule::createExtractor() {
  extractor_ = FeatureExtractorFactory::createExtractor(config_->feature_type);
  if (config_->feature_type == "ASRL_GPU_SURF") {
#ifdef VTR_ENABLE_GPUSURF
    GpuSurfFeatureExtractor *dextractor =
        dynamic_cast<GpuSurfFeatureExtractor *>(extractor_.get());
    dextractor->initialize(config_->gpu_surf_params);
    dextractor->initialize(config_->gpu_surf_stereo_params);
#else
    CLOG(ERROR, "vision.conversion_extraction") << "GPU SURF isn't enabled!";
    throw std::runtime_error("GPU SURF isn't enabled!");
#endif
  } else {
    CLOG(ERROR, "vision.conversion_extraction")
        << "Couldn't determine feature type!";
    throw std::runtime_error("Couldn't determine feature type!");
  }
}

void ConversionExtractionModule::runImpl(QueryCache &qdata0, OutputCache &,
                                         const Graph::Ptr &,
                                         const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<VisionQueryCache &>(qdata0);

  // check if the required data is in this cache
  if (!qdata.rig_images.valid() || !qdata.rig_calibrations.valid()) return;

  // Inputs, images
  auto &rigs = *qdata.rig_images;
  // Outputs, frames
  auto &rig_feature_list = qdata.rig_features.emplace();
  if (extractor_ == nullptr) {
    CLOG(ERROR, "vision.conversion_extraction") << " Our extractor is null!";
    throw std::runtime_error("Extractor is null!");
  }
  for (auto &rig : rigs) {
    rig_feature_list->emplace_back(RigFeatures());
    auto num_input_channels = rig.channels.size();

    auto &rig_features = rig_feature_list->back();
    rig_features.name = rig.name;
    ChannelFeatures (BaseFeatureExtractor::*doit)(const ChannelImages &, bool) =
        &BaseFeatureExtractor::extractChannelFeatures;
    for (unsigned channel_idx = 0; channel_idx < num_input_channels;
         ++channel_idx) {
      auto cc_weight_idx = 0;
      std::vector<std::future<ChannelFeatures>> feature_futures;
      // extract features on this channel. The extractor config selects if the
      // channel requires feature extraction, otherwise it inserts an empty set
      // of channel features
      feature_futures.emplace_back(std::async(std::launch::async, doit,
                                              extractor_.get(),
                                              rig.channels[channel_idx], true));
      // make the appropriate conversions.
      for (unsigned conversion_idx = 0;
           conversion_idx < config_->conversions.size(); ++conversion_idx) {
        const auto &input_channel = rig.channels[channel_idx];
        const auto &conversion =
            StringToImageConversion(config_->conversions[conversion_idx]);
        // convert
        if (conversion == ImageConversion::RGB_TO_GRAYSCALE) {
          rig.channels.emplace_back(RGB2Grayscale(input_channel));
        } else if (conversion == ImageConversion::RGB_TO_COLOR_CONSTANT) {
          // move the new channel onto the rig.
          rig.channels.emplace_back(RGB2ColorConstant(
              rig.channels[channel_idx],
              config_->color_constant_weights[cc_weight_idx],
              config_->color_constant_histogram_equalization));
          cc_weight_idx++;
        } else if (conversion == ImageConversion::UNKNOWN) {
          throw std::runtime_error("Image conversion " +
                                   config_->conversions[conversion_idx] +
                                   " unknown!");
        }

        // extract
        feature_futures.emplace_back(std::async(std::launch::async, doit,
                                                extractor_.get(),
                                                rig.channels.back(), true));
      }  // finish the conversions

      // get the futures.
      for (auto &future : feature_futures) {
        rig_features.channels.emplace_back(future.get());
      }
    }
  }

  /// \todo port visualization functions
#if false
  if (config_->visualize_raw_features)  // check if visualization is enabled
    showRawFeatures(*qdata.vis_mutex, qdata, " raw features");
#endif
}

}  // namespace vision
}  // namespace vtr