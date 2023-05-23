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
 * \file conversion_extraction_module.hpp
 * \brief ConversionExtractionModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_tactic/cache.hpp>
#include "vtr_tactic/task_queue.hpp"

#include <vtr_vision/features/extractor/base_feature_extractor.hpp>
#include <vtr_vision/features/extractor/extractor_configs.hpp>
#include <vtr_vision/visualize.hpp>

namespace vtr {
namespace vision {

/**
 * \brief A module that converts images from RGB to grayscale or other forms,
 * and extracts features using surf/orb in parallel.
 * \details
 * requires: qdata.[rig_images]
 * outputs: qdata.[rig_images, rig_features, rig_extra]
 *
 * This module stores the converted images (gray scaled, color constant) in
 * extra channels of each image of qdata.rig_images.
 * The features corresponding to each channel are stored in qdata.rig_features.
 * Only stereo matched features are stored.
 */
class ConversionExtractionModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "stereo.conversion_extraction";
  PTR_TYPEDEFS(ConversionExtractionModule);

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);
    std::string feature_type =
        "ASRL_GPU_SURF";  // ["OPENCV_ORB", "ASRL_GPU_SURF"]

    ORBConfiguration opencv_orb_params;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);

#ifdef VTR_ENABLE_GPUSURF
    asrl::GpuSurfConfiguration gpu_surf_params;
    asrl::GpuSurfStereoConfiguration gpu_surf_stereo_params;
#endif
#ifdef VTR_VISION_LEARNED
    vision::LearnedFeatureConfiguration learned_feature_params;
    vision::LearnedFeatureStereoConfiguration learned_feature_stereo_params;
#endif
    /** \brief The collection of user requested image conversions. */
    std::vector<std::string> conversions = {"RGB_TO_GRAYSCALE",
                                            "RGB_TO_COLOR_CONSTANT"};

    /**
     * \brief The collection of color constant weights.
     * \details The size of this vector must be greater or equal to the number
     * of requested color constant conversions.
     */
    std::vector<double> color_constant_weights = {0.43};

    /** \brief Histogram equalization flag for color constant transformation. */
    bool color_constant_histogram_equalization = false;

    /** \brief Flag for displaying the raw detected features */
    bool visualize_raw_features = true;

    /** \brief Flag for displaying the disparity */
    bool visualize_disparity = true;

    /** \brief Flag for using the extractor for learned features */
    bool use_learned = false;

    /** \brief Flag to determine whether feature matches should be displayed*/
    bool visualize = false;
  };


  ConversionExtractionModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {
        createExtractor();
      }
      
 private:
  /**
   * \brief Uses multi-threading to perform image conversion (i.e. RGB to
   * grayscale and CC),and feature extraction in parallel for each rig, channel
   * and camera.
   */
    void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;

  void createExtractor();

  /** \brief Module configuration. */
  Config::ConstPtr config_;

  /** \brief Feature Extractor */
  std::shared_ptr<BaseFeatureExtractor> extractor_;
  std::shared_ptr<BaseFeatureExtractor> extractor_learned_;

  VTR_REGISTER_MODULE_DEC_TYPE(ConversionExtractionModule);

};

}  // namespace vision
}  // namespace vtr