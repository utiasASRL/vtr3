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
 * outputs: qdata.[rig_images, rig_features]
 *
 * This module stores the converted images (gray scaled, color constant) in
 * extra channels of each image of qdata.rig_images.
 * The features corresponding to each channel are stored in qdata.rig_features.
 * Only stereo matched features are stored.
 */
class ConversionExtractionModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "conversion_extraction";

  /** \brief Config parameters. */
  struct Config {
    std::string feature_type =
        "ASRL_GPU_SURF";  // ["OPENCV_ORB", "ASRL_GPU_SURF"]

    ORBConfiguration opencv_orb_params;
#ifdef VTR_ENABLE_GPUSURF
    asrl::GpuSurfConfiguration gpu_surf_params;
    asrl::GpuSurfStereoConfiguration gpu_surf_stereo_params;
#endif
    vision::LearnedFeatureConfiguration learned_feature_params;
    vision::LearnedFeatureStereoConfiguration learned_feature_stereo_params;

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
    bool visualize_raw_features = false;

    /** \brief Flag for displaying the disparity */
    bool visualize_disparity = false;

    /** \brief Flag for using the extractor for learned features */
    bool use_learned = false;
  };

  ConversionExtractionModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  /**
   * \brief Uses multi-threading to perform image conversion (i.e. RGB to
   * grayscale and CC),and feature extraction in parallel for each rig, channel
   * and camera.
   */
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &) override;

  /**
   * \brief Visualizes raw features that were extracted on all images and their
   * conversions.
   */
  void visualizeImpl(tactic::QueryCache &qdata,
                     const tactic::Graph::ConstPtr &) override;

  void createExtractor();

  std::shared_ptr<Config> config_;

  /** \brief Feature Extractor */
  std::shared_ptr<BaseFeatureExtractor> extractor_;
  std::shared_ptr<BaseFeatureExtractor> extractor_learned_;
};

}  // namespace vision
}  // namespace vtr
