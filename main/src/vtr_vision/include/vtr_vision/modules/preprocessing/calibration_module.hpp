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
#include <vtr_vision/types.hpp>

// #include <vtr_vision/features/extractor/base_feature_extractor.hpp>
// #include <vtr_vision/features/extractor/extractor_configs.hpp>
//#include <vtr_vision/visualize.hpp>

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
class CalibrationModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "stereo.adapter";
  PTR_TYPEDEFS(CalibrationModule);

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);

    CameraDistortion distortion;
    CameraIntrinsic intrinsic;
    Transform extrinsic;
    
  };


  CalibrationModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

  // ConversionExtractionModule(const std::string &name = static_name)
  //     : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  // void configFromROS(const rclcpp::Node::SharedPtr &node,
  //                    const std::string param_prefix) override;

 private:
  /**
   * \brief Uses multi-threading to perform image conversion (i.e. RGB to
   * grayscale and CC),and feature extraction in parallel for each rig, channel
   * and camera.
   */
  // void runImpl(tactic::QueryCache &qdata,
  //              const tactic::Graph::ConstPtr &) override;

    void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;


  /** \brief Module configuration. */
  Config::ConstPtr config_;


  VTR_REGISTER_MODULE_DEC_TYPE(CalibrationModule);

};

}  // namespace vision
}  // namespace vtr