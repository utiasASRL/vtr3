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

// Include PyTorch headers FIRST to avoid namespace conflicts with ROS/std
#ifdef VTR_VISION_LEARNED
#include <torch/script.h>
#include <torch/torch.h>
#endif

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
 * \brief A module that converts images from ROS to RigImages and loads the calibrations
 * from the config file
 * \details
 * requires: qdata.[left_image, right_image]
 * outputs: qdata.[rig_images, rig_calibrations, rig_names]
 *
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
    float baseline = 0.0;       //m
    std::string rig_name = "stereo";
    int target_width = -1;
    
  };


  CalibrationModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}


 private:
  /**
   * \brief Uses multi-threading to perform image conversion (i.e. RGB to
   * grayscale and CC),and feature extraction in parallel for each rig, channel
   * and camera.
   */
    void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;


  /** \brief Module configuration. */
  Config::ConstPtr config_;


  VTR_REGISTER_MODULE_DEC_TYPE(CalibrationModule);

};

}  // namespace vision
}  // namespace vtr