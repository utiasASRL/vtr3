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
 * \file image_triangulation_module.hpp
 * \brief ImageTriangulationModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_tactic/cache.hpp>
#include "vtr_tactic/task_queue.hpp"
#include <vtr_vision/types.hpp>


namespace vtr {
namespace vision {

/**
 * \brief A module that generates landmarks from image features. The landmark
 * point is 3D for stereo camera.
 * \details
 * requires: qdata.[rig_features, rig_calibrations]
 * outputs: qdata.[candidate_landmarks]
 *
 * This module converts stereo-matched features into landmarks with 3D points in
 * the first camera's frame. The landmarks are candidate as it has not been
 * matched to previous experiences.
 */
class ImageTriangulationModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. \todo change this to static_name */
  static constexpr auto static_name = "image_triangulation";
  PTR_TYPEDEFS(ImageTriangulationModule);

  /** \brief Config parameters */
  struct Config : public tactic::BaseModule::Config{
    PTR_TYPEDEFS(Config);
    bool visualize_features = true;
    bool visualize_stereo_features = true;
    bool visualize = false;
    float min_triangulation_depth = 0.01;
    float max_triangulation_depth = 500.0;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix);


  };

  ImageTriangulationModule(const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {
      }


 private:
  /**
   * \brief Generates landmarks from image features. The landmark point is 3D
   * for stereo camera.
   */

  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
              const std::shared_ptr<tactic::TaskExecutor> &executor) override;
              

  Config::ConstPtr config_;
  VTR_REGISTER_MODULE_DEC_TYPE(ImageTriangulationModule);

};

}  // namespace vision
}  // namespace vtr
