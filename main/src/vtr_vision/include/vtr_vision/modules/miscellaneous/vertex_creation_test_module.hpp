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
 * \file vertex_creation_test_module.hpp
 * \brief VertexCreationModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>

namespace vtr {
namespace vision {

/** \brief Reject outliers and estimate a preliminary transform */
class VertexCreationModule : public tactic::BaseModule {
 public:
  /**
   * \brief Static module identifier.
   * \todo change this to static_name
   */
  static constexpr auto static_name = "vertex_creation";

  /** \brief Collection of config parameters */
  struct Config {
    double distance_threshold_min = 0.2;
    double distance_threshold_max = 1.0;
    double rotation_threshold_min = 2.0;
    double rotation_threshold_max = 20.0;
    int match_threshold_min_count = 100;
    int match_threshold_fail_count = 15;
  };

  VertexCreationModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()){};

 protected:
  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace vision
}  // namespace vtr
