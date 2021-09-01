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
 * \file simple_vertex_test_module.hpp
 * \brief SimpleVertexTestModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>
#include <vtr_vision/modules/miscellaneous/vertex_creation_test_module.hpp>

namespace vtr {
namespace vision {

/**
 * \brief A module that determines whether a new vertex should be created.
 * \details
 * requires:
 *   qdata.[ransac_matches, steam_failure, T_q_m]
 * outputs:
 *   qdata.[new_vertex_flag, success]
 */
class SimpleVertexTestModule : public VertexCreationModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "simple_vertex_creation_test";

  /** \brief Collection of config parameters */
  struct Config : VertexCreationModule::Config {
    double min_creation_distance;
    double max_creation_distance;
    double rotation_threshold_min;
    double rotation_threshold_max;
    double min_distance;
    int match_threshold_min_count;
    int match_threshold_fail_count;
  };

  SimpleVertexTestModule(std::string name = static_name)
      : VertexCreationModule{name} {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 protected:
  /**
   * \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

 private:
  /** \brief Module configuration. */
  std::shared_ptr<Config> simple_config_;
};

}  // namespace vision
}  // namespace vtr
