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

namespace vtr {
namespace vision {

/**
 * \brief A module that determines whether a new vertex should be created.
 * \details
 * requires:
 *   qdata.[ransac_matches, T_q_m]
 * outputs:
 *   qdata.[new_vertex_flag, success]
 */
class SimpleVertexTestModule : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(SimpleVertexTestModule);

  /** \brief Static module identifier. */
  static constexpr auto static_name = "simple_vertex_creation_test";

  /** \brief Collection of config parameters */
  struct Config : tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);
    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                        const std::string &param_prefix);
    double min_distance = 0.05;
    double min_creation_distance = 0.2;
    double max_creation_distance = 1.0;
    double rotation_threshold_min = 2.0;
    double rotation_threshold_max = 20.0;
    int match_threshold_min_count = 100;
    int match_threshold_fail_count = 15;
  };

  SimpleVertexTestModule(const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}


 protected:
  /**
   * \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
    void run_(tactic::QueryCache &qdata, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) override;


 private:
  /** \brief Module configuration. */
  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(SimpleVertexTestModule);

};

}  // namespace vision
}  // namespace vtr
