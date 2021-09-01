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
 * \file template_module.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/** \brief A tactic module template */
class TemplateModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "template";

  /** \brief Collection of config parameters */
  struct Config {
    std::string parameter = "default value";
  };

  TemplateModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override {
    /// Configure your module from ROS
    config_ = std::make_shared<Config>();
    // clang-format off
    config_->parameter = node->declare_parameter<std::string>(param_prefix + ".parameter", config_->parameter);
    // clang-format on
    CLOG(INFO, "tactic.module")
        << "Template module parameter set to: " << config_->parameter;
  }

 private:
  void runImpl(QueryCache &, const Graph::ConstPtr &) override {
    /// Pure virtual method that must be overriden.
    /// Do the actual work of your module. Load data from and store data to
    /// QueryCache.
    CLOG(INFO, "tactic.module") << "Running the template module...";
  }

  void updateGraphImpl(QueryCache &, const Graph::Ptr &, VertexId) override {
    /// Override this method if your module needs to store data into the graph.
    CLOG(INFO, "tactic.module")
        << "Template module is updating the pose graph...";
  }

  void visualizeImpl(QueryCache &, const Graph::ConstPtr &) override {
    /// Override this method if you module produces visualization. The mutex is
    /// for OpenCV.
    CLOG(INFO, "tactic.module") << "Template module is being visualized...";
  }

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;  /// \todo no need to be a shared pointer.
};

}  // namespace tactic
}  // namespace vtr