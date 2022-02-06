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
 * \file pipeline.cpp
 * \author Keenan Burnett, Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/pipeline.hpp"
#include "vtr_tactic/modules/factory.hpp"

namespace vtr {
namespace radar {

using namespace tactic;

auto RadarPipeline::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                    const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  config->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  config->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
  // clang-format on
  return config;
}

RadarPipeline::RadarPipeline(
    const Config::ConstPtr &config,
    const std::shared_ptr<ModuleFactory> &module_factory,
    const std::string &name)
    : BasePipeline(module_factory, name), config_(config) {
  // preprocessing
  for (auto module : config_->preprocessing)
    preprocessing_.push_back(factory()->get("preprocessing." + module));
  // odometry
  for (auto module : config_->odometry)
    odometry_.push_back(factory()->get("odometry." + module));
  // localization
  for (auto module : config_->localization)
    localization_.push_back(factory()->get("localization." + module));
}

OutputCache::Ptr RadarPipeline::createOutputCache() const {
  return std::make_shared<RadarOutputCache>();
}

void RadarPipeline::reset() {
  point_map_odo_ = nullptr;
  timestamp_odo_ = nullptr;
  T_r_pm_odo_ = nullptr;
  w_pm_r_in_r_odo_ = nullptr;
  new_scan_odo_.clear();
#if false
  new_raw_scan_odo_.clear();
#endif
  curr_map_loc_ = nullptr;
}

void RadarPipeline::preprocess_(const QueryCache::Ptr &qdata0,
                                const OutputCache::Ptr &output0,
                                const Graph::Ptr &graph,
                                const TaskExecutor::Ptr &executor) {
  for (auto module : preprocessing_)
    module->run(*qdata0, *output0, graph, executor);
}

void RadarPipeline::runOdometry_(const QueryCache::Ptr &qdata0,
                                 const OutputCache::Ptr &output0,
                                 const Graph::Ptr &graph,
                                 const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<RadarQueryCache>(qdata0);
  /// \todo implement
}

void RadarPipeline::runLocalization_(const QueryCache::Ptr &qdata0,
                                     const OutputCache::Ptr &output0,
                                     const Graph::Ptr &graph,
                                     const TaskExecutor::Ptr &executor) {
  auto qdata = std::dynamic_pointer_cast<RadarQueryCache>(qdata0);
  /// \todo implement
}

void RadarPipeline::processKeyframe_(const QueryCache::Ptr &qdata0,
                                     const OutputCache::Ptr &,
                                     const Graph::Ptr &graph,
                                     const TaskExecutor::Ptr &) {
  const auto qdata = std::dynamic_pointer_cast<RadarQueryCache>(qdata0);
  /// \todo implement
}

}  // namespace radar
}  // namespace vtr
