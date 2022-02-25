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
 * \file base_pipeline.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/pipelines/base_pipeline.hpp"

namespace vtr {
namespace tactic {

BasePipeline::BasePipeline(const std::shared_ptr<ModuleFactory> &module_factory,
                           const std::string &name)
    : module_factory_{module_factory}, name_{name} {}

OutputCache::Ptr BasePipeline::createOutputCache() const {
  return std::make_shared<OutputCache>();
}

void BasePipeline::initialize(const OutputCache::Ptr &output,
                              const Graph::Ptr &graph) {
  CLOG(DEBUG, "tactic.pipeline")
      << "\033[1;31mInitializing pipeline: " << name() << "\033[0m";
  common::timing::Stopwatch timer;
  initialize_(output, graph);
  CLOG(DEBUG, "tactic.pipeline") << "Finished initializing pipeline: " << name()
                                 << ", which takes " << timer;
}

void BasePipeline::preprocess(const QueryCache::Ptr &qdata,
                              const OutputCache::Ptr &output,
                              const Graph::Ptr &graph,
                              const std::shared_ptr<TaskExecutor> &executor) {
  CLOG(DEBUG, "tactic.pipeline")
      << "\033[1;31mStart preprocessing: " << name() << "\033[0m";
  common::timing::Stopwatch timer;
  preprocess_(qdata, output, graph, executor);
  CLOG(DEBUG, "tactic.pipeline")
      << "Finished preprocessing: " << name() << ", which takes " << timer;
}

void BasePipeline::runOdometry(const QueryCache::Ptr &qdata,
                               const OutputCache::Ptr &output,
                               const Graph::Ptr &graph,
                               const std::shared_ptr<TaskExecutor> &executor) {
  CLOG(DEBUG, "tactic.pipeline")
      << "\033[1;31mStart running odometry: " << name() << "\033[0m";
  common::timing::Stopwatch timer;
  runOdometry_(qdata, output, graph, executor);
  CLOG(DEBUG, "tactic.pipeline")
      << "Finished running odometry: " << name() << ", which takes " << timer;
}

void BasePipeline::runLocalization(
    const QueryCache::Ptr &qdata, const OutputCache::Ptr &output,
    const Graph::Ptr &graph, const std::shared_ptr<TaskExecutor> &executor) {
  CLOG(DEBUG, "tactic.pipeline")
      << "\033[1;31mStart running localization: " << name() << "\033[0m";
  common::timing::Stopwatch timer;
  runLocalization_(qdata, output, graph, executor);
  CLOG(DEBUG, "tactic.pipeline") << "Finished running localization: " << name()
                                 << ", which takes " << timer;
}

void BasePipeline::onVertexCreation(
    const QueryCache::Ptr &qdata, const OutputCache::Ptr &output,
    const Graph::Ptr &graph, const std::shared_ptr<TaskExecutor> &executor) {
  CLOG(DEBUG, "tactic.pipeline")
      << "\033[1;31mStart processing vertex: " << name() << "\033[0m";
  common::timing::Stopwatch timer;
  onVertexCreation_(qdata, output, graph, executor);
  CLOG(DEBUG, "tactic.pipeline")
      << "Finished processing vertex: " << name() << ", which takes " << timer;
}

std::shared_ptr<ModuleFactory> BasePipeline::factory() const {
  if (module_factory_ == nullptr)
    throw std::runtime_error{"Module factory is a nullptr."};
  return module_factory_;
}

}  // namespace tactic
}  // namespace vtr