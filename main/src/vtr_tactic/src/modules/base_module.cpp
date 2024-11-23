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
 * \file base_module.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/modules/base_module.hpp"

namespace vtr {
namespace tactic {

BaseModule::BaseModule(const std::shared_ptr<ModuleFactory> &module_factory,
                       const std::string &name)
    : module_factory_{module_factory}, name_{name} {}

BaseModule::~BaseModule() {
  CLOG(DEBUG, "tactic.module")
      << "\033[1;31mSummarizing module: " << name()
      << ", count: " << count_.load() << ", time: " << timer_
      << ", time(ms)/count: "
      << (count_.load() > 0 ? (double)timer_.count() / (double)count_.load()
                            : 0) / 1e6
      << "\033[0m";
}

void BaseModule::run(QueryCache &qdata, OutputCache &output,
                     const Graph::Ptr &graph,
                     const std::shared_ptr<TaskExecutor> &executor) {
  CLOG(DEBUG, "tactic.module")
      << "\033[1;31mRunning module: " << name() << "\033[0m";
  common::timing::Stopwatch timer;
  common::timing::Stopwatch<boost::chrono::thread_clock> thread_timer;
  ++count_;
  timer_.start();
  run_(qdata, output, graph, executor);
  timer_.stop();
  CLOG(DEBUG, "tactic.module")
      << "Finished running module: " << name() << ", which takes "
      << thread_timer << " / " << timer;
}

void BaseModule::runAsync(QueryCache &qdata, OutputCache &output,
                          const Graph::Ptr &graph,
                          const std::shared_ptr<TaskExecutor> &executor,
                          const size_t &priority,
                          const boost::uuids::uuid &dep_id) {
  CLOG(DEBUG, "tactic.module")
      << "\033[1;31mRunning module (async): " << name() << "\033[0m";
  common::timing::Stopwatch timer;
  common::timing::Stopwatch<boost::chrono::thread_clock> thread_timer;
  timer_.start();
  runAsync_(qdata, output, graph, executor, priority, dep_id);
  timer_.stop();
  CLOG(DEBUG, "tactic.module")
      << "Finished running module (async): " << name() << ", which takes "
      << thread_timer << " / " << timer;
}

std::shared_ptr<ModuleFactory> BaseModule::factory() const {
  if (auto module_factory_acquired = module_factory_.lock())
    return module_factory_acquired;
  else {
    std::string err{"Module factory has expired"};
    CLOG(ERROR, "tactic.module") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

}  // namespace tactic
}  // namespace vtr