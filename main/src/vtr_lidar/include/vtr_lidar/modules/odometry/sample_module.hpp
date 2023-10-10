// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file sample_module.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"
#include <vtr_torch/modules/torch_module.hpp>
#include "vtr_lidar/cache.hpp"
#include <vector>

namespace vtr {
namespace lidar {

/** \brief Load and store Torch Models */
class TestNNModule : public nn::TorchModule {
 public:
  PTR_TYPEDEFS(TestNNModule);    
  /** \brief Static module identifier. */
  static constexpr auto static_name = "torch.sample";

  /** \brief Config parameters. */
  struct Config : public nn::TorchModule::Config {
    PTR_TYPEDEFS(Config);

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  TestNNModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : nn::TorchModule{config, module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(TestNNModule);

};

}  // namespace nn
}  // namespace vtr