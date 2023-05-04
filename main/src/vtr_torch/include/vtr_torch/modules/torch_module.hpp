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
 * \file torch_module.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

//#define C10_UTIL_LOGGING_IS_NOT_GOOGLE_GLOG_H_
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_tactic/task_queue.hpp"
#include <torch/torch.h>
#include <torch/script.h> 
#include "vtr_torch/types.hpp"
#include <vector>

namespace vtr {
namespace nn {

/** \brief Load and store Torch Models */
class TorchModule : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(TorchModule);    
  /** \brief Static module identifier. */
  static constexpr auto static_name = "torch";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    std::string model_filepath = "default";
    bool use_gpu = false;
    bool abs_filepath = true;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
    
    virtual ~Config() = default;  // for polymorphism

  };

  TorchModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {

        //Load the model
        //Should the system crash out if the model is loaded incorrectly?
        try {
          // Deserialize the ScriptModule from a file using torch::jit::load().
          network = torch::jit::load(config_->model_filepath);
        } catch (const c10::Error& e) {
          CLOG(ERROR, "torch") << "error loading the model\n" << "Tried to load " << config_->model_filepath;
        }

        if (config_->use_gpu && torch::cuda::is_available()){
          device = torch::kCUDA;
          network.to(device);
        }
        CLOG(INFO, "torch") << "Using device " << device << std::endl;

        //Copy the model weights into the graph folder for saving?

      }

  
  virtual ~TorchModule();

 private:
  virtual void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) = 0;

  Config::ConstPtr config_;
  torch::Device device = torch::kCPU;


 protected:
  Module network;

  template <typename DataType>
  torch::Tensor evaluateModel(std::vector<DataType> inputs, const Shape shape);

};

}  // namespace nn
}  // namespace vtr

#include "torch_module.inl"