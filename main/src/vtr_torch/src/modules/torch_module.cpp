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
 * \file torch_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_torch/modules/torch_module.hpp"


namespace vtr {
namespace nn {

using namespace tactic;

auto TorchModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  config->use_gpu = node->declare_parameter<bool>(param_prefix + ".use_gpu", config->use_gpu);
  config->abs_filepath = node->declare_parameter<bool>(param_prefix + ".abs_filepath", config->abs_filepath);

  auto model_dir = node->declare_parameter<std::string>("model_dir", "defalut2");
  model_dir = common::utils::expand_user(common::utils::expand_env(model_dir));

  if (config->abs_filepath){
    config->model_filepath = node->declare_parameter<std::string>(param_prefix + ".filepath", config->model_filepath);
  } else {
    config->model_filepath = model_dir + "/" + node->declare_parameter<std::string>(param_prefix + ".filepath", config->model_filepath);
  }
  // clang-format on
  return config;
}

TorchModule::~TorchModule() {}


torch::Tensor TorchModule::evaluateModel(torch::Tensor input, const Shape shape) {
  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> jit_inputs;

  jit_inputs.push_back(input);


  auto output = network(jit_inputs);

  return output.toTensor().cpu();
}

}  // namespace nn
}  // namespace vtr