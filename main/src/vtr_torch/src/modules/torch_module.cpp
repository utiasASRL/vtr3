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

  config->model_filepath = node->declare_parameter<std::string>(param_prefix + ".filepath", config->model_filepath);
  // clang-format on
  return config;
}

TorchModule::~TorchModule() {}

// template <typename DataType>
// std::vector<DataType> TorchModule::evaluateModel(std::vector<DataType> inputs){
//     torch::NoGradGuard no_grad;
//     std::vector<torch::jit::IValue> jit_inputs;
//     jit_inputs.reserve(inputs.size());

//     for (auto& val : inputs)
//         jit_inputs.push_back(val);

//     auto output = network.forward(jit_inputs).toTensor();
//     std::cout << output;
// }


}  // namespace nn
}  // namespace vtr