#include <torch/torch.h>
#include <torch/script.h> 
#include "vtr_torch/types.hpp"

using namespace torch::indexing;

namespace vtr {
namespace nn {

  template <typename DataType>
  torch::Tensor TorchModule::evaluateModel(std::vector<DataType> inputs, const Shape shape){
    torch::NoGradGuard no_grad;
    std::vector<torch::jit::IValue> jit_inputs;

    auto t_input = torch::from_blob(inputs.data(), shape).to(device);
    jit_inputs.push_back(t_input);


    auto output = network(jit_inputs);

    return output.toTensor().cpu();
  }


    
} // namespace nn 
} // namespace vtr