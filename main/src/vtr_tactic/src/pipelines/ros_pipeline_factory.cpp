#include <vtr_tactic/pipelines/ros_pipeline_factory.hpp>

namespace vtr {
namespace tactic {

BasePipeline::Ptr ROSPipelineFactory::make(
    const std::string &param_prefix) const {
  std::string param_name{param_prefix + "." + type_field_};
  auto type_str = node_->declare_parameter<std::string>(param_name, "");
  if (type_str.empty()) {
    auto msg = "No field: '" + param_name + "' in the parameter list.";
    LOG(ERROR) << msg;
    throw std::runtime_error(msg);
  }
  auto pipeline = PipelineFactory::make(type_str);
  pipeline->configFromROS(node_, param_prefix);
  return pipeline;
}

}  // namespace tactic
}  // namespace vtr