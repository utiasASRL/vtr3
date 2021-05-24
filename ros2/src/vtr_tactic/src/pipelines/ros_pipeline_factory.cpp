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
  configurePipeline(pipeline, type_str, param_prefix);
  return pipeline;
}

void ROSPipelineFactory::configurePipeline(
    BasePipeline::Ptr &pipeline, const std::string &type_str,
    const std::string &param_prefix) const {
  if (isType<StereoPipeline>(type_str))
    configureStereo(pipeline, param_prefix);
  else if (isType<LidarPipeline>(type_str))
    configureLidar(pipeline, param_prefix);
  else {
    LOG(WARNING) << "[Tactic] Cannot find configure function for pipeline: "
                 << type_str;
  }
}

void ROSPipelineFactory::configureStereo(
    BasePipeline::Ptr &pipeline, const std::string &param_prefix) const {
  auto config = std::make_shared<StereoPipeline::Config>();
  // clang-format off
  config->preprocessing = node_->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  config->odometry = node_->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  config->bundle_adjustment = node_->declare_parameter<std::vector<std::string>>(param_prefix + ".bundle_adjustment", config->bundle_adjustment);
  config->localization = node_->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
  // clang-format on
  std::dynamic_pointer_cast<StereoPipeline>(pipeline)->setConfig(config);
}

void ROSPipelineFactory::configureLidar(BasePipeline::Ptr &pipeline,
                                        const std::string &param_prefix) const {
  auto config = std::make_shared<LidarPipeline::Config>();
  // clang-format off
  config->preprocessing = node_->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  config->odometry = node_->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  config->localization = node_->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
  // clang-format on
  std::dynamic_pointer_cast<LidarPipeline>(pipeline)->setConfig(config);
}

}  // namespace tactic
}  // namespace vtr