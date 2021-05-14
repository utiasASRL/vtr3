#include <vtr_tactic/pipelines/pipeline_factory.hpp>

namespace vtr {
namespace tactic {

BasePipeline::Ptr PipelineFactory::make(const std::string& static_name) const {
  LOG(DEBUG) << "Constructing pipeline with static name: " << static_name;
  auto pipeline = type_switch_.make(static_name);
  if (!pipeline) {
    auto msg = "Unknown pipeline of static name: " + static_name;
    LOG(ERROR) << msg;
    throw std::invalid_argument(msg);
  }
  return pipeline;
}

}  // namespace tactic
}  // namespace vtr
