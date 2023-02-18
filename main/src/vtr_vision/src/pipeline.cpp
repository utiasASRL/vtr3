#include <vtr_vision/pipeline.hpp>
#include "vtr_tactic/modules/factory.hpp"


namespace vtr {
namespace vision {

using namespace tactic;
auto StereoPipeline::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                   const std::string &param_prefix) 
    -> ConstPtr {
  auto config = std::make_shared<Config>();
                                   
  // clang-format off
  config->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
//   config_->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config_->odometry);
//   config_->bundle_adjustment = node->declare_parameter<std::vector<std::string>>(param_prefix + ".bundle_adjustment", config_->bundle_adjustment);
//   config_->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config_->localization);
  // clang-format on
    return config;
}


StereoPipeline::StereoPipeline(
    const Config::ConstPtr &config,
    const std::shared_ptr<ModuleFactory> &module_factory,
    const std::string &name)
    : BasePipeline(module_factory, name), config_(config) {
    // preprocessing
    
  for (auto module : config_->preprocessing)
    preprocessing_.push_back(factory()->get("preprocessing." + module));
//   // odometry
//   for (auto module : config_->odometry)
//     odometry_.push_back(factory()->get("odometry." + module));
//   // localization
//   for (auto module : config_->localization)
//     localization_.push_back(factory()->get("localization." + module));
}

StereoPipeline::~StereoPipeline() {}

tactic::OutputCache::Ptr StereoPipeline::createOutputCache() const {
  return std::make_shared<tactic::OutputCache>();
}


void StereoPipeline::preprocess_(const tactic::QueryCache::Ptr &qdata0, const tactic::OutputCache::Ptr &output0,
                   const tactic::Graph::Ptr &graph,
                   const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto qdata = std::dynamic_pointer_cast<CameraQueryCache>(qdata0);
  // for (auto module : preprocessing_) module->run(*qdata0, *output0, graph, executor);
}





} //vision
} //vtr