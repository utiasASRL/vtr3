#include <vtr_tactic/pipelines/lidar_pipeline.hpp>

namespace vtr {
namespace tactic {

void LidarPipeline::configFromROS(const rclcpp::Node::SharedPtr &node,
                                  const std::string &param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config_->preprocessing);
  config_->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config_->odometry);
  config_->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config_->localization);
  // clang-format on
}

void LidarPipeline::initialize(const Graph::Ptr &) {
  if (!module_factory_) {
    std::string error{
        "Module factory required to initialize the lidar pipeline"};
    LOG(ERROR) << error;
    throw std::runtime_error{error};
  }

  // preprocessing
  for (auto module : config_->preprocessing)
    preprocessing_.push_back(module_factory_->make("preprocessing." + module));
  // odometry
  for (auto module : config_->odometry)
    odometry_.push_back(module_factory_->make("odometry." + module));
  // localization
  for (auto module : config_->localization)
    localization_.push_back(module_factory_->make("localization." + module));
}

void LidarPipeline::preprocess(QueryCache::Ptr &qdata,
                               const Graph::Ptr &graph) {
  auto tmp = std::make_shared<MapCache>();
  for (auto module : preprocessing_) module->run(*qdata, *tmp, graph);
}

void LidarPipeline::visualizePreprocess(QueryCache::Ptr &qdata,
                                        const Graph::Ptr &graph) {
  auto tmp = std::make_shared<MapCache>();
  for (auto module : preprocessing_) module->visualize(*qdata, *tmp, graph);
}

void LidarPipeline::runOdometry(QueryCache::Ptr &qdata,
                                const Graph::Ptr &graph) {
  auto tmp = std::make_shared<MapCache>();
  for (auto module : odometry_) module->run(*qdata, *tmp, graph);
  /// \todo detect odometry failure in lidar case, currently assume always
  /// successful
  *qdata->odo_success = true;
}

void LidarPipeline::visualizeOdometry(QueryCache::Ptr &qdata,
                                      const Graph::Ptr &graph) {
  auto tmp = std::make_shared<MapCache>();
  for (auto module : odometry_) module->visualize(*qdata, *tmp, graph);
}

void LidarPipeline::runLocalization(QueryCache::Ptr &qdata,
                                    const Graph::Ptr &graph) {
  // create a new map cache and fill it out
  auto tmp = std::make_shared<MapCache>();
  for (auto module : localization_) module->run(*qdata, *tmp, graph);
}

void LidarPipeline::visualizeLocalization(QueryCache::Ptr &qdata,
                                          const Graph::Ptr &graph) {
  // create a new map cache and fill it out
  auto tmp = std::make_shared<MapCache>();
  for (auto module : localization_) module->visualize(*qdata, *tmp, graph);
}

void LidarPipeline::processKeyframe(QueryCache::Ptr &qdata,

                                    const Graph::Ptr &graph, VertexId live_id) {
  auto tmp = std::make_shared<MapCache>();
  for (auto module : odometry_)
    module->updateGraph(*qdata, *tmp, graph, live_id);
}

}  // namespace tactic
}  // namespace vtr
