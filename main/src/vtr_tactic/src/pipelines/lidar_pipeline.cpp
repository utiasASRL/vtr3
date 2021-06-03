#include <vtr_tactic/pipelines/lidar_pipeline.hpp>

namespace vtr {
namespace tactic {

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
  /// \todo put visualization somewhere else
  for (auto module : preprocessing_) module->visualize(*qdata, *tmp, graph);
}

void LidarPipeline::runOdometry(QueryCache::Ptr &qdata,
                                const Graph::Ptr &graph) {
  auto tmp = std::make_shared<MapCache>();
  for (auto module : odometry_) module->run(*qdata, *tmp, graph);
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
