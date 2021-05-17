#include <vtr_tactic/pipelines/lidar_pipeline.hpp>

namespace vtr {
namespace tactic {

void LidarPipeline::initialize(const Graph::Ptr &graph) {
  if (!module_factory_) {
    std::string error{
        "Module factory required to initialize the lidar pipeline"};
    LOG(ERROR) << error;
    throw std::runtime_error{error};
  }

  preprocessing_module_ = module_factory_->make("preprocessing");
  recall_module_ = module_factory_->make("odometry_recall");
  odometry_icp_module_ = module_factory_->make("odometry_icp");
  keyframe_test_module_ = module_factory_->make("keyframe_test");
  map_maintenance_module_ = module_factory_->make("map");
  windowed_recall_module_ = module_factory_->make("localization_recall");
  localization_icp_module_ = module_factory_->make("localization_icp");
}

void LidarPipeline::preprocess(QueryCache::Ptr &qdata,
                               const Graph::Ptr &graph) {
  auto mdata = std::make_shared<MapCache>();
  preprocessing_module_->run(*qdata, *mdata, graph);
  preprocessing_module_->visualize(*qdata, *mdata, graph);
}

void LidarPipeline::runOdometry(QueryCache::Ptr &qdata,
                                const Graph::Ptr &graph) {
  auto mdata = std::make_shared<MapCache>();
  recall_module_->run(*qdata, *mdata, graph);
  odometry_icp_module_->run(*qdata, *mdata, graph);
  keyframe_test_module_->run(*qdata, *mdata, graph);
  map_maintenance_module_->run(*qdata, *mdata, graph);
}

void LidarPipeline::visualizeOdometry(QueryCache::Ptr &qdata,

                                      const Graph::Ptr &graph) {
  auto mdata = std::make_shared<MapCache>();
  recall_module_->visualize(*qdata, *mdata, graph);
  odometry_icp_module_->visualize(*qdata, *mdata, graph);
  keyframe_test_module_->visualize(*qdata, *mdata, graph);
  map_maintenance_module_->visualize(*qdata, *mdata, graph);
}

void LidarPipeline::runLocalization(QueryCache::Ptr &qdata,

                                    const Graph::Ptr &graph) {
  auto mdata = std::make_shared<MapCache>();
  windowed_recall_module_->run(*qdata, *mdata, graph);
  localization_icp_module_->run(*qdata, *mdata, graph);
}

void LidarPipeline::visualizeLocalization(QueryCache::Ptr &qdata,

                                          const Graph::Ptr &graph) {
  auto mdata = std::make_shared<MapCache>();
  // for now, manually specify the execution order
  windowed_recall_module_->visualize(*qdata, *mdata, graph);
  localization_icp_module_->visualize(*qdata, *mdata, graph);
}

void LidarPipeline::processKeyframe(QueryCache::Ptr &qdata,

                                    const Graph::Ptr &graph, VertexId live_id) {
  auto mdata = std::make_shared<MapCache>();
  // for now, manually specify the execution order
  preprocessing_module_->updateGraph(*qdata, *mdata, graph, live_id);
  recall_module_->updateGraph(*qdata, *mdata, graph, live_id);
  odometry_icp_module_->updateGraph(*qdata, *mdata, graph, live_id);
  keyframe_test_module_->updateGraph(*qdata, *mdata, graph, live_id);
  map_maintenance_module_->updateGraph(*qdata, *mdata, graph, live_id);
}

}  // namespace tactic
}  // namespace vtr
