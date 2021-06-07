#include <vtr_tactic/pipelines/lidar_pipeline.hpp>

namespace vtr {
namespace tactic {

namespace {
PointCloudMapMsg copyPointcloudMap(const std::shared_ptr<PointMap> &map) {
  const auto &points = map->cloud.pts;
  const auto &normals = map->normals;
  const auto &scores = map->scores;
  auto N = points.size();

  PointCloudMapMsg map_msg;
  map_msg.points.reserve(N);
  map_msg.normals.reserve(N);

  for (unsigned i = 0; i < N; i++) {
    // points
    const auto &point = points[i];
    PointXYZMsg point_xyz;
    point_xyz.x = point.x;
    point_xyz.y = point.y;
    point_xyz.z = point.z;
    map_msg.points.push_back(point_xyz);
    // normals
    const auto &normal = normals[i];
    PointXYZMsg normal_xyz;
    normal_xyz.x = normal.x;
    normal_xyz.y = normal.y;
    normal_xyz.z = normal.z;
    map_msg.normals.push_back(normal_xyz);
    // scores
    map_msg.scores = scores;
  }
  return map_msg;
}
}  // namespace

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
  /// Store the current map for odometry to avoid reloading
  if (odo_map_vid_ != nullptr) {
    qdata->current_map_odo = odo_map_;
    qdata->current_map_odo_vid = odo_map_vid_;
  }

  /// Copy over the current map (pointer) being built
  if (new_map_ != nullptr) qdata->new_map = new_map_;

  auto tmp = std::make_shared<MapCache>();
  for (auto module : odometry_) module->run(*qdata, *tmp, graph);

  /// Store the current map for odometry to avoid reloading
  if (qdata->current_map_odo_vid) {
    odo_map_ = qdata->current_map_odo.ptr();
    odo_map_vid_ = qdata->current_map_odo_vid.ptr();
  }

  /// Store the current map being built (must exist)
  new_map_ = qdata->new_map.ptr();

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

  /// Store the current map for odometry to avoid reloading
  odo_map_ = qdata->new_map.ptr();
  odo_map_vid_ = std::make_shared<VertexId>(live_id);
  LOG(INFO) << "Odometry map being updated to vertex: " << live_id;

  /// Clear the current map being built
  new_map_.reset();

  /// Save point cloud map
  /// Note that since we also store the new map to odo_map_ which will then
  /// directly be used for localization (map recall module won't try to load it
  /// from graph), it is ok to save it in a separate thread - no synchronization
  /// issues. Also localizing against a map is a read only operation.
#ifdef DETERMINISTIC_VTR
  savePointcloudMap(qdata, graph, live_id);
#else
  /// Run pipeline according to the state
  if (map_saving_thread_future_.valid()) map_saving_thread_future_.wait();
  LOG(DEBUG) << "[Lidar Pipeline] Launching the point cloud map saving thread.";
  map_saving_thread_future_ =
      std::async(std::launch::async, [this, qdata, graph, live_id]() {
        savePointcloudMap(qdata, graph, live_id);
      });
#endif
}

void LidarPipeline::waitForKeyframeJob() {
  std::lock_guard<std::mutex> lck(map_saving_mutex_);
  if (map_saving_thread_future_.valid()) map_saving_thread_future_.wait();
}

void LidarPipeline::savePointcloudMap(QueryCache::Ptr qdata,
                                      const Graph::Ptr graph,
                                      VertexId live_id) {
  LOG(INFO) << "[Lidar Pipeline] Launching the point cloud map saving thread.";
  /// Store the submap into STPG \todo (yuchen) make this run faster!
  auto map_msg = copyPointcloudMap(qdata->new_map.ptr());
  auto vertex = graph->at(live_id);
  graph->registerVertexStream<PointCloudMapMsg>(live_id.majorId(), "pcl_map");
  vertex->insert("pcl_map", map_msg, *qdata->stamp);
  LOG(INFO)
      << "[Lidar Pipeline] Finish running the point cloud map saving thread.";
}

}  // namespace tactic
}  // namespace vtr
