#include <vtr_tactic/pipelines/lidar_pipeline.hpp>

namespace vtr {
namespace tactic {

namespace {
PointCloudMapMsg copyPointcloudMap(
    const std::shared_ptr<vtr::lidar::PointMap> &map) {
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
  }
  // scores
  map_msg.scores = scores;
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

  config_->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config_->map_voxel_size);
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
  /// Get a better prior T_r_m_odo using trajectory if valid
  setOdometryPrior(qdata, graph);

  /// Store the current map for odometry to avoid reloading
  if (odo_map_vid_ != nullptr) {
    qdata->current_map_odo = odo_map_;
    qdata->current_map_odo_vid = odo_map_vid_;
    qdata->current_map_odo_T_v_m = odo_map_T_v_m_;
  }

  /// Copy over the current map (pointer) being built
  if (new_map_ != nullptr) qdata->new_map = new_map_;

  auto tmp = std::make_shared<MapCache>();
  for (auto module : odometry_) module->run(*qdata, *tmp, graph);

  /// Store the current map for odometry to avoid reloading
  if (qdata->current_map_odo_vid) {
    odo_map_ = qdata->current_map_odo.ptr();
    odo_map_vid_ = qdata->current_map_odo_vid.ptr();
    odo_map_T_v_m_ = qdata->current_map_odo_T_v_m.ptr();
  }

  /// Store the current map being built (must exist)
  new_map_ = qdata->new_map.ptr();

  if (*(qdata->keyframe_test_result) == KeyframeTestResult::FAILURE) {
    LOG(ERROR) << "Odometry failed! Looking into this.";
    throw std::runtime_error{"Odometry failed! Looking into this."};
  } else {
    trajectory_ = qdata->trajectory.ptr();
    trajectory_time_point_ = common::timing::toChrono(*qdata->stamp);
    /// \todo create candidate cached qdata.
  }
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
  odo_map_T_v_m_ = qdata->T_r_m_odo.ptr();
  LOG(DEBUG) << "Odometry map being updated to vertex: " << live_id;

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

void LidarPipeline::setOdometryPrior(QueryCache::Ptr &qdata,
                                     const Graph::Ptr &graph) {
  if (trajectory_ == nullptr) return;

  // we need to update the new T_r_m prediction
  auto live_time = graph->at(*qdata->live_id)->keyFrameTime();
  auto query_time = *qdata->stamp;

  // The elapsed time since the last keyframe
  auto live_time_point = common::timing::toChrono(live_time);
  auto query_time_point = common::timing::toChrono(query_time);
  auto dt_duration = query_time_point - live_time_point;
  double dt = std::chrono::duration<double>(dt_duration).count();

  // Make sure the trajectory is current
  auto traj_dt_duration = query_time_point - trajectory_time_point_;
  double traj_dt = std::chrono::duration<double>(traj_dt_duration).count();
  if (traj_dt > 1.0) {
    LOG(WARNING) << "The trajectory expired after " << traj_dt
                 << " s for estimating the transform from keyframe at "
                 << common::timing::toIsoString(trajectory_time_point_)
                 << " to " << common::timing::toIsoString(query_time_point);
    trajectory_.reset();
    return;
  }

  // Update the T_r_m prediction using prior
  Eigen::Matrix<double, 6, 6> cov =
      Eigen::Matrix<double, 6, 6>::Identity() * pow(dt, 2.0);
  // scale the rotational uncertainty to be one order of magnitude lower than
  // the translational uncertainty.
  cov.block(3, 3, 3, 3) /= 10;

  // Query the saved trajectory estimator we have with the candidate frame
  // time
  auto live_time_nsec =
      steam::Time(static_cast<int64_t>(live_time.nanoseconds_since_epoch));
  auto live_eval = trajectory_->getInterpPoseEval(live_time_nsec);
  // Query the saved trajectory estimator we have with the current frame time
  auto query_time_nsec =
      steam::Time(static_cast<int64_t>(query_time.nanoseconds_since_epoch));
  auto query_eval = trajectory_->getInterpPoseEval(query_time_nsec);

  // find the transform between the candidate and current in the vehicle frame
  EdgeTransform T_r_m_odo(query_eval->evaluate() *
                          live_eval->evaluate().inverse());
  // give it back to the caller, TODO: (old) We need to get the covariance out
  // of the trajectory.

  // This ugliness of setting the time is because we don't have a reliable and
  // tested way of predicting the covariance. This is used by the stereo
  // matcher to decide how tight it should set its pixel search
  T_r_m_odo.setCovariance(cov);

  *qdata->T_r_m_odo = T_r_m_odo;
}

void LidarPipeline::savePointcloudMap(QueryCache::Ptr qdata,
                                      const Graph::Ptr graph,
                                      VertexId live_id) {
  LOG(DEBUG) << "[Lidar Pipeline] Launching the point cloud map saving thread.";

  const auto &T_r_m = *qdata->T_r_m_odo;

  /// get a shared pointer copy and a copy of the pointsk normals and scores
  const auto &map = qdata->new_map.ptr();
  auto points = map->cloud.pts;
  auto normals = map->normals;
  auto scores = map->scores;

  /// Transform subsampled points into the map frame
  const auto T_r_m_mat = T_r_m.matrix();
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)points.data(), 3, points.size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat(
      (float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = (T_r_m_mat.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (T_r_m_mat.block(0, 3, 3, 1)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;

  /// create a new map to rebuild kd-tree \todo optimize this
  LOG(DEBUG) << "Creating new map with size: " << config_->map_voxel_size;
  auto map_to_save =
      std::make_shared<vtr::lidar::PointMap>(config_->map_voxel_size);
  map_to_save->update(points, normals, scores);

  /// Store the submap into STPG \todo (yuchen) make this run faster!
  auto map_msg = copyPointcloudMap(map_to_save);
  auto vertex = graph->at(live_id);
  graph->registerVertexStream<PointCloudMapMsg>(live_id.majorId(), "pcl_map");
  vertex->insert("pcl_map", map_msg, *qdata->stamp);
  LOG(DEBUG)
      << "[Lidar Pipeline] Finish running the point cloud map saving thread.";
}

}  // namespace tactic
}  // namespace vtr
