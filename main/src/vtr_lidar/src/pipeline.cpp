// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file pipeline.cpp
 * \brief LidarPipeline class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/pipeline.hpp>

namespace vtr {
namespace lidar {

namespace {
PointMapMsg copyPointMap(const std::vector<PointXYZ> &points,
                         const std::vector<PointXYZ> &normals,
                         const std::vector<float> &scores,
                         const std::vector<std::pair<int, int>> &movabilities) {
  auto N = points.size();

  PointMapMsg map_msg;
  map_msg.points.reserve(N);
  map_msg.normals.reserve(N);
  map_msg.movabilities.reserve(N);

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
    // movabilities
    const auto &movability = movabilities[i];
    MovabilityMsg mb;
    mb.dynamic_obs = movability.first;
    mb.total_obs = movability.second;
    map_msg.movabilities.push_back(mb);
  }
  // scores
  map_msg.scores = scores;
  return map_msg;
}
}  // namespace

using namespace tactic;

void LidarPipeline::configFromROS(const rclcpp::Node::SharedPtr &node,
                                  const std::string &param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config_->preprocessing);
  config_->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config_->odometry);
  config_->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config_->localization);

  config_->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config_->map_voxel_size);
  // clang-format on

  /// Sets up module config
  module_factory_ = std::make_shared<ROSModuleFactory>(node);
  addModules();
}

void LidarPipeline::initialize(const Graph::Ptr &) {
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

void LidarPipeline::preprocess(QueryCache::Ptr &qdata0,
                               const Graph::Ptr &graph) {
  for (auto module : preprocessing_) module->run(*qdata0, graph);
}

void LidarPipeline::visualizePreprocess(QueryCache::Ptr &qdata0,
                                        const Graph::Ptr &graph) {
  for (auto module : preprocessing_) module->visualize(*qdata0, graph);
}

void LidarPipeline::runOdometry(QueryCache::Ptr &qdata0,
                                const Graph::Ptr &graph) {
  auto qdata = std::dynamic_pointer_cast<LidarQueryCache>(qdata0);

  // Clear internal states on first frame.
  if (*qdata->first_frame) reset();
  *qdata->odo_success = true;      // odometry success default to true
  setOdometryPrior(qdata, graph);  // a better prior T_r_m_odo using trajectory

  CLOG(DEBUG, "lidar.pipeline")
      << "Estimated T_m_r_odo (based on keyframe) from steam trajectory: "
      << (*qdata->T_r_m_odo).inverse().vec().transpose();

  /// Store the current map for odometry to avoid reloading
  if (odo_map_vid_ != nullptr) {
    qdata->current_map_odo = odo_map_;
    qdata->current_map_odo_vid = odo_map_vid_;
    qdata->current_map_odo_T_v_m = odo_map_T_v_m_;
  }

  /// Copy over the current map (pointer) being built
  if (new_map_ != nullptr) qdata->new_map = new_map_;

  for (auto module : odometry_) module->run(*qdata0, graph);

  /// Store the current map for odometry to avoid reloading
  if (qdata->current_map_odo_vid) {
    odo_map_ = qdata->current_map_odo.ptr();
    odo_map_vid_ = qdata->current_map_odo_vid.ptr();
    odo_map_T_v_m_ = qdata->current_map_odo_T_v_m.ptr();
  }

  /// Store the current map being built (must exist)
  if (qdata->new_map) new_map_ = qdata->new_map.ptr();

  if (*(qdata->keyframe_test_result) == KeyframeTestResult::FAILURE) {
    CLOG(WARNING, "lidar.pipeline")
        << "Odometry failed - trying to use the candidate query data to make a "
           "keyframe.";
    if (candidate_qdata_ != nullptr) {
      qdata = candidate_qdata_;
      *qdata->keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
      candidate_qdata_ = nullptr;
    } else {
      std::string err{
          "Does not have a valid candidate query data because last frame is "
          "also a keyframe. This happens when we have successive odometry "
          "failures. Although we can extrapolate using the trajectory, we "
          "instead error out for now because this could cause so many "
          "unpredictable failures and almost always means there is an issue "
          "with lidar odometry."};
      CLOG(ERROR, "lidar.pipeline") << err;
      throw std::runtime_error{err};
      /// \todo need to make sure map maintenance always create a non-empty map
      /// (in case last frame is a keyframe and we failed)
    }
  } else {
    trajectory_ = qdata->trajectory.ptr();
    trajectory_time_point_ = common::timing::toChrono(*qdata->stamp);
    /// keep this frame as a candidate for creating a keyframe
    if (*(qdata->keyframe_test_result) != KeyframeTestResult::CREATE_VERTEX)
      candidate_qdata_ = qdata;
    else
      candidate_qdata_ = nullptr;
  }
}

void LidarPipeline::visualizeOdometry(QueryCache::Ptr &qdata0,
                                      const Graph::Ptr &graph) {
  for (auto module : odometry_) module->visualize(*qdata0, graph);
}

void LidarPipeline::runLocalization(QueryCache::Ptr &qdata0,
                                    const Graph::Ptr &graph) {
  auto qdata = std::dynamic_pointer_cast<LidarQueryCache>(qdata0);

  /// Store the current map for odometry to avoid reloading
  if (loc_map_vid_ != nullptr) {
    qdata->current_map_loc = loc_map_;
    qdata->current_map_loc_vid = loc_map_vid_;
  }

  for (auto module : localization_) module->run(*qdata0, graph);

  /// Store the current map for odometry to avoid reloading
  if (qdata->current_map_loc_vid) {
    loc_map_ = qdata->current_map_loc.ptr();
    loc_map_vid_ = qdata->current_map_loc_vid.ptr();
  }
}

void LidarPipeline::visualizeLocalization(QueryCache::Ptr &qdata0,
                                          const Graph::Ptr &graph) {
  for (auto module : localization_) module->visualize(*qdata0, graph);
}

void LidarPipeline::processKeyframe(QueryCache::Ptr &qdata0,
                                    const Graph::Ptr &graph, VertexId live_id) {
  auto qdata = std::dynamic_pointer_cast<LidarQueryCache>(qdata0);

  for (auto module : odometry_) module->updateGraph(*qdata0, graph, live_id);

  /// Store the current map for odometry to avoid reloading
  odo_map_ = qdata->new_map.ptr();
  odo_map_vid_ = std::make_shared<VertexId>(live_id);  // same as qdata->live_id
  odo_map_T_v_m_ = qdata->new_map_T_v_m.ptr();
  CLOG(DEBUG, "lidar.pipeline")
      << "Odometry map being updated to vertex: " << *odo_map_vid_
      << " with T_m_v (T_map_vertex) set to: "
      << odo_map_T_v_m_->inverse().vec().transpose();

  /// Clear the current map being built
  new_map_.reset();

/// Save point cloud map
/// Note that since we also store the new map to odo_map_ which will then
/// directly be used for localization (map recall module won't try to load it
/// from graph), it is ok to save it in a separate thread - no synchronization
/// issues. Also localizing against a map is a read only operation.
#ifdef VTR_DETERMINISTIC
  savePointcloudMap(qdata, graph, live_id);
#else
  /// Run pipeline according to the state
  CLOG(DEBUG, "lidar.pipeline")
      << "[Lidar Pipeline] Launching the point cloud map saving thread.";
  std::lock_guard<std::mutex> lck(map_saving_mutex_);
  if (map_saving_thread_future_.valid()) map_saving_thread_future_.wait();
  map_saving_thread_future_ =
      std::async(std::launch::async, [this, qdata, graph, live_id]() {
        el::Helpers::setThreadName("lidar.pipeline.map_saving");
        savePointcloudMap(qdata, graph, live_id);
      });
#endif
}

void LidarPipeline::wait() {
  std::lock_guard<std::mutex> lck(map_saving_mutex_);
  if (map_saving_thread_future_.valid()) map_saving_thread_future_.get();
}

void LidarPipeline::reset() {
  candidate_qdata_ = nullptr;
  new_map_ = nullptr;
  odo_map_ = nullptr;
  odo_map_vid_ = nullptr;
  odo_map_T_v_m_ = nullptr;
  loc_map_ = nullptr;
  loc_map_vid_ = nullptr;
  trajectory_ = nullptr;
}

void LidarPipeline::addModules() {
  module_factory_->add<HoneycombConversionModule>();
  module_factory_->add<VelodyneConversionModule>();
  module_factory_->add<PreprocessingModule>();
  module_factory_->add<MapRecallModule>();
  module_factory_->add<OdometryICPModule>();
  module_factory_->add<MapMaintenanceModule>();
  module_factory_->add<KeyframeTestModule>();
  module_factory_->add<WindowedMapRecallModule>();
  module_factory_->add<LocalizationICPModule>();
}

void LidarPipeline::setOdometryPrior(LidarQueryCache::Ptr &qdata,
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
    CLOG(WARNING, "lidar.pipeline")
        << "The trajectory expired after " << traj_dt
        << " s for estimating the transform from keyframe at "
        << common::timing::toIsoString(trajectory_time_point_) << " to "
        << common::timing::toIsoString(query_time_point);
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

void LidarPipeline::savePointcloudMap(LidarQueryCache::Ptr qdata,
                                      const Graph::Ptr graph,
                                      VertexId live_id) {
  CLOG(DEBUG, "lidar.pipeline")
      << "[Lidar Pipeline] Start running the point map saving thread.";

  const auto &T_r_m = *qdata->new_map_T_v_m;

  /// get a shared pointer copy and a copy of the pointsk normals and scores
  const auto &map = qdata->new_map.ptr();
  auto points = map->cloud.pts;
  auto normals = map->normals;
  const auto &scores = map->normal_scores;
  const auto &movabilities = map->movabilities;

  /// Transform map points into the current keyframe
  const auto T_r_m_mat = T_r_m.matrix();
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)points.data(), 3, points.size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat(
      (float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = (T_r_m_mat.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (T_r_m_mat.block(0, 3, 3, 1)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;

  auto map_msg = copyPointMap(points, normals, scores, movabilities);
  auto vertex = graph->at(live_id);
  graph->registerVertexStream<PointMapMsg>(live_id.majorId(), "pointmap");
  vertex->insert("pointmap", map_msg, *qdata->stamp);
  CLOG(DEBUG, "lidar.pipeline")
      << "[Lidar Pipeline] Finish running the point map saving thread.";
}

}  // namespace lidar
}  // namespace vtr
