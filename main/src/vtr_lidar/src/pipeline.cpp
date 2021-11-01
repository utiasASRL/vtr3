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
#include "vtr_lidar/pipeline.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;

void LidarPipeline::configFromROS(const rclcpp::Node::SharedPtr &node,
                                  const std::string &param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config_->preprocessing);
  config_->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config_->odometry);
  config_->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config_->localization);
  // clang-format on

  /// Sets up module config
  module_factory_ = std::make_shared<ROSModuleFactory>(node);
  addModules();
}

void LidarPipeline::initialize(const Graph::Ptr &) {
  /// \todo module contruction should be done in initializer
  // preprocessing
  for (auto module : config_->preprocessing)
    preprocessing_.push_back(module_factory_->make("preprocessing." + module));
  // odometry
  for (auto module : config_->odometry)
    odometry_.push_back(module_factory_->make("odometry." + module));
  // localization
  for (auto module : config_->localization)
    localization_.push_back(module_factory_->make("localization." + module));
  // add task queue to each module
  for (const auto &module : preprocessing_) module->setTaskQueue(task_queue_);
  for (const auto &module : odometry_) module->setTaskQueue(task_queue_);
  for (const auto &module : localization_) module->setTaskQueue(task_queue_);
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
  if (curr_map_odo_ != nullptr) qdata->curr_map_odo = curr_map_odo_;

  /// Copy over the current map (pointer) being built
  if (new_map_odo_ != nullptr) qdata->new_map_odo = new_map_odo_;

  for (auto module : odometry_) module->run(*qdata0, graph);

  /// Store the current map for odometry to avoid reloading
  if (qdata->curr_map_odo) curr_map_odo_ = qdata->curr_map_odo.ptr();

  /// Store the scan if decided to store it
  const auto &T_s_r = *qdata->T_s_r;
  const auto &T_r_m = *qdata->T_r_m_odo;
  bool store_scan = false;
  if (!(*qdata->odo_success))
    store_scan = false;
  else if (new_scan_odo_.empty())
    store_scan = true;
  else {
    const auto &T_m_sm1 = new_scan_odo_.rbegin()->second->T_vertex_map();
    // T_<robot>_<live id> * T_<live id>_<sensor-1> * T_<sensor>_<robot>
    auto se3vec = (T_r_m * T_m_sm1 * T_s_r).vec();
    auto dtran = se3vec.head<3>().norm();
    auto drot = se3vec.tail<3>().norm() * 57.29577;  // 180/pi
    CLOG(DEBUG, "lidar.pipeline")
        << "Relative motion since first lidar scan: tran: " << dtran
        << ", rot: " << drot << " with map size: " << new_scan_odo_.size();
    /// \todo parameters
    if (dtran > 0.3 || drot > 5.0) store_scan = true;
  }
  if (store_scan) {
#if false  /// store raw point cloud
    // raw scan
    auto new_raw_scan_odo = std::make_shared<PointScan<PointWithInfo>>();
    new_raw_scan_odo->point_map() = *qdata->undistorted_raw_point_cloud;
    new_raw_scan_odo->T_vertex_map() = (T_s_r * T_r_m).inverse();
    new_raw_scan_odo_.try_emplace(*qdata->stamp, new_raw_scan_odo);
#endif
    // preprocessed scan
    auto new_scan_odo = std::make_shared<PointScan<PointWithInfo>>();
    new_scan_odo->point_map() = *qdata->undistorted_point_cloud;
    new_scan_odo->T_vertex_map() = (T_s_r * T_r_m).inverse();
    new_scan_odo_.try_emplace(*qdata->stamp, new_scan_odo);
  }

  /// Store the current map being built (must exist)
  if (qdata->new_map_odo) new_map_odo_ = qdata->new_map_odo.ptr();

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
  if (curr_map_loc_ != nullptr) {
    qdata->curr_map_loc = curr_map_loc_;
  }

  for (auto module : localization_) module->run(*qdata0, graph);

  /// Store the current map for odometry to avoid reloading
  if (qdata->curr_map_loc) {
    curr_map_loc_ = qdata->curr_map_loc.ptr();
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
  curr_map_odo_ = qdata->new_map_odo.ptr();
  curr_map_odo_->vertex_id() = live_id;

  /// Prepare to save map and scans
  auto vertex = graph->at(live_id);
  using PointScanLM = storage::LockableMessage<PointScan<PointWithInfo>>;
  using PointMapLM = storage::LockableMessage<PointMap<PointWithInfo>>;

  /// Save the accumulated lidar scans (need to correct transform before saving)
#if false  /// store raw point cloud
  // raw scans
  for (auto it = new_raw_scan_odo_.begin(); it != new_raw_scan_odo_.end();
       it++) {
    // correct transform to the live id - this is essentially:
    // T_<live id>_<scan> = T_<live id>_<last id> * T_<last id>_<scan>
    it->second->T_vertex_map() =
        curr_map_odo_->T_vertex_map() * it->second->T_vertex_map();
    it->second->vertex_id() = live_id;
    // save the point scan
    auto scan_msg = std::make_shared<PointScanLM>(it->second, it->first);
    vertex->insert<PointScan<PointWithInfo>>("raw_point_scan", scan_msg);
  }
  new_raw_scan_odo_.clear();
#endif
  // down-sampled scans
  for (auto it = new_scan_odo_.begin(); it != new_scan_odo_.end(); it++) {
    // correct transform to the live id - this is essentially:
    // T_<live id>_<scan> = T_<live id>_<last id> * T_<last id>_<scan>
    it->second->T_vertex_map() =
        curr_map_odo_->T_vertex_map() * it->second->T_vertex_map();
    it->second->vertex_id() = live_id;
    // save the point scan
    auto scan_msg = std::make_shared<PointScanLM>(it->second, it->first);
    vertex->insert<PointScan<PointWithInfo>>(
        "point_scan", "vtr_lidar_msgs/msg/PointScan", scan_msg);
  }
  new_scan_odo_.clear();

  /// Save the point cloud map
  auto map_msg = std::make_shared<PointMapLM>(curr_map_odo_, *qdata->stamp);
  vertex->insert<PointMap<PointWithInfo>>(
      "point_map", "vtr_lidar_msgs/msg/PointMap", map_msg);

  /// Save the point cloud map copy in case we need it later
  auto map_copy_msg =
      std::make_shared<PointMapLM>(curr_map_odo_, *qdata->stamp);
  vertex->insert<PointMap<PointWithInfo>>(
      "point_map_v" + std::to_string(curr_map_odo_->version()),
      "vtr_lidar_msgs/msg/PointMap", map_copy_msg);

  /// Clear the current map being built
  new_map_odo_.reset();
}

void LidarPipeline::wait() {}

void LidarPipeline::reset() {
  candidate_qdata_ = nullptr;
#if false  /// store raw point cloud
  new_raw_scan_odo_.clear();
#endif
  new_scan_odo_.clear();
  new_map_odo_ = nullptr;
  curr_map_odo_ = nullptr;
  curr_map_loc_ = nullptr;
  trajectory_ = nullptr;
}

void LidarPipeline::addModules() {
  module_factory_->add<HoneycombConversionModuleV2>();
  module_factory_->add<VelodyneConversionModule>();
  module_factory_->add<VelodyneConversionModuleV2>();
  module_factory_->add<PreprocessingModuleV2>();
  module_factory_->add<OdometryICPModuleV2>();
  module_factory_->add<OdometryMapRecallModule>();
  module_factory_->add<OdometryMapMergingModule>();
  module_factory_->add<LocalizationMapRecallModule>();
  module_factory_->add<LocalizationICPModuleV2>();
  module_factory_->add<KeyframeTestModule>();
  module_factory_->add<DynamicDetectionModule>();
  module_factory_->add<IntraExpMergingModule>();
  module_factory_->add<InterExpMergingModule>();
}

void LidarPipeline::setOdometryPrior(LidarQueryCache::Ptr &qdata,
                                     const Graph::Ptr &graph) {
  if (trajectory_ == nullptr) return;

  // we need to update the new T_r_m prediction
  auto live_time =
      pose_graph::toTimestampMsg(graph->at(*qdata->live_id)->keyframeTime());
  auto query_time = pose_graph::toTimestampMsg(*qdata->stamp);

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

}  // namespace lidar
}  // namespace vtr
