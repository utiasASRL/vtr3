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
 * \file tactic_v2.cpp
 * \brief Tactic class method definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_tactic/tactic_v2.hpp"

#include "vtr_tactic/storables.hpp"

namespace vtr {
namespace tactic {

PipelineInterface::PipelineInterface(const bool& enable_parallelization,
                                     const OutputCache::Ptr& output,
                                     const Graph::Ptr& graph,
                                     const size_t& num_async_threads,
                                     const size_t& async_queue_size)
    : task_queue_(std::make_shared<TaskExecutor>(
          output, graph, num_async_threads, async_queue_size)),
      enable_parallelization_(enable_parallelization) {
  // clang-format off
  preprocessing_thread_ = std::thread(&PipelineInterface::preprocess, this);
  odometry_mapping_thread_ = std::thread(&PipelineInterface::runOdometryMapping, this);
  localization_thread_ = std::thread(&PipelineInterface::runLocalization, this);
  // clang-format on
  task_queue_->start();
}

void PipelineInterface::join() {
  auto lck = lockPipeline();
  if (preprocessing_thread_.joinable()) {
    preprocessing_buffer_.push(nullptr, false);
    preprocessing_thread_.join();
  }
  if (odometry_mapping_thread_.joinable()) {
    odometry_mapping_buffer_.push(nullptr, false);
    odometry_mapping_thread_.join();
  }
  if (localization_thread_.joinable()) {
    localization_buffer_.push(nullptr, false);
    localization_thread_.join();
  }
  task_queue_->stop();
}

auto PipelineInterface::lockPipeline() -> PipelineLock {
  // Lock so that no more data are passed into the pipeline
  PipelineLock lock(pipeline_mutex_);
  // Wait for the pipeline to be empty
  pipeline_semaphore_.wait();
  return lock;
}

void PipelineInterface::input(const QueryCache::Ptr& qdata) {
  PipelineLock lock(pipeline_mutex_, std::defer_lock_t());
  if (lock.try_lock_for(std::chrono::milliseconds(30))) {
    if (enable_parallelization_)
      inputParallel(qdata);
    else
      inputSequential(qdata);
  } else {
    CLOG(WARNING, "tactic")
        << "Dropping frame due to unavailable pipeline mutex.";
  }
}

void PipelineInterface::inputSequential(const QueryCache::Ptr& qdata) {
  pipeline_semaphore_.release();

  CLOG(DEBUG, "tactic") << "Accepting a new frame: " << *qdata->stamp;
  input_(qdata);

  CLOG(DEBUG, "tactic") << "Start running preprocessing: " << *qdata->stamp;
  preprocess_(qdata);
  CLOG(DEBUG, "tactic") << "Finish running preprocessing: " << *qdata->stamp;

  CLOG(DEBUG, "tactic") << "Start running odometry mapping, timestamp: "
                        << *qdata->stamp;
  runOdometryMapping_(qdata);
  CLOG(DEBUG, "tactic") << "Finish running odometry mapping, timestamp: "
                        << *qdata->stamp;

  CLOG(DEBUG, "tactic") << "Start running localization, timestamp: "
                        << *qdata->stamp;
  runLocalization_(qdata);
  CLOG(DEBUG, "tactic") << "Finish running localization, timestamp: "
                        << *qdata->stamp;

  pipeline_semaphore_.acquire();
}

void PipelineInterface::inputParallel(const QueryCache::Ptr& qdata) {
  pipeline_semaphore_.release();
  CLOG(DEBUG, "tactic") << "Accepting a new frame: " << *qdata->stamp;
  const bool discardable = input_(qdata);
  const bool discarded = preprocessing_buffer_.push(qdata, discardable);
  CLOG_IF(discarded, WARNING, "tactic")
      << "[input] Buffer is full, one frame discarded.";
  if (discarded) pipeline_semaphore_.acquire();
}

void PipelineInterface::preprocess() {
  el::Helpers::setThreadName("tactic.preprocessing");
  while (true) {
    auto qdata = preprocessing_buffer_.pop();
    if (qdata == nullptr) return;
    CLOG(DEBUG, "tactic") << "Start running preprocessing: " << *qdata->stamp;
    const bool discardable = preprocess_(qdata);
    const bool discarded = odometry_mapping_buffer_.push(qdata, discardable);
    CLOG_IF(discarded, WARNING, "tactic")
        << "[preprocess] Buffer is full, one frame discarded.";
    if (discarded) pipeline_semaphore_.acquire();
    CLOG(DEBUG, "tactic") << "Finish running preprocessing: " << *qdata->stamp;
  }
}

/** \brief Odometry & mapping thread, preprocess->odo&mapping->localization */
void PipelineInterface::runOdometryMapping() {
  el::Helpers::setThreadName("tactic.odometry_mapping");
  while (true) {
    auto qdata = odometry_mapping_buffer_.pop();
    if (qdata == nullptr) return;
    CLOG(DEBUG, "tactic") << "Start running odometry mapping, timestamp: "
                          << *qdata->stamp;
    const bool discardable = runOdometryMapping_(qdata);
    const bool discarded = localization_buffer_.push(qdata, discardable);
    CLOG_IF(discarded, WARNING, "tactic")
        << "[odometry_mapping] Buffer is full, one frame discarded.";
    if (discarded) pipeline_semaphore_.acquire();
    CLOG(DEBUG, "tactic") << "Finish running odometry mapping, timestamp: "
                          << *qdata->stamp;
  }
}

/** \brief Localization thread, odomtry&mapping->localization */
void PipelineInterface::runLocalization() {
  el::Helpers::setThreadName("tactic.localization");
  while (true) {
    auto qdata = localization_buffer_.pop();
    if (qdata == nullptr) return;
    CLOG(DEBUG, "tactic") << "Start running localization, timestamp: "
                          << *qdata->stamp;
    runLocalization_(qdata);
    CLOG(DEBUG, "tactic") << "Finish running localization, timestamp: "
                          << *qdata->stamp;
    pipeline_semaphore_.acquire();
  }
}

auto TacticV2::Config::fromROS(const rclcpp::Node::SharedPtr& node,
                               const std::string& prefix) -> UniquePtr {
  auto config = std::make_unique<Config>();
  // clang-format off

  /// setup tactic
  config->task_queue_num_threads = node->declare_parameter<int>(prefix+".task_queue_num_threads", 1);
  config->task_queue_size = node->declare_parameter<int>(prefix+".task_queue_size", -1);

  config->enable_parallelization = node->declare_parameter<bool>(prefix+".enable_parallelization", false);
  config->preprocessing_skippable = node->declare_parameter<bool>(prefix+".preprocessing_skippable", true);
  config->odometry_mapping_skippable = node->declare_parameter<bool>(prefix+".odometry_mapping_skippable", true);
  config->localization_skippable = node->declare_parameter<bool>(prefix+".localization_skippable", true);

  config->localization_only_keyframe = node->declare_parameter<bool>(prefix+".localization_only_keyframe", false);
  const auto dlc = node->declare_parameter<std::vector<double>>(prefix+".default_loc_cov", std::vector<double>{});
  if (dlc.size() != 6) {
    std::string err{"Tactic default localization covariance malformed. Must be 6 elements!"};
    CLOG(ERROR, "tactic") << err;
    throw std::invalid_argument{err};
  }
  config->default_loc_cov.diagonal() << dlc[0], dlc[1], dlc[2], dlc[3], dlc[4], dlc[5];

  config->extrapolate_odometry = node->declare_parameter<bool>(prefix+".extrapolate_odometry", false);

  config->merge_threshold = node->declare_parameter<std::vector<double>>(prefix+".merge_threshold", std::vector<double>{0.5, 0.25, 0.2});
  if (config->merge_threshold.size() != 3) {
    std::string err{"Merge threshold malformed. Must be 3 elements!"};
    CLOG(ERROR, "tactic") << err;
    throw std::invalid_argument{err};
  }

  config->visualize = node->declare_parameter<bool>(prefix+".visualize", false);
  const auto vis_loc_path_offset = node->declare_parameter<std::vector<double>>(prefix+".vis_loc_path_offset", std::vector<double>{0, 0, 0});
  if (vis_loc_path_offset.size() != 3) {
    std::string err{"Localization path offset malformed. Must be 3 elements!"};
    CLOG(ERROR, "tactic") << err;
    throw std::invalid_argument{err};
  }
  config->vis_loc_path_offset << vis_loc_path_offset[0], vis_loc_path_offset[1], vis_loc_path_offset[2];

  /// setup localization chain
  config->chain_config.min_cusp_distance = node->declare_parameter<double>(prefix+".chain.min_cusp_distance", 1.5);
  config->chain_config.angle_weight = node->declare_parameter<double>(prefix+".chain.angle_weight", 7.0);
  config->chain_config.search_depth = node->declare_parameter<int>(prefix+".chain.search_depth", 20);
  config->chain_config.search_back_depth = node->declare_parameter<int>(prefix+".chain.search_back_depth", 10);
  config->chain_config.distance_warning = node->declare_parameter<double>(prefix+".chain.distance_warning", 3);

  // clang-format on
  return config;
}

TacticV2::TacticV2(Config::UniquePtr config, const BasePipeline::Ptr& pipeline,
                   const OutputCache::Ptr& output, const Graph::Ptr& graph,
                   const TacticCallbackInterface::Ptr& callback)
    : PipelineInterface(config->enable_parallelization, output, graph,
                        config->task_queue_num_threads,
                        config->task_queue_size),
      config_(std::move(config)),
      pipeline_(pipeline),
      output_(output),
      chain_(std::make_shared<LocalizationChain>(config_->chain_config, graph)),
      graph_(graph),
      callback_(callback) {
  //
  output_->chain = chain_;  // shared pointing to the same chain, no copy
  //
  pipeline_->initialize(output_, graph_);
}

void TacticV2::setPipeline(const PipelineMode& pipeline_mode) {
  auto lock = lockPipeline();
  pipeline_mode_ = pipeline_mode;
}

void TacticV2::addRun(const bool ephemeral) {
  (void)ephemeral;  /// \todo unused for now

  auto lock = lockPipeline();

  graph_->addRun();

  // re-initialize the run
  first_frame_ = true;
  current_vertex_id_ = VertexId((uint64_t)-1);

  // re-initialize the localization chain (path will be added later)
  chain_->reset();

  // re-initialize the pose records for visualization
  T_w_m_odo_ = EdgeTransform(true);
  T_w_m_loc_ = EdgeTransform(true);
  keyframe_poses_.clear();
  odometry_poses_.clear();
  keyframe_poses_.reserve(1000);
  odometry_poses_.reserve(5000);
}

void TacticV2::setPath(const VertexId::Vector& path, const bool follow) {
  auto lock = lockPipeline();

  /// Clear any existing path in UI
  callback_->clearPathUI(*this);

  /// Set path and target localization
  CLOG(INFO, "tactic") << "Set path of size " << path.size();

  auto chain_lock = chain_->guard();
  chain_->setSequence(path);
  if (path.size() > 0) {
    chain_->expand();
    if (follow) {
      callback_->publishPathRviz(*this);
      callback_->publishPathUI(*this);
    }
  }
}

bool TacticV2::input_(const QueryCache::Ptr&) {
  return config_->preprocessing_skippable;
}

bool TacticV2::preprocess_(const QueryCache::Ptr& qdata) {
  // Setup caches
  qdata->pipeline_mode.emplace(pipeline_mode_);
  qdata->first_frame.emplace(first_frame_);
  first_frame_ = false;

  /// Preprocess incoming data, which always runs no matter what mode we are in.
  pipeline_->preprocess(qdata, output_, graph_, task_queue_);
  if (config_->visualize)
    pipeline_->visualizePreprocess(qdata, output_, graph_, task_queue_);

  return config_->odometry_mapping_skippable;
}

bool TacticV2::runOdometryMapping_(const QueryCache::Ptr& qdata) {
  // Setup caches
  qdata->live_id.emplace(current_vertex_id_);
  qdata->keyframe_test_result.emplace(KeyframeTestResult::DO_NOTHING);
  qdata->odo_success.emplace(false);

  switch (pipeline_mode_) {
    /// \note There are lots of repetitive code in the following four functions,
    /// maybe we can combine them at some point, but for now, consider leaving
    /// them separate so that it is slightly more clear what is happening during
    /// each pipeline mode.
    case PipelineMode::Branching:
      return branchOdometryMapping(qdata);
    case PipelineMode::Merging:
      return mergeOdometryMapping(qdata);
    case PipelineMode::Searching:
      return searchOdometryMapping(qdata);
    case PipelineMode::Following:
      return followOdometryMapping(qdata);
    default:
      return true;
  }
  return true;
}

bool TacticV2::branchOdometryMapping(const QueryCache::Ptr& qdata) {
  /// Prior assumes no motion since last processed frame
  qdata->T_r_m_odo.emplace(chain_->T_leaf_petiole());
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to live vertex"
                        << *qdata->live_id << " (i.e., T_m_r odometry): "
                        << (*qdata->T_r_m_odo).inverse().vec().transpose();

  /// Get relative pose estimate and whether a keyframe should be created
  pipeline_->runOdometry(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic") << "Estimated transformation from robot to live vertex"
                        << *qdata->live_id << " (i.e., T_m_r odometry): "
                        << (*qdata->T_r_m_odo).inverse().vec().transpose();

  /// Add to a global odometry estimate vector for debugging (only for branch)
  odometry_poses_.push_back(T_w_m_odo_ * (*qdata->T_r_m_odo).inverse());

  if (config_->visualize) {
    callback_->publishOdometryRviz(*this, *qdata);
    pipeline_->visualizeOdometry(qdata, output_, graph_, task_queue_);
  }

  /// Update Odometry in localization chain without updating trunk (because in
  /// branch mode there's no trunk to localize against)
  chain_->updatePetioleToLeafTransform(*qdata->T_r_m_odo, false);

  /// Update persistent localization (only when the first keyframe has been
  /// created so that current vertex id is valid.)
  if (current_vertex_id_.isValid())
    updatePersistentLoc(*qdata->stamp, *qdata->live_id, *qdata->T_r_m_odo,
                        true);

  /// Check if we should create a new vertex
  const auto& keyframe_test_result = *qdata->keyframe_test_result;
  if (keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    /// Add new vertex to the posegraph
    bool first_keyframe = !current_vertex_id_.isValid();
    if (!current_vertex_id_.isValid())
      addDanglingVertex(*(qdata->stamp));
    else
      addConnectedVertex(*(qdata->stamp), *(qdata->T_r_m_odo), true);
    (void)first_keyframe;  /// \todo unused for now
    CLOG(INFO, "tactic") << "Creating a new keyframe with id "
                         << current_vertex_id_;

    /// Compute odometry in world frame for visualization.
    T_w_m_odo_ = T_w_m_odo_ * (*qdata->T_r_m_odo).inverse();
    keyframe_poses_.emplace_back();
    keyframe_poses_.back().pose =
        tf2::toMsg(Eigen::Affine3d(T_w_m_odo_.matrix()));
    if (keyframe_poses_.size() - 1 != current_vertex_id_.minorId()) {
      std::string err{
          "Number of keyframe poses does not match the live vertex minor "
          "id. This will cause localization thread to update wrong keyframe "
          "pose."};
      CLOG(ERROR, "tactic") << err;
      throw std::runtime_error{err};
    }

    /// Update live id to the just-created vertex id and T_r_m_odo
    qdata->live_id.emplace(current_vertex_id_);
    qdata->T_r_m_odo.emplace(true);  // identity with zero covariance

    /// Call the pipeline to process the keyframe
    pipeline_->processKeyframe(qdata, output_, graph_, task_queue_);

    /// Set the new petiole without updating trunk since we are in branch mode
    chain_->setPetiole(current_vertex_id_);

    /// Reset the map to robot transform and new vertex flag
    updatePersistentLoc(*qdata->stamp, current_vertex_id_, EdgeTransform(true),
                        true);
  }

  return config_->localization_skippable;
}

bool TacticV2::mergeOdometryMapping(const QueryCache::Ptr& qdata) {
  (void)qdata;
  return true;
}

bool TacticV2::searchOdometryMapping(const QueryCache::Ptr& qdata) {
  (void)qdata;
  return true;
}

bool TacticV2::followOdometryMapping(const QueryCache::Ptr& qdata) {
  /// Prior assumes no motion since last processed frame
  qdata->T_r_m_odo.emplace(chain_->T_leaf_petiole());
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to live vertex"
                        << *qdata->live_id << " (i.e., T_m_r odometry): "
                        << (*qdata->T_r_m_odo).inverse().vec().transpose();

  /// Get relative pose estimate and whether a keyframe should be created
  pipeline_->runOdometry(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic") << "Estimated transformation from robot to live vertex"
                        << *qdata->live_id << " (i.e., T_m_r odometry): "
                        << (*qdata->T_r_m_odo).inverse().vec().transpose();

  if (config_->visualize) {
    callback_->publishOdometryRviz(*this, *qdata);
    pipeline_->visualizeOdometry(qdata, output_, graph_, task_queue_);
  }

  /// Update odometry in localization chain, also update estimated closest
  /// trunk without looking backwards
  chain_->updatePetioleToLeafTransform(*qdata->T_r_m_odo, true, false);

  /// Update persistent localization (only when the first keyframe has been
  /// created so that current vertex id is valid.)
  const auto [trunk_id, T_leaf_trunk, localized] = [&]() {
    auto lock = chain_->guard();
    return std::make_tuple(chain_->trunkVertexId(), chain_->T_leaf_trunk(),
                           chain_->isLocalized());
  }();
  updatePersistentLoc(*qdata->stamp, trunk_id, T_leaf_trunk, localized, false);

  /// Check if we should create a new vertex
  const auto& keyframe_test_result = *qdata->keyframe_test_result;
  if (keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    /// Add new vertex to the posegraph
    bool first_keyframe = !current_vertex_id_.isValid();
    if (!current_vertex_id_.isValid())
      addDanglingVertex(*(qdata->stamp));
    else
      addConnectedVertex(*(qdata->stamp), *(qdata->T_r_m_odo), false);
    (void)first_keyframe;
    CLOG(INFO, "tactic") << "Creating a new keyframe with id "
                         << current_vertex_id_;

    /// Update live id to the just-created vertex id and T_r_m_odo
    qdata->live_id.emplace(current_vertex_id_);
    qdata->T_r_m_odo.emplace(true);  // identity with zero covariance

    /// Call the pipeline to process the keyframe
    pipeline_->processKeyframe(qdata, output_, graph_, task_queue_);

    /// Set the new petiole without updating trunk since we are in branch mode
    chain_->setPetiole(current_vertex_id_);

    // /// Reset the map to robot transform and new vertex flag
    // updatePersistentLoc(*qdata->stamp, current_vertex_id_,
    // EdgeTransform(true), true);

    /// Compute odometry and localization in world frame for visualization
    {
      auto lock = chain_->guard();
      T_w_m_odo_ = chain_->T_start_petiole();
      keyframe_poses_.emplace_back();
      keyframe_poses_.back().pose =
          tf2::toMsg(Eigen::Affine3d(T_w_m_odo_.matrix()));
      if (keyframe_poses_.size() - 1 != current_vertex_id_.minorId()) {
        std::string err{
            "Number of keyframe poses does not match the live vertex minor "
            "id. This will cause localization thread to update wrong keyframe "
            "pose."};
        CLOG(ERROR, "tactic") << err;
        throw std::runtime_error{err};
      }
      T_w_m_loc_ = chain_->T_start_trunk();
    }
  }

  // Initialize localization
  const auto [map_id, map_sid, T_r_m_loc] = [&]() {
    auto lock = chain_->guard();
    EdgeTransform T_r_m_loc;
    if (!chain_->isLocalized()) {
      const Eigen::Matrix4d temp = Eigen::Matrix4d::Identity(4, 4);
      const Eigen::Matrix<double, 6, 6> loc_cov = config_->default_loc_cov;
      T_r_m_loc = EdgeTransform(temp, loc_cov);
    } else {
      T_r_m_loc = chain_->T_leaf_trunk();
    }

    return std::make_tuple(chain_->trunkVertexId(), chain_->trunkSequenceId(),
                           T_r_m_loc);
  }();
  qdata->map_id.emplace(map_id);
  qdata->map_sid.emplace(map_sid);
  qdata->T_r_m_loc.emplace(T_r_m_loc);

  return config_->localization_skippable;
}

bool TacticV2::runLocalization_(const QueryCache::Ptr& qdata) {
  switch (pipeline_mode_) {
    /// \note There are lots of repetitive code in the following four functions,
    /// maybe we can combine them at some point, but for now, consider leaving
    /// them separate so that it is slightly more clear what is happening during
    /// each pipeline mode.
    case PipelineMode::Branching:
      return branchLocalization(qdata);
    case PipelineMode::Merging:
      return mergeLocalization(qdata);
    case PipelineMode::Searching:
      return searchLocalization(qdata);
    case PipelineMode::Following:
      return followLocalization(qdata);
    default:
      return true;
  }
  return true;
}

bool TacticV2::branchLocalization(const QueryCache::Ptr& qdata) {
  (void)qdata;
  return true;
}

bool TacticV2::mergeLocalization(const QueryCache::Ptr& qdata) {
  (void)qdata;
  return true;
}

bool TacticV2::searchLocalization(const QueryCache::Ptr& qdata) {
  (void)qdata;
  return true;
}

bool TacticV2::followLocalization(const QueryCache::Ptr& qdata) {
  if (*qdata->keyframe_test_result != KeyframeTestResult::CREATE_VERTEX &&
      config_->localization_only_keyframe)
    return true;

  qdata->loc_success.emplace(false);

  // Prior is set in odometry and mapping thread
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to map vertex ("
                        << *(qdata->map_id) << ") (i.e., T_m_r localization): "
                        << (*qdata->T_r_m_loc).inverse().vec().transpose();

  // Run the localizer against the closest vertex
  pipeline_->runLocalization(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic") << "Estimated transformation from robot to map vertex ("
                        << *(qdata->map_id) << ") (i.e., T_m_r localization): "
                        << (*qdata->T_r_m_loc).inverse().vec().transpose();

  if (!(*qdata->loc_success)) {
    CLOG(DEBUG, "tactic") << "Localization failed, skip updating pose graph "
                             "and localization chain.";
    CLOG(DEBUG, "tactic") << "Finish running localization in follow.";
    return true;
  }

  // store localization result
  {
    const auto curr_run = graph_->runs()->sharedLocked().get().rbegin()->second;
    CLOG(DEBUG, "tactic") << "Saving localization result to run "
                          << curr_run->id();
    using LocResLM = storage::LockableMessage<LocalizationResult>;
    auto loc_result = std::make_shared<LocalizationResult>(
        *qdata->stamp, graph_->at(*qdata->map_id)->keyframeTime(),
        *qdata->map_id, *qdata->T_r_m_loc);
    auto msg = std::make_shared<LocResLM>(loc_result, *qdata->stamp);
    curr_run->write<LocalizationResult>("localization_result",
                                        "vtr_msgs/msg/LocalizationResult", msg);
  }

  const auto T_l_m = (*qdata->T_r_m_odo).inverse() * (*qdata->T_r_m_loc);
  CLOG(DEBUG, "tactic") << "Estimated transformation from live vertex "
                        << *qdata->live_id << " to map vertex "
                        << *qdata->map_id << " (i.e., T_m_l): "
                        << T_l_m.inverse().vec().transpose();

  // update the pose graph
  auto edge_id =
      EdgeId(*(qdata->live_id), *(qdata->map_id), pose_graph::Spatial);
  if (graph_->contains(edge_id)) {
    graph_->at(edge_id)->setTransform(T_l_m.inverse());
  } else {
    CLOG(DEBUG, "tactic") << "Adding a spatial edge between "
                          << *(qdata->live_id) << " and " << *(qdata->map_id)
                          << " to the graph.";
    graph_->addEdge(*(qdata->live_id), *(qdata->map_id), pose_graph::Spatial,
                    T_l_m.inverse(), false);
    CLOG(DEBUG, "tactic") << "Done adding the spatial edge between "
                          << *(qdata->live_id) << " and " << *(qdata->map_id)
                          << " to the graph.";
  }

  // Update the transform
  chain_->updateBranchToTwigTransform(*qdata->live_id, *qdata->map_id,
                                      *qdata->map_sid, T_l_m, true, false);

  // Correct keyfram pose (for visualization)
  {
    auto lock = chain_->guard();
    T_w_m_odo_ = chain_->T_start_petiole();
    keyframe_poses_[(*qdata->live_id).minorId()].pose =
        tf2::toMsg(Eigen::Affine3d(chain_->T_start_twig().matrix()));
    T_w_m_loc_ = chain_->T_start_trunk();
  }

  if (config_->visualize) {
    callback_->publishLocalizationRviz(*this, *qdata);
    pipeline_->visualizeLocalization(qdata, output_, graph_, task_queue_);
  }

  return true;
}

void TacticV2::addDanglingVertex(const storage::Timestamp& stamp) {
  /// Add the new vertex
  auto vertex = graph_->addVertex(stamp);
  current_vertex_id_ = vertex->id();
}

void TacticV2::addConnectedVertex(const storage::Timestamp& stamp,
                                  const EdgeTransform& T_r_m,
                                  const bool manual) {
  /// Add the new vertex
  auto previous_vertex_id = current_vertex_id_;
  addDanglingVertex(stamp);

  /// Add connection
  (void)graph_->addEdge(previous_vertex_id, current_vertex_id_,
                        pose_graph::Temporal, T_r_m, manual);
}

void TacticV2::updatePersistentLoc(const storage::Timestamp& t,
                                   const VertexId& v,
                                   const EdgeTransform& T_r_v, bool localized,
                                   bool reset_success) {
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  persistent_loc_.stamp = t;
  persistent_loc_.v = v;
  persistent_loc_.T = T_r_v;
  persistent_loc_.localized = localized;
  if (reset_success) persistent_loc_.successes = 0;
  callback_->publishRobotUI(*this);
}

void TacticV2::updateTargetLoc(const storage::Timestamp& t, const VertexId& v,
                               const EdgeTransform& T_r_v, bool localized,
                               bool reset_success) {
  std::lock_guard<std::mutex> lock(robot_state_mutex_);
  target_loc_.stamp = t;
  target_loc_.v = v;
  target_loc_.T = T_r_v;
  target_loc_.localized = localized;
  if (reset_success) target_loc_.successes = 0;
  callback_->publishRobotUI(*this);
}

}  // namespace tactic
}  // namespace vtr