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
 * \file tactic.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_tactic/tactic.hpp"

#include "vtr_tactic/storables.hpp"

namespace vtr {
namespace tactic {

auto Tactic::Config::fromROS(const rclcpp::Node::SharedPtr& node,
                             const std::string& prefix) -> UniquePtr {
  auto config = std::make_unique<Config>();
  // clang-format off
  /// setup tactic
  config->enable_parallelization = node->declare_parameter<bool>(prefix+".enable_parallelization", false);
  config->preprocessing_skippable = node->declare_parameter<bool>(prefix+".preprocessing_skippable", false);
  config->odometry_mapping_skippable = node->declare_parameter<bool>(prefix+".odometry_mapping_skippable", false);
  config->localization_skippable = node->declare_parameter<bool>(prefix+".localization_skippable", false);

  config->task_queue_num_threads = node->declare_parameter<int>(prefix+".task_queue_num_threads", 1);
  config->task_queue_size = node->declare_parameter<int>(prefix+".task_queue_size", -1);

  config->route_completion_translation_threshold = node->declare_parameter<double>(prefix+".route_completion_translation_threshold", 0.5);

  /// setup localization chain
  config->chain_config.min_cusp_distance = node->declare_parameter<double>(prefix+".chain.min_cusp_distance", 1.5);
  config->chain_config.angle_weight = node->declare_parameter<double>(prefix+".chain.angle_weight", 7.0);
  config->chain_config.search_depth = node->declare_parameter<int>(prefix+".chain.search_depth", 20);
  config->chain_config.search_back_depth = node->declare_parameter<int>(prefix+".chain.search_back_depth", 10);
  config->chain_config.distance_warning = node->declare_parameter<double>(prefix+".chain.distance_warning", 3);

  config->save_odometry_result = node->declare_parameter<bool>(prefix+".save_odometry_result", false);
  config->save_odometry_vel_result = node->declare_parameter<bool>(prefix+".save_odometry_vel_result", false);
  config->save_localization_result = node->declare_parameter<bool>(prefix+".save_localization_result", false);
  config->visualize = node->declare_parameter<bool>(prefix+".visualize", false);
  // clang-format on
  return config;
}

Tactic::Tactic(Config::UniquePtr config, const BasePipeline::Ptr& pipeline,
               const OutputCache::Ptr& output, const Graph::Ptr& graph,
               const Callback::Ptr& callback,
               const TaskQueueCallback::Ptr& task_queue_callback)
    : PipelineInterface(config->enable_parallelization, output, graph,
                        config->task_queue_num_threads, config->task_queue_size,
                        task_queue_callback),
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

auto Tactic::lockPipeline() -> TacticInterface::PipelineLock {
  return PipelineInterface::lockPipeline();
}

void Tactic::setPipeline(const PipelineMode& pipeline_mode) {
  CLOG(INFO, "tactic") << "Setting pipeline mode to " << pipeline_mode;
  pipeline_mode_ = pipeline_mode;
}

void Tactic::addRun(const bool) {
  graph_->addRun();
  // re-initialize the run
  first_frame_ = true;

  // \todo change to VertexId::Invalid():
  current_vertex_id_ = VertexId((uint64_t)-1);
  // re-initialize the localization chain (path will be added later)
  chain_->reset();
  // re-initialize the pose records for visualization
  T_w_v_odo_ = EdgeTransform(true);
  T_w_v_loc_ = EdgeTransform(true);
  // re-initialize the pipeline
  pipeline_->reset();
  //
  callback_->startRun();
}

void Tactic::finishRun() {
  // saving graph here is optional as we save at destruction, just to avoid
  // unexpected data loss
  graph_->save();
  //
  callback_->endRun();
}

void Tactic::setPath(const VertexId::Vector& path, const unsigned& trunk_sid,
                     const EdgeTransform& T_twig_branch, const bool publish) {
  /// Set path and target localization
  CLOG(INFO, "tactic") << "Set path of size " << path.size();
  ///
  auto lock = chain_->guard();
  //
  chain_->setSequence(path);
  if (path.size() > 0) chain_->expand();
  // used as initial guess for trunk
  chain_->resetTrunk(trunk_sid);
  // used as initial guess for localization
  chain_->initializeBranchToTwigTransform(T_twig_branch);
  CLOG(INFO, "tactic") << "Setting sequence to " << chain_->sequence()
                       << " with initial trunk sid: " << trunk_sid
                       << ", and initial Twig to Branch transform: "
                       << T_twig_branch.inverse().vec().transpose();
  //
  if (publish) {
    callback_->pathUpdated(path);
    callback_->publishPathRviz(*chain_);
  }
}

void Tactic::setTrunk(const VertexId& v) {
  CLOG(DEBUG, "tactic") << "Setting persistent loc vertex to " << v;
  RobotStateGuard lock(robot_state_mutex_);
  if (v.isValid()) persistent_loc_ = Localization(v);
  target_loc_ = Localization();
  callback_->robotStateUpdated(persistent_loc_, target_loc_);
}

void Tactic::connectToTrunk(const bool privileged) {
  const auto [twig_vid, branch_vid, T_twig_branch] = [&]() {
    auto lock = chain_->guard();
    return std::make_tuple(chain_->twigVertexId(), chain_->branchVertexId(),
                           chain_->T_twig_branch());
  }();
  CLOG(INFO, "tactic") << "Adding connection " << twig_vid << " --> "
                       << branch_vid << ", privileged: " << std::boolalpha
                       << privileged << ", with T_to_from: "
                       << T_twig_branch.inverse().vec().transpose();
  graph_->addEdge(twig_vid, branch_vid, EdgeType::Spatial, privileged,
                  T_twig_branch.inverse());
}

auto Tactic::getPersistentLoc() const -> Localization {
  RobotStateGuard lock(robot_state_mutex_);
  return persistent_loc_;
}

bool Tactic::isLocalized() const { return chain_->isLocalized(); }

bool Tactic::passedSeqId(const uint64_t& sid) const {
  return chain_->trunkSequenceId() > sid;
}

bool Tactic::routeCompleted() const {
  auto lock = chain_->guard();
  const auto translation = chain_->T_leaf_trunk().r_ab_inb().norm();

  if (chain_->trunkSequenceId() < (chain_->sequence().size() - 2)) {
    return false;
  }

  if (translation > config_->route_completion_translation_threshold) {
    return false;
  }
  
  return true;
}

bool Tactic::input_(const QueryCache::Ptr&) {
  return config_->preprocessing_skippable;
}

bool Tactic::preprocess_(const QueryCache::Ptr& qdata) {
  // Setup caches
  qdata->pipeline_mode.emplace(pipeline_mode_);
  qdata->first_frame.emplace(first_frame_);
  first_frame_ = false;

  // Preprocess incoming data, which always runs no matter what mode we are in.
  pipeline_->preprocess(qdata, output_, graph_, task_queue_);

  return config_->odometry_mapping_skippable;
}

bool Tactic::runOdometryMapping_(const QueryCache::Ptr& qdata) {
  // Setup caches
  qdata->vid_odo.emplace(current_vertex_id_);
  qdata->vertex_test_result.emplace(VertexTestResult::DO_NOTHING);
  qdata->odo_success.emplace(false);

  switch (pipeline_mode_) {
    /// \note There are lots of repetitive code in the following four functions,
    /// maybe we can combine them at some point, but for now, consider leaving
    /// them separate so that it is slightly more clear what is happening during
    /// each pipeline mode.
    case PipelineMode::TeachMetricLoc:
      return teachMetricLocOdometryMapping(qdata);
    case PipelineMode::TeachBranch:
      return teachBranchOdometryMapping(qdata);
    case PipelineMode::TeachMerge:
      return teachMergeOdometryMapping(qdata);
    case PipelineMode::RepeatMetricLoc:
      return repeatMetricLocOdometryMapping(qdata);
    case PipelineMode::RepeatFollow:
      return repeatFollowOdometryMapping(qdata);
    default:
      return true;
  }
  return true;
}

bool Tactic::teachMetricLocOdometryMapping(const QueryCache::Ptr& qdata) {
  // Prior assumes no motion since last processed frame
  qdata->T_r_v_odo.emplace(chain_->T_leaf_petiole());
  qdata->w_v_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to odometry vertex"
                        << *qdata->vid_odo << " (i.e., T_v_r odometry): "
                        << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // Get relative pose estimate and whether a vertex should be created
  pipeline_->runOdometry(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to odometry vertex"
      << *qdata->vid_odo << " (i.e., T_v_r odometry): "
      << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // save odometry result
  if (config_->save_odometry_result) {
    CLOG(DEBUG, "tactic") << "Saving odometry result";
    using OdoResLM = storage::LockableMessage<OdometryResult>;
    auto odo_result = std::make_shared<OdometryResult>(
        *qdata->stamp, T_w_v_odo_ * (*qdata->T_r_v_odo).inverse());
    auto msg = std::make_shared<OdoResLM>(odo_result, *qdata->stamp);
    graph_->write<OdometryResult>("odometry_result",
                                  "vtr_tactic_msgs/msg/OdometryResult", msg);
  }

  // save odometry velocity result
  if (config_->save_odometry_vel_result) {
    CLOG(DEBUG, "tactic") << "Saving odometry velocity result";
    using TwistLM = storage::LockableMessage<geometry_msgs::msg::Twist>;
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();

    // Populate Twist message
    auto vel = *qdata->w_v_r_in_r_odo;
    twist_msg->linear.x = vel(0, 0);
    twist_msg->linear.y = vel(1, 0);
    twist_msg->linear.z = vel(2, 0);
    twist_msg->angular.x = vel(3, 0);
    twist_msg->angular.y = vel(4, 0);
    twist_msg->angular.z = vel(5, 0);

    auto msg = std::make_shared<TwistLM>(twist_msg, *qdata->stamp);
    graph_->write<geometry_msgs::msg::Twist>(
      "odometry_vel_result", "geometry_msgs/msg/Twist",
      msg);
  }

  // Rviz visualization
  if (config_->visualize) {
    const auto lock = chain_->guard();
    callback_->publishOdometryRviz(*qdata->stamp, *qdata->T_r_v_odo,
                                   T_w_v_odo_);
  }

  // Update odometry in localization chain without updating trunk (because in
  // branch mode there's no trunk to localize against)
  chain_->updatePetioleToLeafTransform(*qdata->stamp, *qdata->w_v_r_in_r_odo,
                                       *qdata->T_r_v_odo, false);

  // Update persistent localization (only when the first vertex has been
  // created so that current vertex id is valid.)
  // localized is always set to true, since we are in teach mode
  if (current_vertex_id_.isValid())
    updatePersistentLoc(*qdata->stamp, *qdata->vid_odo, *qdata->T_r_v_odo,
                        true);

  // Check if we should create a new vertex
  const auto& vertex_test_result = *qdata->vertex_test_result;
  if (vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    // Add new vertex to the posegraph
    addVertexEdge(*(qdata->stamp), *(qdata->T_r_v_odo), true,
                  *(qdata->env_info));
    CLOG(INFO, "tactic") << "Creating a new vertex with id "
                         << current_vertex_id_;

    // Compute odometry vertex frame in world frame for visualization.
    T_w_v_odo_ = T_w_v_odo_ * (*qdata->T_r_v_odo).inverse();

    // Update live id to the just-created vertex id and T_r_v_odo
    qdata->vid_odo = current_vertex_id_;
    qdata->T_r_v_odo = EdgeTransform(true);

    // Call the pipeline to process the vertex
    pipeline_->onVertexCreation(qdata, output_, graph_, task_queue_);

    // Set the new petiole without updating trunk since we are in branch mode
    chain_->setPetiole(current_vertex_id_);

    // Reset the map to robot transform and new vertex flag
    updatePersistentLoc(*qdata->stamp, current_vertex_id_, EdgeTransform(true),
                        true);
  }

  // Initialize localization
  auto lock = chain_->guard();
  qdata->vid_loc.emplace(chain_->trunkVertexId());
  qdata->sid_loc.emplace(chain_->trunkSequenceId());
  qdata->T_r_v_loc.emplace(chain_->T_leaf_trunk());

  return config_->localization_skippable;
}

bool Tactic::teachBranchOdometryMapping(const QueryCache::Ptr& qdata) {
  // Prior assumes no motion since last processed frame
  qdata->T_r_v_odo.emplace(chain_->T_leaf_petiole());
  qdata->w_v_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to odometry vertex"
                        << *qdata->vid_odo << " (i.e., T_v_r odometry): "
                        << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // Get relative pose estimate and whether a vertex should be created
  pipeline_->runOdometry(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to odometry vertex"
      << *qdata->vid_odo << " (i.e., T_v_r odometry): "
      << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // save odometry result
  if (config_->save_odometry_result) {
    CLOG(DEBUG, "tactic") << "Saving odometry result";
    using OdoResLM = storage::LockableMessage<OdometryResult>;
    auto odo_result = std::make_shared<OdometryResult>(
        *qdata->stamp, T_w_v_odo_ * (*qdata->T_r_v_odo).inverse());
    auto msg = std::make_shared<OdoResLM>(odo_result, *qdata->stamp);
    graph_->write<OdometryResult>("odometry_result",
                                  "vtr_tactic_msgs/msg/OdometryResult", msg);
  }

  // save odometry velocity result
  if (config_->save_odometry_vel_result) {
    CLOG(DEBUG, "tactic") << "Saving odometry velocity result";
    using TwistLM = storage::LockableMessage<geometry_msgs::msg::Twist>;
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();

    // Populate Twist message
    auto vel = *qdata->w_v_r_in_r_odo;
    twist_msg->linear.x = vel(0, 0);
    twist_msg->linear.y = vel(1, 0);
    twist_msg->linear.z = vel(2, 0);
    twist_msg->angular.x = vel(3, 0);
    twist_msg->angular.y = vel(4, 0);
    twist_msg->angular.z = vel(5, 0);

    auto msg = std::make_shared<TwistLM>(twist_msg, *qdata->stamp);
    graph_->write<geometry_msgs::msg::Twist>(
      "odometry_vel_result", "geometry_msgs/msg/Twist",
      msg);
  }


  // Rviz visualization
  if (config_->visualize) {
    const auto lock = chain_->guard();
    callback_->publishOdometryRviz(*qdata->stamp, *qdata->T_r_v_odo,
                                   T_w_v_odo_);
  }

  // Update odometry in localization chain without updating trunk (because in
  // teachMetricLoc mode we only try to localize against one vertex)
  chain_->updatePetioleToLeafTransform(*qdata->stamp, *qdata->w_v_r_in_r_odo,
                                       *qdata->T_r_v_odo, false);

  // Update persistent localization (only when the first vertex has been
  // created so that current vertex id is valid.)
  if (current_vertex_id_.isValid())
    updatePersistentLoc(*qdata->stamp, *qdata->vid_odo, *qdata->T_r_v_odo,
                        true);

  // Check if we should create a new vertex
  const auto& vertex_test_result = *qdata->vertex_test_result;
  if (vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    // Add new vertex to the posegraph
    addVertexEdge(*(qdata->stamp), *(qdata->T_r_v_odo), true,
                  *(qdata->env_info));
    CLOG(INFO, "tactic") << "Creating a new vertex with id "
                         << current_vertex_id_;

    // Compute odometry in world frame for visualization.
    T_w_v_odo_ = T_w_v_odo_ * (*qdata->T_r_v_odo).inverse();

    // Update live id to the just-created vertex id and T_r_v_odo
    qdata->vid_odo = current_vertex_id_;
    qdata->T_r_v_odo = EdgeTransform(true);

    // Call the pipeline to process the vertex
    pipeline_->onVertexCreation(qdata, output_, graph_, task_queue_);

    // Set the new petiole without updating trunk since we are in branch mode
    chain_->setPetiole(current_vertex_id_);

    // Reset the map to robot transform and new vertex flag
    updatePersistentLoc(*qdata->stamp, current_vertex_id_, EdgeTransform(true),
                        true);
  }

  return config_->localization_skippable;
}

bool Tactic::teachMergeOdometryMapping(const QueryCache::Ptr& qdata) {
  // Prior assumes no motion since last processed frame
  qdata->T_r_v_odo.emplace(chain_->T_leaf_petiole());
  qdata->w_v_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to odometry vertex"
                        << *qdata->vid_odo << " (i.e., T_v_r odometry): "
                        << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // Get relative pose estimate and whether a vertex should be created
  pipeline_->runOdometry(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to odometry vertex"
      << *qdata->vid_odo << " (i.e., T_v_r odometry): "
      << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // save odometry result
  if (config_->save_odometry_result) {
    CLOG(DEBUG, "tactic") << "Saving odometry result";
    using OdoResLM = storage::LockableMessage<OdometryResult>;
    auto odo_result = std::make_shared<OdometryResult>(
        *qdata->stamp, T_w_v_odo_ * (*qdata->T_r_v_odo).inverse());
    auto msg = std::make_shared<OdoResLM>(odo_result, *qdata->stamp);
    graph_->write<OdometryResult>("odometry_result",
                                  "vtr_tactic_msgs/msg/OdometryResult", msg);
  }

  // save odometry velocity result
  if (config_->save_odometry_vel_result) {
    CLOG(DEBUG, "tactic") << "Saving odometry velocity result";
    using TwistLM = storage::LockableMessage<geometry_msgs::msg::Twist>;
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();

    // Populate Twist message
    auto vel = *qdata->w_v_r_in_r_odo;
    twist_msg->linear.x = vel(0, 0);
    twist_msg->linear.y = vel(1, 0);
    twist_msg->linear.z = vel(2, 0);
    twist_msg->angular.x = vel(3, 0);
    twist_msg->angular.y = vel(4, 0);
    twist_msg->angular.z = vel(5, 0);

    auto msg = std::make_shared<TwistLM>(twist_msg, *qdata->stamp);
    graph_->write<geometry_msgs::msg::Twist>(
      "odometry_vel_result", "geometry_msgs/msg/Twist",
      msg);
  }

  // Rviz visualization
  if (config_->visualize) {
    const auto lock = chain_->guard();
    callback_->publishOdometryRviz(*qdata->stamp, *qdata->T_r_v_odo,
                                   T_w_v_odo_);
  }

  // Update odometry in localization chain without updating trunk (because in
  // branch mode there's no trunk to localize against)
  chain_->updatePetioleToLeafTransform(*qdata->stamp, *qdata->w_v_r_in_r_odo,
                                       *qdata->T_r_v_odo, false);

  // Update persistent localization (only when the first vertex has been
  // created so that current vertex id is valid.)
  // localized is always set to true, since we are in teach mode
  if (current_vertex_id_.isValid())
    updatePersistentLoc(*qdata->stamp, *qdata->vid_odo, *qdata->T_r_v_odo,
                        true);

  // Check if we should create a new vertex
  const auto& vertex_test_result = *qdata->vertex_test_result;
  if (vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    // Add new vertex to the posegraph
    addVertexEdge(*(qdata->stamp), *(qdata->T_r_v_odo), true,
                  *(qdata->env_info));
    CLOG(INFO, "tactic") << "Creating a new vertex with id "
                         << current_vertex_id_;

    // Compute odometry in world frame for visualization.
    T_w_v_odo_ = T_w_v_odo_ * (*qdata->T_r_v_odo).inverse();

    // Update live id to the just-created vertex id and T_r_v_odo
    qdata->vid_odo = current_vertex_id_;
    qdata->T_r_v_odo = EdgeTransform(true);

    // Call the pipeline to process the vertex
    pipeline_->onVertexCreation(qdata, output_, graph_, task_queue_);

    // Set the new petiole without updating trunk since we are in branch mode
    chain_->setPetiole(current_vertex_id_);

    // Reset the map to robot transform and new vertex flag
    updatePersistentLoc(*qdata->stamp, current_vertex_id_, EdgeTransform(true),
                        true);
  }

  // Initialize localization
  auto lock = chain_->guard();
  qdata->vid_loc.emplace(chain_->trunkVertexId());
  qdata->sid_loc.emplace(chain_->trunkSequenceId());
  qdata->T_r_v_loc.emplace(chain_->T_leaf_trunk());

  return config_->localization_skippable;
}

bool Tactic::repeatMetricLocOdometryMapping(const QueryCache::Ptr& qdata) {
  // Prior assumes no motion since last processed frame
  qdata->T_r_v_odo.emplace(chain_->T_leaf_petiole());
  qdata->w_v_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to odometry vertex"
                        << *qdata->vid_odo << " (i.e., T_v_r odometry): "
                        << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // Get relative pose estimate and whether a vertex should be created
  pipeline_->runOdometry(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to odometry vertex"
      << *qdata->vid_odo << " (i.e., T_v_r odometry): "
      << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // Rviz visualization
  if (config_->visualize) {
    const auto lock = chain_->guard();
    callback_->publishOdometryRviz(*qdata->stamp, *qdata->T_r_v_odo,
                                   T_w_v_odo_);
  }

  // Update odometry in localization chain, also update estimated closest
  // trunk without looking backwards
  chain_->updatePetioleToLeafTransform(*qdata->stamp, *qdata->w_v_r_in_r_odo,
                                       *qdata->T_r_v_odo, true, false);

  // Update persistent localization, "isLocalized" says whether we have
  // localized yet
  const auto [trunk_vid, T_leaf_trunk, localized] = [&]() {
    auto lock = chain_->guard();
    return std::make_tuple(chain_->trunkVertexId(), chain_->T_leaf_trunk(),
                           chain_->isLocalized());
  }();
  updatePersistentLoc(*qdata->stamp, trunk_vid, T_leaf_trunk, localized);

  // Check if we should create a new vertex
  const auto& vertex_test_result = *qdata->vertex_test_result;
  if (vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    // Add new vertex to the posegraph
    addVertexEdge(*(qdata->stamp), *(qdata->T_r_v_odo), false,
                  *(qdata->env_info));
    CLOG(INFO, "tactic") << "Creating a new vertex with id "
                         << current_vertex_id_;

    // Update live id to the just-created vertex id and T_r_v_odo
    qdata->vid_odo = current_vertex_id_;
    qdata->T_r_v_odo = EdgeTransform(true);  // identity with zero covariance

    // Call the pipeline to process the vertex
    pipeline_->onVertexCreation(qdata, output_, graph_, task_queue_);

    // Set the new petiole without updating trunk since we are in branch mode
    chain_->setPetiole(current_vertex_id_);

    // Compute odometry and localization in world frame for visualization
    auto lock = chain_->guard();
    T_w_v_odo_ = chain_->T_start_petiole();
    T_w_v_loc_ = chain_->T_start_trunk();
  }

  // Initialize localization
  auto lock = chain_->guard();
  qdata->vid_loc.emplace(chain_->trunkVertexId());
  qdata->sid_loc.emplace(chain_->trunkSequenceId());
  qdata->T_r_v_loc.emplace(chain_->T_leaf_trunk());

  return config_->localization_skippable;
}

bool Tactic::repeatFollowOdometryMapping(const QueryCache::Ptr& qdata) {
  // Prior assumes no motion since last processed frame
  qdata->T_r_v_odo.emplace(chain_->T_leaf_petiole());
  qdata->w_v_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
  CLOG(DEBUG, "tactic") << "Prior transformation from robot to odometry vertex"
                        << *qdata->vid_odo << " (i.e., T_v_r odometry): "
                        << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // Get relative pose estimate and whether a vertex should be created
  pipeline_->runOdometry(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to odometry vertex"
      << *qdata->vid_odo << " (i.e., T_v_r odometry): "
      << (*qdata->T_r_v_odo).inverse().vec().transpose();

  // Rviz visualization
  if (config_->visualize) {
    const auto lock = chain_->guard();
    callback_->publishOdometryRviz(*qdata->stamp, *qdata->T_r_v_odo,
                                   T_w_v_odo_);
  }

  // Update odometry in localization chain, also update estimated closest
  // trunk without looking backwards
  chain_->updatePetioleToLeafTransform(*qdata->stamp, *qdata->w_v_r_in_r_odo,
                                       *qdata->T_r_v_odo, true, false);

  // Update persistent localization, "isLocalized" says whether we have
  // localized yet
  const auto [trunk_vid, T_leaf_trunk, localized] = [&]() {
    auto lock = chain_->guard();
    return std::make_tuple(chain_->trunkVertexId(), chain_->T_leaf_trunk(),
                           chain_->isLocalized());
  }();
  updatePersistentLoc(*qdata->stamp, trunk_vid, T_leaf_trunk, localized);

  // Check if we should create a new vertex
  const auto& vertex_test_result = *qdata->vertex_test_result;
  if (vertex_test_result == VertexTestResult::CREATE_VERTEX) {
    // Add new vertex to the posegraph
    addVertexEdge(*(qdata->stamp), *(qdata->T_r_v_odo), false,
                  *(qdata->env_info));
    CLOG(INFO, "tactic") << "Creating a new vertex with id "
                         << current_vertex_id_;

    // Update live id to the just-created vertex id and T_r_v_odo
    qdata->vid_odo = current_vertex_id_;
    qdata->T_r_v_odo = EdgeTransform(true);  // identity with zero covariance

    // Call the pipeline to process the vertex
    pipeline_->onVertexCreation(qdata, output_, graph_, task_queue_);

    // Set the new petiole without updating trunk since we are in branch mode
    chain_->setPetiole(current_vertex_id_);

    // Compute odometry and localization in world frame for visualization
    auto lock = chain_->guard();
    T_w_v_odo_ = chain_->T_start_petiole();
    T_w_v_loc_ = chain_->T_start_trunk();
  }

  // Initialize localization
  auto lock = chain_->guard();
  qdata->vid_loc.emplace(chain_->trunkVertexId());
  qdata->sid_loc.emplace(chain_->trunkSequenceId());
  qdata->T_r_v_loc.emplace(chain_->T_leaf_trunk());

  return config_->localization_skippable;
}

bool Tactic::runLocalization_(const QueryCache::Ptr& qdata) {
  switch (pipeline_mode_) {
    /// \note There are lots of repetitive code in the following four functions,
    /// maybe we can combine them at some point, but for now, consider leaving
    /// them separate so that it is slightly more clear what is happening during
    /// each pipeline mode.
    case PipelineMode::TeachMetricLoc:
      return teachMetricLocLocalization(qdata);
    case PipelineMode::TeachBranch:
      return teachBranchLocalization(qdata);
    case PipelineMode::TeachMerge:
      return teachMergeLocalization(qdata);
    case PipelineMode::RepeatMetricLoc:
      return repeatMetricLocLocalization(qdata);
    case PipelineMode::RepeatFollow:
      return repeatFollowLocalization(qdata);
    default:
      return true;
  }
  return true;
}

bool Tactic::teachMetricLocLocalization(const QueryCache::Ptr& qdata) {
  // Prior is set in odometry and mapping thread
  CLOG(DEBUG, "tactic")
      << "Prior transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();

  // Run the localizer against the closest vertex
  qdata->loc_success.emplace(false);
  pipeline_->runLocalization(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();
  if (!(*qdata->loc_success)) {
    CLOG(WARNING, "tactic") << "Localization failed, skip updating pose graph "
                               "and localization chain.";
    chain_->lostLocalization();
    return true;
  }

  // Compute map vertex to odometry vertex transform (i.e., subtract odometry)
  const auto T_v_odo_loc = (*qdata->T_r_v_odo).inverse() * (*qdata->T_r_v_loc);
  CLOG(DEBUG, "tactic") << "Estimated transformation from odometry vertex "
                        << *qdata->vid_odo << " to localization vertex "
                        << *qdata->vid_loc << " (i.e., T_v_loc_odo): "
                        << T_v_odo_loc.inverse().vec().transpose();

  // Update the transform
  chain_->updateBranchToTwigTransform(*qdata->vid_odo, *qdata->vid_loc,
                                      *qdata->sid_loc, T_v_odo_loc, false);

  return true;
}

bool Tactic::teachBranchLocalization(const QueryCache::Ptr& qdata) {
  (void)qdata;  // nothing to do in branch
  return true;
}

bool Tactic::teachMergeLocalization(const QueryCache::Ptr& qdata) {
  // Prior is set in odometry and mapping thread
  CLOG(DEBUG, "tactic")
      << "Prior transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();

  // Run the localizer against the closest vertex
  qdata->loc_success.emplace(false);
  pipeline_->runLocalization(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();
  if (!(*qdata->loc_success)) {
    CLOG(DEBUG, "tactic") << "Localization failed, skip updating pose graph "
                             "and localization chain.";
    chain_->lostLocalization();
    return true;
  }

  // Compute map vertex to odometry vertex transform (i.e., subtract odometry)
  const auto T_v_odo_loc = (*qdata->T_r_v_odo).inverse() * (*qdata->T_r_v_loc);
  CLOG(DEBUG, "tactic") << "Estimated transformation from odometry vertex "
                        << *qdata->vid_odo << " to localization vertex "
                        << *qdata->vid_loc << " (i.e., T_v_loc_odo): "
                        << T_v_odo_loc.inverse().vec().transpose();

  // Update the transform
  chain_->updateBranchToTwigTransform(*qdata->vid_odo, *qdata->vid_loc,
                                      *qdata->sid_loc, T_v_odo_loc, true, true);

  // Update the robot state
  updateTargetLoc(*qdata->stamp, *qdata->vid_loc, *qdata->T_r_v_loc, true);

  return true;
}

bool Tactic::repeatMetricLocLocalization(const QueryCache::Ptr& qdata) {
  // Prior is set in odometry and mapping thread
  CLOG(DEBUG, "tactic")
      << "Prior transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();

  // Run the localizer against the closest vertex
  qdata->loc_success.emplace(false);
  pipeline_->runLocalization(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();
  if (!(*qdata->loc_success)) {
    CLOG(DEBUG, "tactic") << "Localization failed, skip updating pose graph "
                             "and localization chain.";
    chain_->lostLocalization();

    return true;
  }

  // Compute map vertex to odometry vertex transform (i.e., subtract odometry)
  const auto T_v_odo_loc = (*qdata->T_r_v_odo).inverse() * (*qdata->T_r_v_loc);
  CLOG(DEBUG, "tactic") << "Estimated transformation from odometry vertex "
                        << *qdata->vid_odo << " to localization vertex "
                        << *qdata->vid_loc << " (i.e., T_v_loc_odo): "
                        << T_v_odo_loc.inverse().vec().transpose();

  // Update the transform
  chain_->updateBranchToTwigTransform(*qdata->vid_odo, *qdata->vid_loc,
                                      *qdata->sid_loc, T_v_odo_loc, true, true);

  return true;
}

bool Tactic::repeatFollowLocalization(const QueryCache::Ptr& qdata) {
  // Prior is set in odometry and mapping thread
  CLOG(DEBUG, "tactic")
      << "Prior transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();

  // Run the localizer against the closest vertex
  qdata->loc_success.emplace(false);
  pipeline_->runLocalization(qdata, output_, graph_, task_queue_);
  CLOG(DEBUG, "tactic")
      << "Estimated transformation from robot to localization vertex ("
      << *(qdata->vid_loc) << ") (i.e., T_v_r localization): "
      << (*qdata->T_r_v_loc).inverse().vec().transpose();

  // save localization result
  if (config_->save_localization_result) {
    CLOG(DEBUG, "tactic") << "Saving localization result";
    using LocResLM = storage::LockableMessage<LocalizationResult>;
    auto loc_result = std::make_shared<LocalizationResult>(
        *qdata->stamp, graph_->at(*qdata->vid_loc)->vertexTime(),
        *qdata->vid_loc, *qdata->T_r_v_loc);
    auto msg = std::make_shared<LocResLM>(loc_result, *qdata->stamp);
    graph_->write<LocalizationResult>(
        "localization_result", "vtr_tactic_msgs/msg/LocalizationResult", msg);
  }

  if (!(*qdata->loc_success)) {
    CLOG(WARNING, "tactic") << "Localization failed, skip updating pose graph "
                               "and localization chain.";
    return true;
  }

  const auto T_v_odo_loc = (*qdata->T_r_v_odo).inverse() * (*qdata->T_r_v_loc);
  CLOG(DEBUG, "tactic") << "Estimated transformation from odometry vertex "
                        << *qdata->vid_odo << " to localization vertex "
                        << *qdata->vid_loc << " (i.e., T_v_loc_odo): "
                        << T_v_odo_loc.inverse().vec().transpose();

  // update the pose graph
  auto edge_id = EdgeId(*(qdata->vid_odo), *(qdata->vid_loc));
  if (graph_->contains(edge_id)) {
    graph_->at(edge_id)->setTransform(T_v_odo_loc.inverse());
  } else {
    CLOG(DEBUG, "tactic") << "Adding a spatial edge between "
                          << *(qdata->vid_odo) << " and " << *(qdata->vid_loc)
                          << " to the graph.";
    graph_->addEdge(*(qdata->vid_odo), *(qdata->vid_loc), EdgeType::Spatial,
                    false, T_v_odo_loc.inverse());
    CLOG(DEBUG, "tactic") << "Done adding the spatial edge between "
                          << *(qdata->vid_odo) << " and " << *(qdata->vid_loc)
                          << " to the graph.";
  }

  // Update the transform
  chain_->updateBranchToTwigTransform(*qdata->vid_odo, *qdata->vid_loc,
                                      *qdata->sid_loc, T_v_odo_loc, true,
                                      false);

  // Correct keyfram pose (for visualization)
  auto lock = chain_->guard();
  T_w_v_odo_ = chain_->T_start_petiole();
  T_w_v_loc_ = chain_->T_start_trunk();
  //
  if (config_->visualize)
    callback_->publishLocalizationRviz(*qdata->stamp, T_w_v_loc_);

  return true;
}

void Tactic::addVertexEdge(const Timestamp& stamp, const EdgeTransform& T_r_v,
                           const bool manual, const EnvInfo& env_info) {
  //
  const auto previous_vertex_id = current_vertex_id_;

  // Add the new vertex
  auto vertex = graph_->addVertex(stamp);
  current_vertex_id_ = vertex->id();

  // Store environment info into the vertex
  using EnvInfoLM = storage::LockableMessage<EnvInfo>;
  const auto env_info_data = std::make_shared<EnvInfo>(env_info);
  const auto env_info_msg = std::make_shared<EnvInfoLM>(env_info_data, stamp);
  vertex->insert<EnvInfo>("env_info", "vtr_tactic_msgs/msg/EnvInfo", env_info_msg);

  // Store waypoint name info into the vertex
  using WaypointNameLM = storage::LockableMessage<WaypointName>;
  const auto waypoint_name_data = std::make_shared<WaypointName>();
  const auto waypoint_name_msg = std::make_shared<WaypointNameLM>(waypoint_name_data, stamp);
  vertex->insert<WaypointName>("waypoint_name", "vtr_tactic_msgs/msg/WaypointNames", waypoint_name_msg);

  // Add the new edge
  if (!previous_vertex_id.isValid()) return;
  (void)graph_->addEdge(previous_vertex_id, current_vertex_id_,
                        EdgeType::Temporal, manual, T_r_v);
}

void Tactic::updatePersistentLoc(const Timestamp& t, const VertexId& v,
                                 const EdgeTransform& T_r_v,
                                 const bool localized) {
  RobotStateGuard lock(robot_state_mutex_);
  persistent_loc_.stamp = t;
  persistent_loc_.v = v;
  persistent_loc_.T = T_r_v;
  persistent_loc_.localized = localized;
  callback_->robotStateUpdated(persistent_loc_, target_loc_);
}

void Tactic::updateTargetLoc(const Timestamp& t, const VertexId& v,
                             const EdgeTransform& T_r_v, const bool localized) {
  RobotStateGuard lock(robot_state_mutex_);
  target_loc_.stamp = t;
  target_loc_.v = v;
  target_loc_.T = T_r_v;
  target_loc_.localized = localized;
  callback_->robotStateUpdated(persistent_loc_, target_loc_);
}

}  // namespace tactic
}  // namespace vtr
