#include <vtr_tactic/tactic.hpp>

namespace vtr {
namespace tactic {

auto Tactic::Config::fromROS(const rclcpp::Node::SharedPtr node) -> const Ptr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // setup localization chain
  config->chain_config.min_cusp_distance = node->declare_parameter<double>("tactic.chain.min_cusp_distance", 1.5);
  config->chain_config.angle_weight = node->declare_parameter<double>("tactic.chain.angle_weight", 7.0);
  config->chain_config.search_depth = node->declare_parameter<int>("tactic.chain.search_depth", 20);
  config->chain_config.search_back_depth = node->declare_parameter<int>("tactic.chain.search_back_depth", 10);
  config->chain_config.distance_warning = node->declare_parameter<double>("tactic.chain.distance_warning", 3);
  // setup tactic
  auto dlc = node->declare_parameter<std::vector<double>>("tactic.default_loc_cov", std::vector<double>{});
  if (dlc.size() != 6) {
    LOG(WARNING) << "Tactic default localization covariance malformed ("
                 << dlc.size() << " elements). Must be 6 elements!";
  }
  // make at least size elements to prevent segfault
  if (dlc.size() < 6) dlc.resize(6, 1.);
  config->default_loc_cov.setZero();
  config->default_loc_cov.diagonal() << dlc[0], dlc[1], dlc[2], dlc[3], dlc[4], dlc[5];
  // clang-format on
  return config;
}

void Tactic::initializePipeline() {
  /// pipeline specific initialization
  pipeline_->initialize(mdata_, graph_);

  /// for visualization only
  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(*mdata_->node);
  odo_path_pub_ = (*mdata_->node)->create_publisher<ROSPathMsg>("odo_path", 10);
  loc_path_pub_ = (*mdata_->node)->create_publisher<ROSPathMsg>("loc_path", 10);
}

void Tactic::runPipeline(QueryCache::Ptr qdata) {
  /// Lock to make sure the pipeline does not change during processing
  LockType lck(pipeline_mutex_, std::defer_lock_t());
  // If we cannot lock in 30ms, give up and hope that the next frame will work
  if (!lck.try_lock_for(std::chrono::milliseconds(30))) {
    LOG(WARNING) << "[Tactic] Dropping frame due to unavailable pipeline mutex";
    return;
  }

  /// Preprocess incoming data
  LOG(INFO) << "[Tactic] Preprocessing incoming data.";
  pipeline_->preprocess(qdata, mdata_, graph_);

#ifdef DETERMINISTIC_VTR
  LOG(INFO) << "[Tactic] Finished preprocessing incoming data.";
  runPipeline_(qdata);
#else
  /// Run pipeline according to the state
  if (odometry_thread_future_.valid()) odometry_thread_future_.wait();
  LOG(INFO) << "[Tactic] Launching the odometry thread.";
  odometry_thread_future_ =
      std::async(std::launch::async, [this, qdata]() { runPipeline_(qdata); });
  LOG(INFO) << "[Tactic] Finished preprocessing incoming data.";
#endif
}

void Tactic::runPipeline_(QueryCache::Ptr qdata) {
  LOG(INFO) << "[Tactic] Running odometry on incoming data.";
  /// Setup caches
  qdata->steam_mutex.fallback(steam_mutex_ptr_);
  qdata->first_frame.fallback(first_frame_);
  qdata->live_id.fallback(current_vertex_id_);
  qdata->keyframe_test_result.fallback(KeyframeTestResult::DO_NOTHING);
  qdata->map_id.fallback(chain_.trunkVertexId());  /// \todo remove this

  switch (pipeline_mode_) {
    case PipelineMode::Idle:
      break;
    case PipelineMode::Branching:
      branch(qdata);
      break;
    case PipelineMode::Merging:
      merge(qdata);
      break;
    case PipelineMode::Searching:
      search(qdata);
      break;
    case PipelineMode::Following:
      follow(qdata);
      break;
  }
  LOG(INFO) << "[Tactic] Finished odometry & mapping on incoming data.";
}

void Tactic::branch(QueryCache::Ptr qdata) {
  /// \todo find a better default value for odometry
  qdata->T_r_m_odo.fallback(true);  // set covariance to zero

  /// Odometry to get relative pose estimate and whether a keyframe should be
  /// created
  pipeline_->runOdometry(qdata, mdata_, graph_);
  /// \todo (yuchen) find a better place for this
  publishOdometry(qdata);
  pipeline_->visualizeOdometry(qdata, mdata_, graph_);

  LOG(DEBUG) << "Estimated transformation from live vertex ("
             << *(qdata->live_id)
             << ") to robot (i.e., T_r_m odometry): " << *qdata->T_r_m_odo;

  /// Update persistent localization (first frame won't have a valid live id)
  /// \todo only update this when odometry is successful
  /// \todo handle case where first frame is not a keyframe
  if (!first_frame_)
    updatePersistentLoc(*qdata->live_id, *qdata->T_r_m_odo, true);

  /// Publish odometry result on live robot localization
  if (publisher_)
    publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                             target_loc_);

  /// Update Odometry in localization chain
  /// \todo only when current_vertex_id is valid?
  chain_.updatePetioleToLeafTransform(*qdata->T_r_m_odo, true);

  /// Update the localization with respect to the privileged chain
  /// (target localization)
  /// \todo this is never true in branch mode
  updateTargetLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                  chain_.isLocalized());

  /// Check if we should create a new vertex
  const auto &keyframe_test_result = *(qdata->keyframe_test_result);
  if (keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    LOG(INFO) << "[Tactic] Creating a new keyframe!";

    /// Should finish odometry jobs before running localization
    pipeline_->waitForKeyframeJob();

    /// Add new vertex to the posegraph
    if (first_frame_)
      addDanglingVertex(*(qdata->stamp));
    else
      addConnectedVertex(*(qdata->stamp), *(qdata->T_r_m_odo));
    /// \todo qdata->live_id.fallback(current_vertex_id_);

    /// Call the pipeline to make and process the keyframe
    pipeline_->processKeyframe(qdata, mdata_, graph_, current_vertex_id_);

    /// (Temp) also compute odometry in world frame
    /// \todo change this estimate to use the localization chain
    if (!first_frame_)
      T_m_w_odo_.push_back(*qdata->T_r_m_odo * T_m_w_odo_.back());

    /// We try to branch off from existing experience, so the first frame
    /// connects to existing graph based on persistent localization
    if (first_frame_ && persistent_loc_.v.isSet()) {
      /// Should finish odometry jobs before running localization
      pipeline_->waitForKeyframeJob();

      qdata->live_id.fallback(current_vertex_id_);
      qdata->map_id.fallback(persistent_loc_.v);
      qdata->T_r_m_loc.fallback(persistent_loc_.T);

      LOG(INFO) << "Branching from existing experience: " << persistent_loc_.v
                << " --> " << current_vertex_id_;
      pipeline_->runLocalization(qdata, mdata_, graph_);

      LOG(DEBUG) << "Estimated transformation from trunk to robot (T_r_m "
                    "localization): "
                 << *qdata->T_r_m_loc;

      LOG(INFO) << "Adding new branch with offset: "
                << (*qdata->T_r_m_loc).inverse().vec().transpose();
      (void)graph_->addEdge(current_vertex_id_, persistent_loc_.v,
                            (*qdata->T_r_m_loc).inverse(), pose_graph::Spatial,
                            true);
    }

    /// must happen in key frame
    chain_.setPetiole(current_vertex_id_, first_frame_);
    chain_.updatePetioleToLeafTransform(EdgeTransform(true), true);
    /// Reset the map to robot transform and new vertex flag
    updatePersistentLoc(current_vertex_id_, EdgeTransform(true), true);
    /// Update odometry result on live robot localization
    if (publisher_)
      publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                               target_loc_);
  }

  // unset the first frame flag
  /// \todo what happens if the first frame is not a keyframe?
  first_frame_ = false;
}

void Tactic::follow(QueryCache::Ptr qdata) {
  /// \todo check that first_frame flag is false?

  /// \todo find a better default value for odometry
  qdata->T_r_m_odo.fallback(true);  // set covariance to zero

  /// Odometry to get relative pose estimate and whether a keyframe should be
  /// created
  pipeline_->runOdometry(qdata, mdata_, graph_);
  /// \todo (yuchen) find a better place for this
  publishOdometry(qdata);
  pipeline_->visualizeOdometry(qdata, mdata_, graph_);

  LOG(DEBUG) << "Estimated transformation from live vertex ("
             << *(qdata->live_id)
             << ") to robot (i.e., T_r_m odometry): " << *qdata->T_r_m_odo;

  /// Update Odometry in localization chain
  chain_.updatePetioleToLeafTransform(*qdata->T_r_m_odo, false);

  /// Update the localization with respect to the privileged chain
  /// (target localization)
  /// \todo this is never true in branch mode
  updatePersistentLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                      chain_.isLocalized());

  /// Publish odometry result on live robot localization
  if (publisher_)
    publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                             target_loc_);

  /// Check if we should create a new vertex
  const auto &keyframe_test_result = *(qdata->keyframe_test_result);
  if (keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    LOG(INFO) << "[Tactic] Creating a new keyframe!";

    /// Add new vertex to the posegraph
    if (first_frame_)
      /// \todo also localize against persistent localization
      addDanglingVertex(*(qdata->stamp));
    else
      addConnectedVertex(*(qdata->stamp), *(qdata->T_r_m_odo));

    /// Call the pipeline to make and process the keyframe
    pipeline_->processKeyframe(qdata, mdata_, graph_, current_vertex_id_);

    /// must happen in key frame
    chain_.setPetiole(current_vertex_id_, first_frame_);
    chain_.updatePetioleToLeafTransform(EdgeTransform(true), true);

    /// (Temp) also compute odometry in world frame
    if (!first_frame_) {
      T_m_w_odo_.push_back(chain_.T_start_leaf().inverse());
      T_m_w_loc_.push_back(chain_.T_start_trunk().inverse());
    }

    /// Should finish odometry jobs before running localization
    /// performance gains from when this is also running in a separate thread.
    pipeline_->waitForKeyframeJob();

    *qdata->live_id = current_vertex_id_;     /// \todo why is it here?
    *qdata->map_id = chain_.trunkVertexId();  /// \todo fallback invalid
    *qdata->loc_success = false;
    /// \todo create loc data
    if (!chain_.isLocalized()) {
      qdata->T_r_m_loc.fallback(
          Eigen::Matrix4d(Eigen::Matrix4d::Identity(4, 4)));
      const Eigen::Matrix<double, 6, 6> loc_cov = config_->default_loc_cov;
      qdata->T_r_m_loc->setCovariance(loc_cov);
    } else {
      qdata->T_r_m_loc.fallback(chain_.T_leaf_trunk());
    }

    // Run the localizer against the closest vertex
    pipeline_->runLocalization(qdata, mdata_, graph_);
    // publishLocalization(qdata);
    // pipeline_->visualizeLocalization(qdata, mdata_, graph_);

    LOG(DEBUG) << "Estimated transformation from localization vertex ("
               << *(qdata->map_id) << ") to live vertex (" << *(qdata->live_id)
               << ") (i.e., T_r_m localization): " << *qdata->T_r_m_loc;

    /// \todo (yuchen) add an edge no matter localization is successful or not
    const auto &T_r_m_loc = *(qdata->T_r_m_loc);

    // update the transform
    auto edge_id =
        EdgeId(current_vertex_id_, chain_.trunkVertexId(), pose_graph::Spatial);
    if (graph_->contains(edge_id)) {
      graph_->at(edge_id)->setTransform(T_r_m_loc.inverse());
    } else {
      graph_->addEdge(current_vertex_id_, chain_.trunkVertexId(),
                      T_r_m_loc.inverse(), pose_graph::Spatial, false);
    }

    // update the transform
    chain_.updateBranchToTwigTransform(T_r_m_loc, false);

    /// Move the localization chain forward upon successful localization
    chain_.convertPetioleTrunkToTwigBranch();

    /// Update the localization with respect to the privileged chain
    /// (target localization)
    updatePersistentLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                        chain_.isLocalized());
    /// Publish odometry result on live robot localization
    if (publisher_)
      publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                               target_loc_);
  }

  publishLocalization(qdata);
  pipeline_->visualizeLocalization(qdata, mdata_, graph_);

  // unset the first frame flag
  first_frame_ = false;
}

void Tactic::merge(QueryCache::Ptr qdata) {
  /// \todo check that first frame is set to false at this moment?

  /// \todo find a better default value for odometry
  qdata->T_r_m_odo.fallback(true);  // set covariance to zero

  /// Odometry to get relative pose estimate and whether a keyframe should be
  /// created
  pipeline_->runOdometry(qdata, mdata_, graph_);
  /// \todo (yuchen) find a better place for this
  publishOdometry(qdata);
  pipeline_->visualizeOdometry(qdata, mdata_, graph_);

  LOG(DEBUG) << "Estimated transformation from live vertex ("
             << *(qdata->live_id)
             << ") to robot (i.e., T_r_m odometry): " << *qdata->T_r_m_odo;

  /// Update persistent localization (first frame won't have a valid live id)
  /// \todo only update this when odometry is successful
  /// \todo handle case where first frame is not a keyframe
  if (!first_frame_)
    updatePersistentLoc(*qdata->live_id, *qdata->T_r_m_odo, true);

  /// Publish odometry result on live robot localization
  if (publisher_)
    publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                             target_loc_);

  /// Update Odometry in localization chain
  chain_.updatePetioleToLeafTransform(*qdata->T_r_m_odo, true);

  /// Update the localization with respect to the privileged chain
  /// (target localization)
  updateTargetLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                  chain_.isLocalized());

  /// \todo Old merge pipeline may increase keyframe creation frequency to
  /// localize more frequently (when not localized).

  /// Check if we should create a new vertex
  const auto &keyframe_test_result = *(qdata->keyframe_test_result);
  if (keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    LOG(INFO) << "[Tactic] Create a new keyframe and localize against the "
                 "merge path!";

    /// Add new vertex to the posegraph
    if (first_frame_)
      /// \todo also localize against persistent localization
      addDanglingVertex(*(qdata->stamp));
    else
      addConnectedVertex(*(qdata->stamp), *(qdata->T_r_m_odo));

    /// Call the pipeline to make and process the keyframe
    pipeline_->processKeyframe(qdata, mdata_, graph_, current_vertex_id_);

    /// must happen in key frame
    chain_.setPetiole(current_vertex_id_, first_frame_);
    chain_.updatePetioleToLeafTransform(EdgeTransform(true), true);
    /// Reset the map to robot transform and new vertex flag
    updatePersistentLoc(current_vertex_id_, EdgeTransform(true), true);
    /// Update odometry result on live robot localization
    if (publisher_)
      publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                               target_loc_);

    /// Should finish odometry jobs before running localization
    /// performance gains from when this is also running in a separate thread.
    pipeline_->waitForKeyframeJob();

    /// Start localization
    *qdata->live_id = current_vertex_id_;
    *qdata->map_id = chain_.trunkVertexId();
    *qdata->loc_success = false;
    /// \todo the following localization logic needs change, there is also a
    /// flag here to decide how we generate the prior for localization
    if (!chain_.isLocalized()) {
      qdata->T_r_m_loc.fallback(
          Eigen::Matrix4d(Eigen::Matrix4d::Identity(4, 4)));
      const Eigen::Matrix<double, 6, 6> loc_cov = config_->default_loc_cov;
      qdata->T_r_m_loc->setCovariance(loc_cov);
    } else {
      qdata->T_r_m_loc.fallback(chain_.T_leaf_trunk());
    }

    // Run the localizer against the closest vertex
    pipeline_->runLocalization(qdata, mdata_, graph_);
    // publishLocalization(qdata);
    // pipeline_->visualizeLocalization(qdata, mdata_, graph_);

    if (*qdata->loc_success) {
      LOG(DEBUG) << "Estimated transformation from localization vertex ("
                 << *(qdata->map_id) << ") to live vertex ("
                 << *(qdata->live_id)
                 << ") (i.e., T_r_m localization): " << *qdata->T_r_m_loc;

      /// \todo (yuchen) add an edge no matter localization is successful or not
      const auto &T_r_m_loc = *(qdata->T_r_m_loc);

      // update the transform
      chain_.updateBranchToTwigTransform(T_r_m_loc, false);

      /// Move the localization chain forward upon successful localization
      chain_.convertPetioleTrunkToTwigBranch();

      /// Update the localization with respect to the privileged chain
      /// (target localization)
      updateTargetLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                      chain_.isLocalized());

      /// Increment localization success
      target_loc_.successes++;
    } else {
      LOG(INFO)
          << "Failed to estimate transformation from localization vertex ("
          << *(qdata->map_id) << ") to live vertex (" << *(qdata->live_id)
          << ")";

      /// Decrement localization success
      target_loc_.successes--;
    }

    /// \todo need some logic here to decide whether or not to move to the next
    /// vertex to localize against
  } else {
    LOG(INFO) << "[Tactic] Localize against the merge path without creating a "
                 "new keyframe!";

    /// \todo remove this once chain is used in odometry (should be the same)
    chain_.setPetiole(current_vertex_id_, first_frame_);

    /// Setup localization cache
    *qdata->live_id = current_vertex_id_;
    *qdata->map_id = chain_.trunkVertexId();
    *qdata->loc_success = false;
    /// \todo the following localization logic needs change, there is also a
    /// flag here to decide how we generate the prior for localization
    if (!chain_.isLocalized()) {
      qdata->T_r_m_loc.fallback(
          Eigen::Matrix4d(Eigen::Matrix4d::Identity(4, 4)));
      const Eigen::Matrix<double, 6, 6> loc_cov = config_->default_loc_cov;
      qdata->T_r_m_loc->setCovariance(loc_cov);
    } else {
      qdata->T_r_m_loc.fallback(chain_.T_leaf_trunk());
    }

    // Run the localizer against the closest vertex
    pipeline_->runLocalization(qdata, mdata_, graph_);
    // publishLocalization(qdata);
    // pipeline_->visualizeLocalization(qdata, mdata_, graph_);

    if (*qdata->loc_success) {
      LOG(DEBUG) << "Estimated transformation from localization vertex ("
                 << *(qdata->map_id) << ") to live vertex ("
                 << *(qdata->live_id)
                 << ") (i.e., T_r_m localization): " << *qdata->T_r_m_loc;

      /// \todo (yuchen) add an edge no matter localization is successful or not
      const auto T_r_m_loc =
          (*qdata->T_r_m_odo).inverse() * (*qdata->T_r_m_loc);

      // update the transform
      chain_.updateBranchToTwigTransform(T_r_m_loc, false);

      /// Move the localization chain forward upon successful localization
      chain_.convertPetioleTrunkToTwigBranch();

      /// Update the localization with respect to the privileged chain
      /// (target localization)
      updateTargetLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                      chain_.isLocalized());

      /// Increment localization success
      target_loc_.successes++;
    } else {
      LOG(INFO)
          << "Failed to estimate transformation from localization vertex ("
          << *(qdata->map_id) << ") to live vertex (" << *(qdata->live_id)
          << ")";

      /// Decrement localization success
      target_loc_.successes--;
    }
  }

  /// \todo need some logic here to decide whether or not to move to the next
  /// vertex to localize against
  LOG(INFO) << "Localizing againt vertex: " << target_loc_.v
            << ", number of successes is " << target_loc_.successes;
  if (target_loc_.successes < -5) {
    LOG(INFO) << "Cannot localize against this vertex, move to the next one.";
    auto trunk_seq =
        (chain_.trunkSequenceId() + 3) % uint32_t(chain_.sequence().size() - 1);
    chain_.resetTrunk(trunk_seq);
  }

  publishLocalization(qdata);
  pipeline_->visualizeLocalization(qdata, mdata_, graph_);

  // unset the first frame flag
  first_frame_ = false;
}

void Tactic::search(QueryCache::Ptr qdata) {
  /// \todo find a better default value for odometry
  qdata->T_r_m_odo.fallback(true);  // set covariance to zero

  /// Odometry to get relative pose estimate and whether a keyframe should be
  /// created
  pipeline_->runOdometry(qdata, mdata_, graph_);
  /// \todo (yuchen) find a better place for this
  publishOdometry(qdata);
  pipeline_->visualizeOdometry(qdata, mdata_, graph_);

  LOG(DEBUG) << "Estimated transformation from live vertex ("
             << *(qdata->live_id)
             << ") to robot (i.e., T_r_m odometry): " << *qdata->T_r_m_odo;

  // /// Update persistent localization (first frame won't have a valid live id)
  // /// \todo only update this when odometry is successful
  // /// \todo handle case where first frame is not a keyframe
  // if (!first_frame_)
  //   updatePersistentLoc(*qdata->live_id, *qdata->T_r_m_odo, true);

  // /// Publish odometry result on live robot localization
  // if (publisher_)
  //   publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
  //                            target_loc_);

  /// Update Odometry in localization chain
  chain_.updatePetioleToLeafTransform(*qdata->T_r_m_odo, true);

  // /// Update the localization with respect to the privileged chain
  // /// (target localization)
  // updateTargetLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
  //                 chain_.isLocalized());

  /// \todo Old merge pipeline may increase keyframe creation frequency to
  /// localize more frequently (when not localized).

  /// Check if we should create a new vertex
  const auto &keyframe_test_result = *(qdata->keyframe_test_result);
  if (keyframe_test_result == KeyframeTestResult::CREATE_VERTEX) {
    LOG(INFO) << "[Tactic] Create a new keyframe and localize against the "
                 "merge path!";

    /// Add new vertex to the posegraph
    if (first_frame_)
      /// \todo also localize against persistent localization
      addDanglingVertex(*(qdata->stamp));
    else
      addConnectedVertex(*(qdata->stamp), *(qdata->T_r_m_odo));

    /// Call the pipeline to make and process the keyframe
    pipeline_->processKeyframe(qdata, mdata_, graph_, current_vertex_id_);

    /// must happen in key frame
    chain_.setPetiole(current_vertex_id_, first_frame_);
    chain_.updatePetioleToLeafTransform(EdgeTransform(true), true);
    // /// Reset the map to robot transform and new vertex flag
    // updatePersistentLoc(current_vertex_id_, EdgeTransform(true), true);
    // /// Update odometry result on live robot localization
    // if (publisher_)
    //   publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
    //                            target_loc_);

    /// Should finish odometry jobs before running localization
    /// performance gains from when this is also running in a separate thread.
    pipeline_->waitForKeyframeJob();

    /// Start localization
    *qdata->live_id = current_vertex_id_;
    *qdata->map_id = chain_.trunkVertexId();
    *qdata->loc_success = false;
    /// \todo the following localization logic needs change, there is also a
    /// flag here to decide how we generate the prior for localization
    if (!chain_.isLocalized()) {
      qdata->T_r_m_loc.fallback(
          Eigen::Matrix4d(Eigen::Matrix4d::Identity(4, 4)));
      const Eigen::Matrix<double, 6, 6> loc_cov = config_->default_loc_cov;
      qdata->T_r_m_loc->setCovariance(loc_cov);
    } else {
      qdata->T_r_m_loc.fallback(chain_.T_leaf_trunk());
    }

    // Run the localizer against the closest vertex
    pipeline_->runLocalization(qdata, mdata_, graph_);
    // publishLocalization(qdata);
    // pipeline_->visualizeLocalization(qdata, mdata_, graph_);

    if (*qdata->loc_success) {
      LOG(DEBUG) << "Estimated transformation from localization vertex ("
                 << *(qdata->map_id) << ") to live vertex ("
                 << *(qdata->live_id)
                 << ") (i.e., T_r_m localization): " << *qdata->T_r_m_loc;

      /// \todo (yuchen) add an edge no matter localization is successful or not
      const auto &T_r_m_loc = *(qdata->T_r_m_loc);

      // update the transform
      chain_.updateBranchToTwigTransform(T_r_m_loc, false);

      /// Move the localization chain forward upon successful localization
      chain_.convertPetioleTrunkToTwigBranch();

      // /// Update the localization with respect to the privileged chain
      // /// (target localization)
      // updateTargetLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
      //                 chain_.isLocalized());
      /// Update the localization with respect to the privileged chain
      /// (target localization)
      updatePersistentLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                          chain_.isLocalized());
      /// Publish odometry result on live robot localization
      if (publisher_)
        publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                                 target_loc_);

      /// Increment localization success
      persistent_loc_.successes++;
    } else {
      LOG(INFO)
          << "Failed to estimate transformation from localization vertex ("
          << *(qdata->map_id) << ") to live vertex (" << *(qdata->live_id)
          << ")";

      /// Decrement localization success
      persistent_loc_.successes--;
    }

    /// \todo need some logic here to decide whether or not to move to the next
    /// vertex to localize against
  } else {
    LOG(INFO) << "[Tactic] Localize against the merge path without creating a "
                 "new keyframe!";

    /// \todo remove this once chain is used in odometry (should be the same)
    chain_.setPetiole(current_vertex_id_, first_frame_);

    /// Setup localization cache
    *qdata->live_id = current_vertex_id_;
    *qdata->map_id = chain_.trunkVertexId();
    *qdata->loc_success = false;
    /// \todo the following localization logic needs change, there is also a
    /// flag here to decide how we generate the prior for localization
    if (!chain_.isLocalized()) {
      qdata->T_r_m_loc.fallback(
          Eigen::Matrix4d(Eigen::Matrix4d::Identity(4, 4)));
      const Eigen::Matrix<double, 6, 6> loc_cov = config_->default_loc_cov;
      qdata->T_r_m_loc->setCovariance(loc_cov);
    } else {
      qdata->T_r_m_loc.fallback(chain_.T_leaf_trunk());
    }

    // Run the localizer against the closest vertex
    pipeline_->runLocalization(qdata, mdata_, graph_);
    // publishLocalization(qdata);
    // pipeline_->visualizeLocalization(qdata, mdata_, graph_);

    if (*qdata->loc_success) {
      LOG(DEBUG) << "Estimated transformation from localization vertex ("
                 << *(qdata->map_id) << ") to live vertex ("
                 << *(qdata->live_id)
                 << ") (i.e., T_r_m localization): " << *qdata->T_r_m_loc;

      /// \todo (yuchen) add an edge no matter localization is successful or not
      const auto T_r_m_loc =
          (*qdata->T_r_m_odo).inverse() * (*qdata->T_r_m_loc);

      // update the transform
      chain_.updateBranchToTwigTransform(T_r_m_loc, false);

      /// Move the localization chain forward upon successful localization
      chain_.convertPetioleTrunkToTwigBranch();

      // /// Update the localization with respect to the privileged chain
      // /// (target localization)
      // updateTargetLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
      //                 chain_.isLocalized());
      /// Update the localization with respect to the privileged chain
      /// (target localization)
      updatePersistentLoc(chain_.trunkVertexId(), chain_.T_leaf_trunk(),
                          chain_.isLocalized());
      /// Publish odometry result on live robot localization
      if (publisher_)
        publisher_->publishRobot(persistent_loc_, chain_.trunkSequenceId(),
                                 target_loc_);

      /// Increment localization success
      persistent_loc_.successes++;
    } else {
      LOG(INFO)
          << "Failed to estimate transformation from localization vertex ("
          << *(qdata->map_id) << ") to live vertex (" << *(qdata->live_id)
          << ")";

      /// Decrement localization success
      persistent_loc_.successes--;
    }
  }

  /// \todo need some logic here to decide whether or not to move to the next
  /// vertex to localize against
  LOG(INFO) << "Localizing againt vertex: " << persistent_loc_.v
            << ", number of successes is " << persistent_loc_.successes;
  if (persistent_loc_.successes < -5) {
    LOG(INFO) << "Cannot localize against this vertex, move to the next one.";
    /// \todo for now, only search from the first 10 vertices
    auto trunk_seq = (chain_.trunkSequenceId() + 3) % uint32_t(30);
    chain_.resetTrunk(trunk_seq);
  }

  publishLocalization(qdata);
  pipeline_->visualizeLocalization(qdata, mdata_, graph_);

  // unset the first frame flag
  first_frame_ = false;
}

void Tactic::publishOdometry(QueryCache::Ptr qdata) {
  /// Publish the latest keyframe estimate in world frame
  Eigen::Affine3d T(T_m_w_odo_.back().inverse().matrix());
  auto msg = tf2::eigenToTransform(T);
  msg.header.frame_id = "world";
  msg.header.stamp = *(qdata->rcl_stamp);
  msg.child_frame_id = "odometry keyframe";
  tf_broadcaster_->sendTransform(msg);

  /// Publish the teach path
  ROSPathMsg path;
  path.header.frame_id = "world";
  path.header.stamp = *(qdata->rcl_stamp);
  auto &poses = path.poses;
  for (const auto &pose : T_m_w_odo_) {
    PoseStampedMsg ps;
    ps.pose = tf2::toMsg(Eigen::Affine3d(pose.inverse().matrix()));
    poses.push_back(ps);
  }
  odo_path_pub_->publish(path);

  /// Publish the current frame
  Eigen::Affine3d T2(qdata->T_r_m_odo->inverse().matrix());
  auto msg2 = tf2::eigenToTransform(T2);
  msg2.header.frame_id = "odometry keyframe";
  msg2.header.stamp = *(qdata->rcl_stamp);
  msg2.child_frame_id = "robot";
  tf_broadcaster_->sendTransform(msg2);
}

void Tactic::publishLocalization(QueryCache::Ptr qdata) {
  /// Publish the current frame localized against in world frame
  Eigen::Affine3d T(T_m_w_loc_.back().inverse().matrix());
  auto msg = tf2::eigenToTransform(T);
  msg.header.frame_id = "world";
  msg.header.stamp = *(qdata->rcl_stamp);
  // apply an offset to y axis
  msg.transform.translation.z -= 0.2;
  // msg.transform.translation.z -= 10;  // for lidar
  msg.child_frame_id = "localization keyframe";
  tf_broadcaster_->sendTransform(msg);
}

}  // namespace tactic
}  // namespace vtr