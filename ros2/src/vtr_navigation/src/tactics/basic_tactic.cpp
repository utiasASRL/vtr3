#if false
#include <vtr_navigation/factories/pipeline_factory.h>
#include <vtr_navigation/memory/live_memory_manager.h>
#include <vtr_navigation/memory/map_memory_manager.h>
#include <vtr_navigation/pipelines/base_pipeline.h>
#endif
#include <vtr_navigation/tactics/basic_tactic.hpp>
#if false
#include <vtr_navigation/types.h>
#include <vtr_planning/state_machine_interface.h>

#include <asrl/messages/VOStatus.pb.h>
// #include <asrl/common/emotions.hpp>
#endif
namespace vtr {
namespace navigation {
#if false
BasicTactic::BasicTactic(
    TacticConfig& config, const std::shared_ptr<ConverterAssembly>& converter,
    const std::shared_ptr<QuickVoAssembly>& quick_vo,
    const std::shared_ptr<RefinedVoAssembly>& refined_vo,
    const std::shared_ptr<LocalizerAssembly>& localizer,
    // const std::shared_ptr<TerrainAssessmentAssembly>& terrain_assessment,
    std::shared_ptr<Graph> graph)
    : first_frame_(true),
      map_status_(MAP_NEW),
      converter_(converter),
      quick_vo_(quick_vo),
      refined_vo_(refined_vo),
      chain_(config.locchain_config, graph),
      localizer_(localizer) {
  steam_mutex_ptr_.reset(new std::mutex());

  if (graph == nullptr) {
    // TODO: Load from file, set up directory to save.
    pose_graph_.reset(
        new Graph(robochunk::util::split_directory(config.data_directory) +
                      "/graph_index",
                  0));
  } else {
    pose_graph_ = graph;
  }
  // add an initial run (for UAV state machine, as runs are not created by the
  // states)
  if (config.insert_initial_run) {
    this->addRun();
  }

  config_ = config;
  // chain_ = {config.locchain_config, pose_graph_}; // \todo initialized before
  publisher_ = nullptr;

  if (config.map_memory_config.enable) {
    LOG(INFO) << "Starting map memory manager";
    map_memory_manager_ = std::make_shared<MapMemoryManager>(
        pose_graph_, chain_, config.map_memory_config);
    map_memory_manager_->start();
  }

  if (config.live_memory_config.enable) {
    LOG(INFO) << "Starting live memory manager";
    live_memory_manager_ = std::make_shared<LiveMemoryManager>(
        pose_graph_, this, config.live_memory_config);
    live_memory_manager_->start();
  }

#if 0
  if (ta_parallelization_) {
    LOG(INFO) << "Terrain assessment in parallel mode";
  } else {
    LOG(INFO) << "Terrain assessment in series mode";
  }
#endif
}

BasicTactic::~BasicTactic() { this->halt(); }

void BasicTactic::halt() {
  // wait for the pipeline to clear
  auto lck = lockPipeline();

#if 0
  // stop the path tracker
  if (path_tracker_) path_tracker_->stopAndJoin();
#endif

  // stop the memory managers
  map_memory_manager_.reset();
  live_memory_manager_.reset();

  return;
}

#if 0
void BasicTactic::setPathTracker(
    std::shared_ptr<asrl::path_tracker::Base> path_tracker) {
  path_tracker_ = path_tracker;
}

void BasicTactic::setHoverController(
    std::shared_ptr<asrl::path_tracker::Base> hover_controller) {
  hover_controller_ = hover_controller;
}

void BasicTactic::setGimbalController(
    std::shared_ptr<asrl::path_tracker::Base> gimbal_controller) {
  gimbal_controller_ = gimbal_controller;
}
#endif

void BasicTactic::setupCaches(QueryCachePtr query_data, MapCachePtr map_data) {
  // update the query cache with the necessary tactic data
  query_data->T_sensor_vehicle.fallback(T_sensor_vehicle_);
  query_data->steam_mutex = steam_mutex_ptr_;
  // default to success
  *map_data->success = true;
}

#if 0
void BasicTactic::startControlLoop(asrl::pose_graph::LocalizationChain& chain) {
  if (!path_tracker_) {
    LOG(WARNING) << "Path tracker not set! Cannot start control loop.";
    return;
  }

  LOG(INFO) << "Starting path tracker.";
  path_tracker_->followPathAsync(path_tracker::State::PAUSE, chain);
  return;
}

void BasicTactic::stopPathTracker(void) {
  if (!path_tracker_) {
    LOG(WARNING) << "Path tracker not set! Cannot stop control loop.";
    return;
  }

  LOG(INFO) << "Stopping path tracker.";
  path_tracker_->stopAndJoin();
  return;
}
#endif

void BasicTactic::setPath(const vtr::planning::PathType& path, bool follow) {
  LOG(DEBUG) << "[Lock Requested] setPath";
  auto lck = lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] setPath";

  chain_.setSequence(path);
  targetLocalization_ = Localization();

#if 0
  if (publisher_ != nullptr) {
    publisher_->clearPath();
  }
#endif

  if (path.size() > 0) {
    chain_.expand();
#if 0
    if (publisher_ != nullptr) {
      if (follow) {
        publisher_->publishPath(chain_);

        // Start the new path tracker
        startControlLoop(chain_);
      }
    }
#endif
  } else {
#if 0
    // make sure path tracker is stopped
    stopPathTracker();
#endif
  }
#if 0
  if (publisher_) publisher_->publishRobot(persistentLocalization_);
#endif
  return;
}

#if 0
bool BasicTactic::startHover(const asrl::planning::PathType& path) {
  // check that we have a path
  if (path.empty()) {
    LOG(ERROR) << "path is empty!";
    return false;
  }

  // check that the hover controller exists
  if (!hover_controller_) {
    LOG(ERROR)
        << "Could not start hover control. Hover controller not initialized.";
    return false;
  }

  // lock the pipeline
  LOG(DEBUG) << "[Lock Requested] startHover";
  auto lck = lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] startHover";

  // update the localisation chain
  chain_.setSequence(path);
  targetLocalization_ = Localization();

  // compound transforms to get poses in a global frame
  chain_.expand();

  LOG(INFO) << "Starting hover controller.";
  hover_controller_->followPathAsync(path_tracker::State::PAUSE, chain_);

  //  stop any running controllers
  if (path_tracker_ && path_tracker_->isRunning()) {
    LOG(INFO) << "Commanding path tracker to stop";
    path_tracker_->setState(path_tracker::State::STOP);
    path_tracker_->finishControlLoop();  // added finishControlLoop - path
                                         // tracker should exit thread
  }

  // set the hover controller to run
  hover_controller_->setState(path_tracker::State::RUN);
  LOG(INFO) << "Hover controller started successfully.";

  LOG(DEBUG) << "[Lock Released] startHover";
  return true;
}

void BasicTactic::stopAllControl() {
  // check that the hover controller exists
  if (!hover_controller_) {
    LOG(ERROR)
        << "Could not stop hover control. Hover controller not initialized.";
  }

  if (!path_tracker_) {
    LOG(ERROR) << "Could not stop path tracker control. Path tracking "
                  "controller not initialized.";
  }

  //  stop hover controller
  if (hover_controller_ && hover_controller_->isRunning()) {
    LOG(INFO) << "Setting hover controller to stop";
    hover_controller_->setState(path_tracker::State::STOP);
    hover_controller_->finishControlLoop();  // added finishControlLoop - path
                                             // tracker should exit thread
  }

  //  stop pt controller
  if (path_tracker_ && path_tracker_->isRunning()) {
    LOG(INFO) << "Setting path tracker to stop";
    path_tracker_->setState(path_tracker::State::STOP);
    path_tracker_->finishControlLoop();  // added finishControlLoop - path
                                         // tracker should exit thread
  }
  return;
}

bool BasicTactic::startFollow(const asrl::planning::PathType& path) {
  // check that we have a path
  if (path.empty()) {
    LOG(ERROR) << "path is empty!";
    return false;
  }

  // check that the path tracker exists
  if (!path_tracker_) {
    LOG(ERROR) << "Could not start path_tracker. Path tracker not initialized.";
    return false;
  }

  // lock the pipeline
  LOG(DEBUG) << "[Lock Requested] startFollow";
  auto lck = lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] startFollow";

  // update the localisation chain
  chain_.setSequence(path);
  targetLocalization_ = Localization();

  // compund transforms to get poses in a global frame
  chain_.expand();

  // Start the new path tracker
  LOG(INFO) << "Starting path tracker.";
  path_tracker_->followPathAsync(path_tracker::State::PAUSE, chain_);

  //  stop any running controllers
  if (hover_controller_ && hover_controller_->isRunning()) {
    hover_controller_->setState(path_tracker::State::STOP);
    hover_controller_->finishControlLoop();
  }

  // set the path tracker to run
  path_tracker_->setState(path_tracker::State::RUN);

  LOG(DEBUG) << "[Lock Released] startFollow";
  return true;
}
#endif

void BasicTactic::setTrunk(const VertexId& v) {
  // We cannot change the trunk externally while a frame is in the pipeline
  LOG(DEBUG) << "[Lock Requested] setTrunk";
  auto lck = lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] setTrunk";

  persistentLocalization_ = Localization(v);
  targetLocalization_ = Localization();
#if 0
  if (publisher_ != nullptr) {
    publisher_->publishRobot(persistentLocalization_);
  }
#endif
  LOG(DEBUG) << "[Lock Released] setTrunk";
}

void BasicTactic::runPipeline(QueryCachePtr query_data) {
  // Lock to make sure the pipeline isn't changed during processing
  LockType lck(pipeline_mutex_, std::defer_lock_t());

  // If we cannot lock in 30ms, give up and hope that the next frame will work
  if (!lck.try_lock_for(std::chrono::milliseconds(30))) {
    LOG(WARNING)
        << "[BasicTactic] Dropping frame due to unavailable pipeline mutex";
    return;
  }

  // make a new map cache
  MapCachePtr map_data(new MapCache);

  // add initial data to the cache
  setupCaches(query_data, map_data);

  // run the image converter
  pipeline_->convertData(query_data, map_data);

  // setup and run the pipeline processData();
  processData(query_data, map_data);
}

void BasicTactic::processData(QueryCachePtr query_data, MapCachePtr map_data) {
  // if we have a stereo or greater rig, the map is automatically initialized
  if (query_data->rig_calibrations.is_valid() &&
      query_data->rig_calibrations->begin()->intrinsics.size() > 1) {
    map_data->map_status = MAP_INITIALIZED;
  } else {
    // only in mono, if the map is not yet initialised, should we set it here
    map_data->map_status = map_status_;
  }

  // now run the rest of the pipeline
  using IsKf = BasePipeline::KeyframeRequest;
  IsKf keyframe_request =
      pipeline_->processData(query_data, map_data, first_frame_);

  // When it's the first frame, we want to drop a keyframe
  // if (first_frame_ && create_keyframe) create_keyframe =
  // std::max(create_keyframe, IsKf::YES);

  // Check to see if our keyframe thread is available
  bool thread_available =
      keyframe_thread_future_.valid() == false ||
      keyframe_thread_future_.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready;

  // If we don't parallelize or don't skip, then we process every keyframe
  bool process_every_keyframe =
      !config_.keyframe_parallelization || !config_.keyframe_skippable;

  // Check the different keyframe requests and their requirements
  bool really_create_keyframe = keyframe_request == IsKf::YES ||
                                (keyframe_request == IsKf::IF_AVAILABLE &&
                                 (process_every_keyframe || thread_available));

  // Create a keyframe if we're sure!
  if (really_create_keyframe) {
    QueryCachePtr kf_query_cache = pipeline_->candidateQueryCache();
    MapCachePtr kf_map_cache = pipeline_->candidateMapCache();
    if (kf_query_cache == nullptr || kf_map_cache == nullptr) {
      LOG_IF(!first_frame_, INFO) << "candidate is null, reverting back!!\n";
      kf_query_cache = query_data;
      kf_map_cache = map_data;
    }
    // make a keyframe regardless of processing.
    pipeline_->makeKeyFrame(kf_query_cache, kf_map_cache, first_frame_);
    // If there is a thread avaliable, then make a new keyframe job and update
    // the chain.
    if (thread_available || !config_.keyframe_skippable) {
      if (keyframe_thread_future_.valid())
        keyframe_thread_future_.wait();  // in case refined vo is still going
      pipeline_->wait();  // in case there is still a localization job going on
      chain_.resetTwigAndBranch();
      if (config_.keyframe_parallelization == true) {
        keyframe_thread_future_ = std::async(
            std::launch::async, &BasePipeline::processKeyFrame, pipeline_.get(),
            kf_query_cache, kf_map_cache, first_frame_);
      } else {
        pipeline_->processKeyFrame(kf_query_cache, kf_map_cache, first_frame_);
      }
    } else {
      pipeline_->processPetiole(query_data, map_data, first_frame_);
      LOG(WARNING) << "Skipping keyframe job.";
    }
  }

  if (query_data->live_id.is_valid() && query_data->rig_images.is_valid()) {
    // update the vertex with the VO status
    asrl::status_msgs::VOStatus status;
    status.set_leaf_image_stamp((*query_data->stamp).nanoseconds_since_epoch());
    // Compute the current time in seconds.
    auto time_now = std::chrono::system_clock::now().time_since_epoch();
    double time_now_secs =
        static_cast<double>(time_now.count() *
                            std::chrono::system_clock::period::num) /
        std::chrono::system_clock::period::den;

    status.set_leaf_processed_stamp(static_cast<int64_t>(time_now_secs * 1e9));
    status.set_branch_id(chain_.branchVertexId());
    status.set_trunk_id(chain_.trunkVertexId());

    // set the transforms.
    *status.mutable_t_leaf_trunk() << chain_.T_leaf_trunk();
    *status.mutable_t_branch_trunk() << chain_.T_branch_trunk();

    status.set_success(*map_data->success);
    status.set_keyframe_flag(query_data->new_vertex_flag.is_valid() &&
                             *(query_data->new_vertex_flag) !=
                                 CREATE_CANDIDATE);
    auto vertex = pose_graph_->at(*query_data->live_id);

    // fill in the status
    auto run = pose_graph_->run((*query_data->live_id).majorId());
    std::string vo_status_str("/results/VO");
    if (!run->hasVertexStream(vo_status_str)) {
      run->registerVertexStream(vo_status_str, true);
    }

    vertex->insert<asrl::status_msgs::VOStatus>(vo_status_str, status,
                                                *query_data->stamp);
  }

  // we now must have processed the first frame (if there was image data)
  if (first_frame_ && really_create_keyframe) first_frame_ = false;

  // if the map has been initialized, keep a record
  map_status_ = *map_data->map_status;

#if 0
  // Do terrain assessment.
  pipeline_->assessTerrain(query_data, map_data, ta_parallelization_,
                           ta_thread_future_);

  // Compute T_0_q
  // pipeline_->computeT_0_q(query_data, map_data);
#endif
}

void BasicTactic::setPipeline(const vtr::planning::PipelineType& pipeline) {
  // Lock to make sure all frames clear the pipeline
  LOG(DEBUG) << "[Lock Requested] setPipeline";
  auto lck = lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] setPipeline";

  // Change the pipeline after we're sure everything is clear
  pipeline_ = PipelineFactory::make(pipeline, this);

  LOG(DEBUG) << "[Lock Released] setPipeline";
}

#if 0
bool BasicTactic::needNewVertex(const QueryCache& query_cache,
                                const MapCache&) const {
  return *query_cache.new_vertex_flag;
}
#endif

VertexId BasicTactic::addDanglingVertex(
    const robochunk::std_msgs::TimeStamp& stamp) {
  // Add the new vertex
  auto vertex = pose_graph_->addVertex(stamp);
  current_vertex_id_ = vertex->id();
  // We've now fulfilled any new frame requirements
  return current_vertex_id_;
}

/** \brief Clears the pipeline and stops callbacks
 */
auto BasicTactic::lockPipeline() -> LockType {
  // Lock to make sure all frames clear the pipeline
  LockType lck(pipeline_mutex_);

#if 0
  // Pause the trackers/controllers but keep the current goal
  asrl::path_tracker::State pt_state;

  if (path_tracker_ && path_tracker_->isRunning()) {
    pt_state = path_tracker_->getState();
    if (pt_state != path_tracker::State::STOP) {
      LOG(INFO) << "pausing path tracker thread";
    }
    path_tracker_->pause();
  }

  asrl::path_tracker::State hc_state;
  if (hover_controller_ && hover_controller_->isRunning()) {
    hc_state = hover_controller_->getState();
    if (hc_state != path_tracker::State::STOP) {
      LOG(INFO) << "pausing hover control thread";
      hover_controller_->pause();
    }
  }

  if (gimbal_controller_ && gimbal_controller_->isRunning()) {
    gimbal_controller_->pause();
  }
#endif

  // Join the keyframe thread to make sure that all optimization is done
  if (keyframe_thread_future_.valid()) keyframe_thread_future_.wait();

#if 0
  // Join the terrain assessment thread to make sure that it's done
  if (ta_thread_future_.valid()) {
    ta_thread_future_.wait();
  }
#endif

  // Let the pipeline wait for any threads it owns
  if (pipeline_) pipeline_->wait();

#if 0
  // resume the trackers/controllers to their original state
  if (path_tracker_ && path_tracker_->isRunning()) {
    path_tracker_->setState(pt_state);
    LOG(INFO) << "resuming path tracker thread to state " << int(pt_state);
  }

  if (hover_controller_ && hover_controller_->isRunning()) {
    hover_controller_->setState(hc_state);
    LOG(INFO) << "resuming hover control thread to state " << int(hc_state);
  }

  if (gimbal_controller_) {
    gimbal_controller_->resume();
  }
#endif

  // wait on any threads this tactic holds
  this->wait();

  return lck;
}

VertexId BasicTactic::addConnectedVertex(
    const robochunk::std_msgs::TimeStamp& stamp,
    const lgmath::se3::TransformationWithCovariance& T_q_m) {
  // Add the new vertex
  auto previous_vertex_id = current_vertex_id_;
  addDanglingVertex(stamp);

  // Add connection
  // \todo (old) make a virtual pipeline function: bool pipeline_.isManual();
  bool is_manual = !dynamic_cast<MetricLocalizationPipeline*>(pipeline_.get())
// \todo disable this once added merge pipeline.
#if 0
      && !dynamic_cast<LocalizationSearchPipeline*>(pipeline_.get())
#endif
      ;
  auto edge =
      pose_graph_->addEdge(previous_vertex_id, current_vertex_id_, T_q_m,
                           asrl::pose_graph::Temporal, is_manual);

  return current_vertex_id_;
}

double BasicTactic::distanceToSeqId(const uint64_t& seq_id) {
  // Lock to make sure the path isn't changed out from under us
  std::lock_guard<std::recursive_timed_mutex> lck(pipeline_mutex_);

  // Clip the sequence ID to the max/min for the chain
  auto clip_seq = unsigned(std::min(seq_id, chain_.sequence().size() - 1));
  clip_seq = std::max(clip_seq, 0u);
  auto trunk_seq = unsigned(chain_.trunkSequenceId());

  if (clip_seq == trunk_seq) {
    return 0.;
  }

  unsigned start_seq = std::min(clip_seq, trunk_seq);
  unsigned end_seq = std::max(clip_seq, trunk_seq);

  // Compound raw distance along the path
  double dist = 0.;
  for (unsigned idx = start_seq; idx < end_seq; ++idx) {
    dist +=
        (chain_.pose(idx) * chain_.pose(idx + 1).inverse()).r_ab_inb().norm();
  }

  // Returns a negative value if we have passed that sequence already
  return (clip_seq < chain_.trunkSequenceId()) ? -dist : dist;
}

vtr::planning::LocalizationStatus BasicTactic::tfStatus(
    const EdgeTransform& tf) const {
  if (!tf.covarianceSet()) return vtr::planning::LocalizationStatus::LOST;
  double ex = std::sqrt(persistentLocalization_.T.cov()(0, 0)),
         ey = std::sqrt(persistentLocalization_.T.cov()(1, 1)),
         et = std::sqrt(persistentLocalization_.T.cov()(5, 5));
  // Check if we're so uncertain that we're lost
  if (ex > config_.loc_lost_thresh(0) || ey > config_.loc_lost_thresh(1) ||
      et > config_.loc_lost_thresh(2))
    return vtr::planning::LocalizationStatus::LOST;
  // If we're not lost, check if we're dead reckoning
  else if (ex > config_.loc_deadreckoning_thresh(0) ||
           ey > config_.loc_deadreckoning_thresh(1) ||
           et > config_.loc_deadreckoning_thresh(2))
    return vtr::planning::LocalizationStatus::DeadReckoning;
  // If we got this far, this is a confident transform
  return vtr::planning::LocalizationStatus::Confident;
}

vtr::planning::TacticStatus BasicTactic::status() const {
  // TODO: Return actual status.
  auto rval = vtr::planning::TacticStatus();

  rval.localization_ = persistentLocalization_.localized
                           ? tfStatus(persistentLocalization_.T)
                           : vtr::planning::LocalizationStatus::Forced;

  rval.targetLocalization_ = chain_.isLocalized()
                                 ? tfStatus(chain_.T_leaf_trunk())
                                 : vtr::planning::LocalizationStatus::Forced;

  return rval;
}

const Localization& BasicTactic::persistentLoc() const {
  return persistentLocalization_;
}

const Localization& BasicTactic::targetLoc() const {
  return targetLocalization_;
}

const VertexId& BasicTactic::connectToTrunk(bool privileged) {
  if (chain_.T_leaf_petiole().vec().norm() > 1E-3) {
    pipeline_->makeKeyframeFromCandidate();
  }

  auto neighbours = pose_graph_->at(current_vertex_id_)->spatialNeighbours();
  if (neighbours.size() == 1) {
    pose_graph_->at(current_vertex_id_, *neighbours.begin())
        ->setManual(privileged);
  } else if (neighbours.empty()) {
    LOG(DEBUG) << "Adding closure " << current_vertex_id_ << " --> "
               << chain_.trunkVertexId() << std::endl
               << std::endl
               << chain_.T_leaf_trunk().inverse() << std::endl;
    (void)pose_graph_->addEdge(current_vertex_id_, chain_.trunkVertexId(),
                               chain_.T_leaf_trunk().inverse(),
                               asrl::pose_graph::Spatial, privileged);
  }
  return current_vertex_id_;
}

void BasicTactic::updateLocalization(QueryCachePtr q_data, MapCachePtr m_data) {
  // Compute the current time in seconds.
  auto time_now = std::chrono::system_clock::now().time_since_epoch();
  double time_now_secs =
      static_cast<double>(time_now.count() *
                          std::chrono::system_clock::period::num) /
      std::chrono::system_clock::period::den;
  int64_t time_now_ns = time_now_secs * 1e9;

  // We need to know where we are to update the path tracker.,,
  if (!chain_.isLocalized()) {
    LOG(WARNING) << "Chain isn't localized; delaying localization update to "
                    "path tracker.";
    return;
  }

  // Get the contents for the localization message
  EdgeTransform T_root_trunk = chain_.T_trunk_target(0).inverse();
  EdgeTransform T_leaf_trunk = chain_.T_leaf_trunk();

  int64_t stamp = (*q_data->stamp).nanoseconds_since_epoch();

  // Try to update our estimate using steam trajectory extrapolation
  EdgeTransform T_leaf_trunk_extrapolated = T_leaf_trunk;
  bool updated_using_steam = false;

  if (config_.extrapolate_VO == true && q_data->trajectory.is_valid() == true) {
    bool trajectory_timed_out =
        time_now_ns - stamp > config_.extrapolate_timeout * 1e9;
    if (trajectory_timed_out) {
// warning suppressed for working on offline datasets
#if 0
      LOG(WARNING) << "The trajectory timed out after "
                   << (time_now_ns - stamp) * 1e-9
                   << " s before updating the path tracker.";
#endif
    } else {
#if 0
      // Send an update to the path tracker including the trajectory
      if (path_tracker_) {
        uint64_t im_stamp_ns = (*q_data->stamp).nanoseconds_since_epoch();
        path_tracker_->notifyNewLeaf(chain_, *q_data->trajectory,
                                     currentVertexID(), im_stamp_ns);
        updated_using_steam = true;
      }
#endif
    }  // valid trajectory
  }    // try to extrapolate

  // generate T_leaf_trunk in the sensor coordinate system (for the gimbal
  // extrapolation)

  // use the live transform as the default, but this should be updated but the
  // cached transform
  EdgeTransform T_s_v = *q_data->T_sensor_vehicle;

  // double check we have a cached transform
  // get the rig name
  auto& query_features = *q_data->rig_features;
  auto& rig_name = query_features[0].name;

  // get the map vertex
  auto map_vertex = pose_graph_->at(chain_.trunkVertexId());

  // retrieve the vehcile to camera transform for the map vertex
  auto rc_transforms =
      map_vertex->retrieveKeyframeData<robochunk::kinematic_msgs::Transform>(
          "/" + rig_name + "/T_sensor_vehicle");
  if (rc_transforms != nullptr) {  // check if we have the data. Some older
                                   // datasets may not have this saved
    Eigen::Matrix<double, 6, 1> tmp;
    auto mt = rc_transforms->mutable_translation();
    auto mr = rc_transforms->mutable_orientation();
    tmp << mt->x(), mt->y(), mt->z(), mr->x(), mr->y(), mr->z();
    T_s_v = lgmath::se3::TransformationWithCovariance(tmp);
  } else {
    LOG(WARNING) << "T_sensor_vehicle not set!";
  }

  // EdgeTransform T_leaf_trunk_sensor =
  // T_s_v_leaf*T_leaf_trunk_extrapolated*T_s_v_leaf.inverse();

  // get the transform between the sensor pose at the trunk and the leaf
  EdgeTransform T_leaf_trunk_sensor =
      T_s_v * T_leaf_trunk_extrapolated.inverse();  // *T_s_v.inverse();

#if 0
  // Publish the new localization if the chain is localized, otherise publish
  // the last valid persistent localization
  if (publisher_) {  // We need a publisher to publish...
    publisher_->updateLocalization(T_leaf_trunk, T_root_trunk,
                                   T_leaf_trunk_sensor, stamp);
  }
#endif

  updatePersistentLocalization(chain_.trunkVertexId(), T_leaf_trunk);

#if 0
  // Update the transform in the new path tracker if it is not nullptr and we
  // did not use STEAM
  if (path_tracker_ && !updated_using_steam) {
    path_tracker_->notifyNewLeaf(chain_, asrl::common::timing::toChrono(stamp),
                                 currentVertexID());
  }

  if (hover_controller_ && !updated_using_steam) {
    hover_controller_->notifyNewLeaf(
        chain_, asrl::common::timing::toChrono(stamp), currentVertexID());
  }
#endif

  // TODO THIS BLOCK WILL GO ONCE LOC CHAIN IS LOGGED SOMEWHERE MORE SUITABLE
  if (q_data->live_id.is_valid()) {
    // update the vertex with the VO status
    asrl::status_msgs::VOStatus status;
    status.set_leaf_image_stamp((*q_data->stamp).nanoseconds_since_epoch());
    status.set_leaf_processed_stamp(static_cast<int64_t>(time_now_secs * 1e9));
    //      status.computation_time_ms = q_data->qvo_timer.elapsedMs();
    status.set_twig_id(chain_.twigVertexId());
    status.set_branch_id(chain_.branchVertexId());
    status.set_trunk_id(chain_.trunkVertexId());

    // set the transforms.
    *status.mutable_t_leaf_trunk() << chain_.T_leaf_trunk();
    *status.mutable_t_leaf_trunk_extrapolated() << T_leaf_trunk_extrapolated;
    *status.mutable_t_leaf_twig() << chain_.T_leaf_twig();
    *status.mutable_t_twig_branch() << chain_.T_twig_branch();
    *status.mutable_t_branch_trunk() << chain_.T_branch_trunk();

    status.set_success(*m_data->success);
    status.set_keyframe_flag(q_data->new_vertex_flag.is_valid() &&
                             *(q_data->new_vertex_flag) != CREATE_CANDIDATE);
    auto vertex = pose_graph_->at(*q_data->live_id);
    // fill in the status
    vertex->insert<asrl::status_msgs::VOStatus>("/results/VO", status,
                                                *q_data->stamp);
  }
}
#endif
}  // namespace navigation
}  // namespace vtr
