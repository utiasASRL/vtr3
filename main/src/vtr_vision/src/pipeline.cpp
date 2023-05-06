#include <vtr_vision/pipeline.hpp>
#include "vtr_tactic/modules/factory.hpp"


namespace vtr {
namespace vision {

using namespace tactic;
auto StereoPipeline::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                   const std::string &param_prefix) 
    -> ConstPtr {
  auto config = std::make_shared<Config>();
                                   
  // clang-format off
  config->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  config->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  config->bundle_adjustment = node->declare_parameter<std::vector<std::string>>(param_prefix + ".bundle_adjustment", config->bundle_adjustment);
  config->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
  // clang-format on
  return config;
}


StereoPipeline::StereoPipeline(
    const Config::ConstPtr &config,
    const std::shared_ptr<ModuleFactory> &module_factory,
    const std::string &name)
    : BasePipeline(module_factory, name), config_(config) {
    // preprocessing
    
  for (auto module : config_->preprocessing)
    preprocessing_.push_back(factory()->get("preprocessing." + module));

//   // odometry
  for (auto module : config_->odometry)
    odometry_.push_back(factory()->get("odometry." + module));
//   // localization
  for (auto module : config_->localization)
    localization_.push_back(factory()->get("localization." + module));
}

StereoPipeline::~StereoPipeline() {}

tactic::OutputCache::Ptr StereoPipeline::createOutputCache() const {
  return std::make_shared<tactic::OutputCache>();
}


void StereoPipeline::preprocess_(const tactic::QueryCache::Ptr &qdata0, const tactic::OutputCache::Ptr &output0,
                   const tactic::Graph::Ptr &graph,
                   const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto qdata = std::dynamic_pointer_cast<CameraQueryCache>(qdata0);
  
  if (!qdata->vis_mutex.valid()) {
    qdata->vis_mutex.emplace();
  }
  
  for (auto module : preprocessing_) module->run(*qdata0, *output0, graph, executor);
  
}

void StereoPipeline::runOdometry_(const tactic::QueryCache::Ptr &qdata0, const tactic::OutputCache::Ptr &output0,
                   const tactic::Graph::Ptr &graph,
                   const std::shared_ptr<tactic::TaskExecutor> &executor) {
  // auto qdata = std::dynamic_pointer_cast<CameraQueryCache>(qdata0);
  auto &qdata = dynamic_cast<CameraQueryCache &>(*qdata0);
 
  qdata.T_r_m.emplace(*qdata.T_r_v_odo);
  qdata.T_r_m_prior.emplace(*qdata.T_r_v_odo);
  CLOG(WARNING, "stereo.pipeline") << "T_r_v_odo set";

  CLOG(WARNING, "stereo.pipeline") << "First frame: " << *qdata.first_frame ;

  if (!(*qdata.first_frame)){
    qdata.timestamp_odo.emplace(timestamp_odo_);
    setOdometryPrior(qdata, graph);
  }
  CLOG(WARNING, "stereo.pipeline")
      << "Finished setting odometry prior, running modules";
  for (auto module : odometry_) module->run(*qdata0, *output0, graph, executor);

  // If VO failed, revert T_r_m to the initial prior estimate
  if (*qdata.odo_success == false) {
    CLOG(WARNING, "stereo.pipeline")
        << "VO FAILED, reverting to trajectory estimate.";
    *qdata.T_r_m = *qdata.T_r_m_prior;
  }

  // check if we have a non-failed frame
  if (*qdata.vertex_test_result == VertexTestResult::DO_NOTHING) {
    CLOG(WARNING, "stereo.pipeline")
        << "VO FAILED, trying to use the candidate query data to make "
           "a keyframe.";
    if (candidate_qdata_ != nullptr) {
      //A valid candidate exists, use it!
      qdata = *candidate_qdata_;
      *qdata.vertex_test_result = VertexTestResult::CREATE_VERTEX;
      candidate_qdata_ = nullptr;
    } else {
      CLOG(ERROR, "stereo.pipeline")
          << "Does not have a valid candidate query data.";
      // qdata->trajectory.clear();
      // trajectory is no longer valid
      trajectory_.reset();
    }
  } else {
    // keep a pointer to the trajectory
    // trajectory_ = qdata->trajectory.ptr();
    // trajectory_time_point_ = common::timing::toChrono(*qdata->stamp);
    /// keep this frame as a candidate for creating a keyframe
    if (*qdata.vertex_test_result == VertexTestResult::CREATE_CANDIDATE)
      candidate_qdata_ = std::make_shared<CameraQueryCache>(qdata);
  }

  if (*qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX)
    timestamp_odo_ = *qdata.stamp;


  // set result
  qdata.T_r_v_odo = *qdata.T_r_m;
}

void StereoPipeline::setOdometryPrior(CameraQueryCache &qdata,
                                      const tactic::Graph::Ptr &graph) {

  auto T_r_m_est = estimateTransformFromKeyframe(*qdata.timestamp_odo, *qdata.stamp,
                                                 true);

  // \todo The trajectory is not set better not to extrapolate for now.
  //Use the w_ vector instead of the trajectory. Like Lidar.
  *qdata.T_r_m_prior = T_r_m_est;
}

tactic::EdgeTransform StereoPipeline::estimateTransformFromKeyframe(
    const tactic::Timestamp &kf_stamp, const tactic::Timestamp &curr_stamp,
    bool check_expiry) {
  tactic::EdgeTransform T_q_m;
  // The elapsed time since the last keyframe
  // auto curr_time_point = common::timing::toChrono(curr_stamp);
  // auto dt_duration = curr_time_point - common::timing::toChrono(kf_stamp);
  // double dt = std::chrono::duration<double>(dt_duration).count();

  double dt = (curr_stamp - kf_stamp) / 1e9; //convert to seconds

  // Make sure the trajectory is current
  if (check_expiry && trajectory_) {
    //auto traj_dt_duration = curr_time_point - trajectory_time_point_;
    //double traj_dt = std::chrono::duration<double>(traj_dt_duration).count();
    if (dt > 1.0 /* tactic->config().extrapolate_timeout */) {
      CLOG(WARNING, "stereo.pipeline")
          << "The trajectory expired after " << dt
          << " s for estimating the transform from keyframe at "
          << kf_stamp;
      trajectory_.reset();
    }
  }

  // we need to update the new T_q_m prediction
  Eigen::Matrix<double, 6, 6> cov =
      Eigen::Matrix<double, 6, 6>::Identity() * pow(dt, 2.0);
  // scale the rotational uncertainty to be one order of magnitude lower than
  // the translational uncertainty.
  cov.block(3, 3, 3, 3) /= 10;
  if (trajectory_ != nullptr) {
    // Query the saved trajectory estimator we have with the candidate frame
    // time
    auto candidate_time =
        steam::traj::Time(static_cast<int64_t>(kf_stamp));
    auto candidate_eval = trajectory_->getPoseInterpolator(candidate_time);
    // Query the saved trajectory estimator we have with the current frame time
    auto query_time =
        steam::traj::Time(static_cast<int64_t>(curr_stamp));
    auto curr_eval = trajectory_->getPoseInterpolator(query_time);

    // find the transform between the candidate and current in the vehicle frame
    T_q_m = candidate_eval->evaluate().inverse() * curr_eval->evaluate();
    // give it back to the caller, TODO: (old) We need to get the covariance out
    // of the trajectory.

    // This ugliness of setting the time is because we don't have a reliable and
    // tested way of predicting the covariance. This is used by the stereo
    // matcher to decide how tight it should set its pixel search
    T_q_m.setCovariance(cov);

    CLOG(DEBUG, "stereo.pipeline")
        << "Estimated T_q_m (based on keyframe) from steam trajectory.";
  } else {
    // since we don't have a trajectory, we can't accurately estimate T_q_m
    T_q_m.setCovariance(4 * cov);

    CLOG(DEBUG, "stereo.pipeline")
        << "Estimated T_q_m is identity with high covariance.";
  }
  return T_q_m;
}



void StereoPipeline::onVertexCreation_(const QueryCache::Ptr &qdata0,
                                      const OutputCache::Ptr &,
                                      const Graph::Ptr &graph,
                                      const TaskExecutor::Ptr &) {

  auto qdata = std::dynamic_pointer_cast<CameraQueryCache>(qdata0);
  auto live_id = *qdata->vid_odo;

  saveLandmarks(*qdata, graph, live_id);

  // if (*qdata->first_frame) return;

//     // sliding-window bundle adjustment
// #ifdef VTR_DETERMINISTIC
//   runBundleAdjustment(qdata, graph, live_id);
// #else
//   /// Run pipeline according to the state
//   CLOG(DEBUG, "stereo.pipeline") << "Launching the bundle adjustment thread.";
//   std::lock_guard<std::mutex> lck(bundle_adjustment_mutex_);
//   if (bundle_adjustment_thread_future_.valid())
//     bundle_adjustment_thread_future_.get();
//   bundle_adjustment_thread_future_ =
//       std::async(std::launch::async, [this, qdata, graph, live_id]() {
//         el::Helpers::setThreadName("bundle-adjustment-thread");
//         runBundleAdjustment(qdata, graph, live_id);
//       });
// #endif
}

void StereoPipeline::runLocalization_(const tactic::QueryCache::Ptr &qdata0, const tactic::OutputCache::Ptr &output0,
                        const tactic::Graph::Ptr &graph,
                        const std::shared_ptr<tactic::TaskExecutor> &executor) {
  
  auto qdata = std::dynamic_pointer_cast<CameraQueryCache>(qdata0);

  // {
  //   /// Localization waits at least until the current bundle adjustment has
  //   /// finished.
  //   std::lock_guard<std::mutex> lck(bundle_adjustment_mutex_);
  //   if (bundle_adjustment_thread_future_.valid())
  //     bundle_adjustment_thread_future_.get();
  // }

  //What did this do?
  //qdata->map_id.emplace(*qdata->map_id);
  
  //qdata->T_r_m_prior.emplace(*qdata->T_r_v_loc);
  //qdata->T_r_m.emplace(*qdata->T_r_v_loc);
  qdata->localization_status.emplace();
  //qdata->loc_timer.emplace();

  for (auto module : localization_) module->run(*qdata0, *output0, graph, executor);

  auto live_id = *qdata->vid_odo;
  // for (auto module : localization_)
  //   module->updateGraph(*qdata0, graph, live_id);

  /// \todo yuchen move the actual graph saving to somewhere appropriate.
  saveLocalization(*qdata, graph, live_id);

  if (qdata->loc_success) {
    CLOG(WARNING, "stereo.pipeline") << "Localization pipeline failed.";
  } else {
    *qdata->loc_success = true;
    *qdata->T_r_v_loc = *qdata->T_r_m;
  }
  
    
}


void StereoPipeline::saveLandmarks(CameraQueryCache &qdata,
                                   const Graph::Ptr &graph,
                                   const VertexId &live_id) {
  // sanity check
  if (qdata.candidate_landmarks.valid() == false ||
      qdata.rig_features.valid() == false ||
      qdata.rig_images.valid() == false) {
    return;
  }

  // get the current vertex
  auto vertex = graph->at(live_id);
  // auto persistent_id = graph->toPersistent(live_id);
  auto persistent_id = live_id;

  // now update the live frame
  const auto &features = *qdata.rig_features;
  const auto &images = *qdata.rig_images;
  const auto &stamp = *qdata.stamp;

  // Iterate through each rig.
  for (uint32_t rig_idx = 0; rig_idx < features.size(); ++rig_idx) {
    auto &rig_name = features[rig_idx].name;

    // create a landmark and observation message for this rig.
    RigLandmarksMsg landmarks;
    RigObservationsMsg observations;
    RigCountsMsg obs_cnt, lm_cnt;
    observations.name = rig_name;
    landmarks.name = rig_name;

    if (qdata.ransac_matches.valid() == false)
      // likely the first frame, insert all the landmarks seen by the first
      // frame
      addAllLandmarks(landmarks, observations, rig_idx, qdata, graph,
                      persistent_id);
    else
      // otherwise, only add new landmarks.
      addLandmarksAndObs(landmarks, observations, rig_idx, qdata, graph,
                         persistent_id);

      // addAllLandmarksRGB(landmarks, observations, rig_idx, qdata, graph,
      //                    persistent_id);

    // record the number of observations and landmarks
    for (const auto &channel_lm : landmarks.channels) {
      lm_cnt.channels.emplace_back().count = channel_lm.lm_info.size();
    }
    for (const auto &channel_obs : observations.channels) {
      auto &new_channel_cnt = obs_cnt.channels.emplace_back();
      if (!channel_obs.cameras.size()) continue;
      new_channel_cnt.count = channel_obs.cameras[0].keypoints.size();
    }

    // Insert the data into the vertex.

    // fill the landmarks and landmark counts
    std::string lm_str = rig_name + "_landmarks";
    using LM_Msg = storage::LockableMessage<RigLandmarksMsg>;
    auto lm_msg =
        std::make_shared<LM_Msg>(std::make_shared<RigLandmarksMsg>(landmarks), *qdata.stamp);

    vertex->insert<RigLandmarksMsg>(lm_str, "vtr_messages/msg/RigLandmarks", lm_msg);
      


    std::string lm_cnt_str = lm_str + "_counts";
    using LM_Cnt_Msg = storage::LockableMessage<RigCountsMsg>;
    auto lm_cnt_msg =
            std::make_shared<LM_Cnt_Msg>(std::make_shared<RigCountsMsg>(lm_cnt), *qdata.stamp);
    vertex->insert<RigCountsMsg>(lm_cnt_str, "vtr_messages/msg/RigCounts", lm_cnt_msg);


    std::string obs_str = rig_name + "_observations";
    using Obs_Msg = storage::LockableMessage<RigObservationsMsg>;
    auto obs_msg =
            std::make_shared<Obs_Msg>(std::make_shared<RigObservationsMsg>(observations), *qdata.stamp);
    vertex->insert<RigObservationsMsg>(obs_str, "vtr_messages/msg/RigObservations", obs_msg);




    std::string obs_cnt_str = obs_str + "_counts";
    using Obs_Cnt_Msg = storage::LockableMessage<RigCountsMsg>;
    auto obs_cnt_msg =
            std::make_shared<Obs_Cnt_Msg>(std::make_shared<RigCountsMsg>(obs_cnt), *qdata.stamp);
    vertex->insert<RigCountsMsg>(obs_cnt_str, "vtr_messages/msg/RigCounts", obs_cnt_msg);



  //   graph->registerVertexStream<RigLandmarksMsg>(rid, lm_str);
  //   graph->registerVertexStream<RigCountsMsg>(rid, lm_cnt_str);
  //   vertex->insert(lm_str, landmarks, stamp);
  //   vertex->insert(lm_cnt_str, lm_cnt, stamp);

  //   // fill the observations and observation counts
  //   std::string obs_str = rig_name + "_observations";
  //   std::string obs_cnt_str = obs_str + "_counts";
  //   graph->registerVertexStream<RigObservationsMsg>(rid, obs_str);
  //   graph->registerVertexStream<RigCountsMsg>(rid, obs_cnt_str);
  //   vertex->insert(obs_str, observations, stamp);
  //   vertex->insert(obs_cnt_str, obs_cnt, stamp);

    // insert the vehicle->sensor transform
    //TODO: change to 
    Eigen::Matrix<double, 6, 1> T_s_v_vec = qdata.T_s_r->vec();
    TransformMsg T_s_v;
    T_s_v.translation.x = T_s_v_vec(0);
    T_s_v.translation.y = T_s_v_vec(1);
    T_s_v.translation.z = T_s_v_vec(2);
    T_s_v.orientation.x = T_s_v_vec(3);
    T_s_v.orientation.y = T_s_v_vec(4);
    T_s_v.orientation.z = T_s_v_vec(5);

    // fill the vehicle->sensor transform
    std::string tsv_str = rig_name + "_T_sensor_vehicle";
    using Tsv_Msg = storage::LockableMessage<TransformMsg>;
    auto tsv_msg =
            std::make_shared<Tsv_Msg>(std::make_shared<TransformMsg>(T_s_v), *qdata.stamp);
    vertex->insert<TransformMsg>(tsv_str, " vtr_messages/msg/Transform", tsv_msg);

    // graph->registerVertexStream<TransformMsg>(rid, tsv_str);
    // vertex->insert(tsv_str, T_s_v, stamp);

    // make an empty velocity vector
    VelocityMsg velocity;
    velocity.translational.x = 0.0;
    velocity.translational.y = 0.0;
    velocity.translational.z = 0.0;
    velocity.rotational.x = 0.0;
    velocity.rotational.y = 0.0;
    velocity.rotational.z = 0.0;
    // fill the velocities
    std::string vel_str = rig_name + "_velocities";
    using Vel_Msg = storage::LockableMessage<VelocityMsg>;
    auto vel_msg =
            std::make_shared<Vel_Msg>(std::make_shared<VelocityMsg>(velocity), *qdata.stamp);
    vertex->insert<VelocityMsg>(vel_str, "vtr_messages/msg/Velocity", vel_msg);

    // graph->registerVertexStream<VelocityMsg>(rid, vel_str);
    // vertex->insert(vel_str, velocity, stamp);

  //   // fill the visualization images
  //   std::string vis_str = rig_name + "_visualization_images";
  //   graph->registerVertexStream<ImageMsg>(rid, vis_str);
  //   // find the channel that contains the grayscale versions of the images and
  //   // use the first image for visualization
  //   for (auto channel_img_itr = rig_img_itr->channels.begin();
  //        channel_img_itr != rig_img_itr->channels.end(); channel_img_itr++) {
  //     if (channel_img_itr->name == "RGB" &&
  //         !channel_img_itr->cameras.empty()) {
  //       auto image_msg = messages::copyImages(channel_img_itr->cameras[0]);
  //       vertex->insert(vis_str, image_msg, stamp);
  //       break;
  //     }
  //   }
  //   ++rig_img_itr;
  }
  // CLOG(DEBUG, "stereo.pipeline")
  //     << "Stored features, landmarks, empty velocity vector and "
  //        "visualization images into vertex "
  //     << vertex->id();
}



void StereoPipeline::addAllLandmarks(
    RigLandmarksMsg &landmarks, RigObservationsMsg &observations,
    const int &rig_idx, const CameraQueryCache &qdata, const Graph::Ptr &,
    const VertexId &persistent_id) {
  // Get a reference to the query landmarks/features
  const auto &rig_landmarks = (*qdata.candidate_landmarks)[rig_idx];
  const auto &rig_features = (*qdata.rig_features)[rig_idx];

  // Iterate through each channel in the rig.
  for (uint32_t channel_idx = 0; channel_idx < rig_features.channels.size();
       ++channel_idx) {
    // Set up the observations for this channel.
    auto &channel_obs = observations.channels.emplace_back();
    addChannelObs(channel_obs, rig_features.channels[channel_idx],
                  rig_landmarks.channels[channel_idx], persistent_id, rig_idx,
                  channel_idx);
  }

  // Convert all of the landmarks to a ros message.
  landmarks = messages::copyLandmarks(rig_landmarks);
  for (int channel_idx = 0; channel_idx < (int)landmarks.channels.size();
       ++channel_idx) {
    auto &channel_landmarks = landmarks.channels[channel_idx];
    for (int lm_idx = 0; lm_idx < (int)channel_landmarks.points.size();
         lm_idx++) {
      // Set the match to other landmarks up.
      auto &match = channel_landmarks.matches.emplace_back();
      auto &from = match.from_id;
      from.vid = persistent_id;
      from.rig = rig_idx;
      from.channel = channel_idx;
      from.camera = -1;
      from.idx = lm_idx;
    }
  }
}



void StereoPipeline::addChannelObs(
    ChannelObservationsMsg &channel_obs,
    const vision::ChannelFeatures &channel_features,
    const vision::ChannelLandmarks &, const VertexId &persistent_id,
    const int &rig_idx, const int &channel_idx) {
  channel_obs.name = channel_features.name;

  // iterate through the cameras
  for (uint32_t camera_idx = 0; camera_idx < channel_features.cameras.size();
       ++camera_idx) {
    const auto &camera_features = channel_features.cameras[camera_idx];
    auto &camera_obs = channel_obs.cameras.emplace_back();
    camera_obs.name = camera_features.name;

    // iterate through the keypoints
    for (uint32_t idx = 0; idx < camera_features.keypoints.size(); ++idx) {
      const auto &keypoint = camera_features.keypoints[idx];
      // fill in the match (in the case of brand new keypoints it just matches
      // to itself)
      auto &match = camera_obs.landmarks.emplace_back();
      auto &from = match.from_id;
      from.vid = persistent_id;
      from.rig = rig_idx;
      from.channel = channel_idx;
      from.camera = camera_idx;
      from.idx = idx;

      auto &to = match.to_id.emplace_back();
      to.vid = persistent_id;
      to.rig = rig_idx;
      to.channel = channel_idx;
      to.camera = camera_idx;
      to.idx = idx;

      // Set the keypoint in the message
      auto &obs_keypoint = camera_obs.keypoints.emplace_back();
      auto &kp_pos = obs_keypoint.position;
      kp_pos.x = keypoint.pt.x;
      kp_pos.y = keypoint.pt.y;

      // set the precision in the message
      const auto &feat_info = camera_features.feat_infos[idx];
      camera_obs.precisions.emplace_back(feat_info.precision);
      camera_obs.covariances.emplace_back(feat_info.covariance(0, 0));
      camera_obs.covariances.emplace_back(feat_info.covariance(0, 1));
      camera_obs.covariances.emplace_back(feat_info.covariance(1, 0));
      camera_obs.covariances.emplace_back(feat_info.covariance(1, 1));
    }
  }
}




void StereoPipeline::addLandmarksAndObs(
    RigLandmarksMsg &landmarks, RigObservationsMsg &observations,
    const int &rig_idx, const CameraQueryCache &qdata, const Graph::Ptr &,
    const VertexId &persistent_id) {
  // Get a reference to the query landmarks/features
  const auto &rig_landmarks = (*qdata.candidate_landmarks)[rig_idx];
  const auto &rig_features = (*qdata.rig_features)[rig_idx];

  // get a reference to the map landmarks/obs
  auto &rig_map_lm = (*qdata.map_landmarks)[rig_idx];

  // get a reference to the RANSAC matches / map landmarks/obs
  vision::RigMatches all_matches;
  auto &ransac_matches = (*qdata.ransac_matches)[rig_idx];

#if false
  // if there are triangulated matches, concatenate them with the RANSAC matches
  if (qdata.triangulated_matches.valid() == true) {
    auto &new_matches =
        (*qdata.triangulated_matches)[rig_idx];  // newly triangulated matches
    all_matches = messages::concatenateMatches(ransac_matches, new_matches);
  } else {
#endif
  all_matches = ransac_matches;
#if false
  }
#endif

  // Iterate through all of the matches in this channel.
  for (uint32_t channel_idx = 0; channel_idx < all_matches.channels.size();
       ++channel_idx) {
    const auto &channel_landmarks = rig_landmarks.channels[channel_idx];
    const auto &channel_features = rig_features.channels[channel_idx];
    const auto &channel_matches = all_matches.channels[channel_idx].matches;
    const auto &map_channel_lm = rig_map_lm.observations.channels[channel_idx];

    // LOG(INFO) << "Add landmarks: " << channel_features.name;

    // Get the number of candidate landmarks for this channel.
    auto num_candidate_landmarks =
        rig_landmarks.channels[channel_idx].points.cols();

    // LOG(INFO) << "Num candidate landmarks: " << num_candidate_landmarks;

    // create a new set of observations for this channel.
    auto &channel_obs = observations.channels.emplace_back();
    channel_obs.name = channel_features.name;

    // add the appropriate amount of cameras for this observation
    while (channel_obs.cameras.size() < channel_features.cameras.size()) {
      channel_obs.cameras.emplace_back();
    }

    // set up the landmark flags.
    std::vector<bool> new_landmark_flags;
    new_landmark_flags.resize(num_candidate_landmarks, true);

    // Add observations to old landmarks we have matches to.
    // For learned features (RGB channel) there are 0 matches (as we ignore
    // them for stereo/odometry matching) and all landmarks will be added by
    // the next function (addNewLandmarksAndObs).
    addObsToOldLandmarks(channel_obs, channel_matches, channel_features,
                         map_channel_lm, new_landmark_flags, persistent_id,
                         rig_idx, channel_idx);

    // Create new landmarks for candidate landmarks we do not have matches to.
    auto &new_channel_landmarks = landmarks.channels.emplace_back();
    addNewLandmarksAndObs(new_channel_landmarks, channel_obs,
                          new_landmark_flags, channel_landmarks,
                          channel_features, persistent_id, rig_idx,
                          channel_idx);

  }  // end for channel
}

void StereoPipeline::addObsToOldLandmarks(
    ChannelObservationsMsg &new_obs, const vision::SimpleMatches &matches,
    const vision::ChannelFeatures &features,
    const vision::ChannelObservations &map_lm_obs,
    std::vector<bool> &new_landmark_flags,
    const VertexId &persistent_id, const int &rig_idx,
    const int &channel_idx) {
  // Iterate through every match
  for (uint32_t match_idx = 0; match_idx < matches.size(); ++match_idx) {
    auto &match = matches[match_idx];
    // flag this landmark as not new.
    new_landmark_flags[match.second] = false;

    // Go through each camera and add the observation.
    for (uint32_t camera_idx = 0; camera_idx < features.cameras.size();
         ++camera_idx) {
      // Get a reference to the feature / keypoint from the query frame.
      const auto &camera_features = features.cameras[camera_idx];
      const auto &keypoint = camera_features.keypoints[match.second];

      // Set the keypoint in the message.
      auto &camera_obs = new_obs.cameras[camera_idx];
      auto &obs_keypoint = camera_obs.keypoints.emplace_back();
      auto &kp_pos = obs_keypoint.position;
      kp_pos.x = keypoint.pt.x;
      kp_pos.y = keypoint.pt.y;

      // set the precision in the message
      const auto &feat_info = camera_features.feat_infos[match.second];
      camera_obs.precisions.emplace_back(feat_info.precision);
      camera_obs.covariances.emplace_back(feat_info.covariance(0, 0));
      camera_obs.covariances.emplace_back(feat_info.covariance(0, 1));
      camera_obs.covariances.emplace_back(feat_info.covariance(1, 0));
      camera_obs.covariances.emplace_back(feat_info.covariance(1, 1));

      // fill in the match (The live vertices information)
      auto &lm_match = camera_obs.landmarks.emplace_back();
      auto &from = lm_match.from_id;
      from.vid = persistent_id;
      from.rig = rig_idx;
      from.channel = channel_idx;
      from.camera = camera_idx;
      from.idx = match.second;

      // Add the index to the landmark. (this might be many in the case of
      // multi-experience).
      auto &map_lm = map_lm_obs.cameras[camera_idx].landmarks[match.first];
      for (auto &lm_idx : map_lm.to) {
        lm_match.to_id.emplace_back(messages::copyLandmarkId(lm_idx));
      }  // end for lm_idx

    }  // end for camera
  }    // end for match
}

void StereoPipeline::addNewLandmarksAndObs(
    ChannelLandmarksMsg &new_landmarks,
    ChannelObservationsMsg &new_observations,
    const std::vector<bool> &new_landmark_flags,
    const vision::ChannelLandmarks &landmarks,
    const vision::ChannelFeatures &features,
    const VertexId &persistent_id, const int &rig_idx,
    const int &channel_idx) {
  // Iterate through the candidate landmarks, if its matched pass by, otherwise
  // add it.
  auto num_new_landmarks = 0;
  // determine the number of new landmarks
  for (uint32_t idx = 0; idx < new_landmark_flags.size(); ++idx) {
    if (new_landmark_flags[idx] == true) {
      num_new_landmarks++;
    }
  }
  // set up the name and descriptor type of these new landmarks.
  new_landmarks.name = landmarks.name;

  auto &desc_type = new_landmarks.desc_type;
  desc_type = messages::copyDescriptorType(landmarks.appearance.feat_type);

  // Allocate memory to fill in the descriptors.
  auto datasize =
      num_new_landmarks * landmarks.appearance.feat_type.bytes_per_desc;
  auto &descriptors = new_landmarks.descriptors;
  descriptors.resize(datasize);

  // determine the descriptor step size.
  auto step_size = landmarks.appearance.feat_type.bytes_per_desc;

  // Iterate through every candidate landmark.
  for (uint32_t lm_idx = 0; lm_idx < landmarks.points.cols(); ++lm_idx) {
    // If this is a landmark with no observation, then add it to the message.
    if (new_landmark_flags[lm_idx] == true) {
      // move over the kp info
      auto &lm_info = new_landmarks.lm_info.emplace_back();

      // fill in the landmark feature info
      lm_info.laplacian_bit =
          landmarks.appearance.feat_infos[lm_idx].laplacian_bit;
      lm_info.scale = landmarks.appearance.keypoints[lm_idx].octave;
      lm_info.orientation = landmarks.appearance.keypoints[lm_idx].angle;
      lm_info.response = landmarks.appearance.keypoints[lm_idx].response;
      // lm_info.precision = (#); // precision isn't set in the landmark

      // move over the point
      auto &point = new_landmarks.points.emplace_back();
      auto &channel_point = landmarks.points.col(lm_idx);
      point.x = channel_point(0);
      point.y = channel_point(1);
      point.z = channel_point(2);
      point.w = 1.0;

      // move over the covariance
      auto &channel_cov = landmarks.covariances.col(lm_idx);
      for (unsigned i = 0; i < 9; ++i)
        new_landmarks.covariance.emplace_back(channel_cov(i));
      new_landmarks.num_vo_observations.emplace_back(1);

      // move over the descriptor
      float *src_ptr =
          (float *)&landmarks.appearance.descriptors.data[lm_idx * step_size];
      float *dst_ptr =
          (float *)&descriptors[(new_landmarks.points.size() - 1) * step_size];
      memcpy(dst_ptr, src_ptr, step_size);

      // set the landmark validity
      new_landmarks.valid.emplace_back(landmarks.valid.at(lm_idx));

      // Set the match to other landmarks up.
      auto &match = new_landmarks.matches.emplace_back();
      auto &from = match.from_id;
      from.vid = persistent_id;
      from.rig = rig_idx;
      from.channel = channel_idx;
      from.camera = -1;
      from.idx = new_landmarks.points.size() - 1;

      // make an observation
      for (uint32_t camera_idx = 0; camera_idx < features.cameras.size();
           ++camera_idx) {
        const auto &camera_features = features.cameras[camera_idx];
        const auto &keypoint = camera_features.keypoints[lm_idx];
        auto &camera_obs = new_observations.cameras[camera_idx];

        camera_obs.name = camera_features.name;

        // Set the keypoint in the message.
        auto &obs_keypoint = camera_obs.keypoints.emplace_back();
        auto &kp_pos = obs_keypoint.position;
        kp_pos.x = keypoint.pt.x;
        kp_pos.y = keypoint.pt.y;

        // Set the precision in the message.
        const auto &feat_info = camera_features.feat_infos[lm_idx];
        camera_obs.precisions.emplace_back(feat_info.precision);
        camera_obs.covariances.emplace_back(feat_info.covariance(0, 0));
        camera_obs.covariances.emplace_back(feat_info.covariance(0, 1));
        camera_obs.covariances.emplace_back(feat_info.covariance(1, 0));
        camera_obs.covariances.emplace_back(feat_info.covariance(1, 1));

        // fill in the match (in the case of brand new keypoints it just matches
        // to itself):
        auto &lm_match = camera_obs.landmarks.emplace_back();
        auto &from = lm_match.from_id;
        from.vid = persistent_id;
        from.rig = rig_idx;
        from.channel = channel_idx;
        from.camera = camera_idx;
        from.idx = new_landmarks.points.size() - 1;

        auto &to = lm_match.to_id.emplace_back();
        to.vid = persistent_id;
        to.rig = rig_idx;
        to.channel = channel_idx;
        to.camera = camera_idx;
        to.idx = new_landmarks.points.size() - 1;
      }
    }
  }
}



void StereoPipeline::saveLocalization(CameraQueryCache &qdata,
                                      const Graph::Ptr &graph,
                                      const VertexId &live_id) {
  // sanity check
  if (!qdata.ransac_matches.valid()) {
    CLOG(ERROR, "stereo.pipeline")
        << "LocalizerAssembly::" << __func__ << "() ransac matches not present";
    return;
  } else if (!qdata.map_landmarks.valid()) {
    CLOG(ERROR, "stereo.pipeline")
        << "LocalizerAssembly::" << __func__ << "() map landmarks not present";
    return;
  } else if (!qdata.localization_status.valid()) {
    CLOG(ERROR, "stereo.pipeline") << "LocalizerAssembly::" << __func__
                                   << "() localization status not present";
    return;
  } else if (!qdata.migrated_landmark_ids.valid()) {
    CLOG(ERROR, "stereo.pipeline") << "LocalizerAssembly::" << __func__
                                   << "() migrated landmark ID's not present";
    return;
  }

  // save the localization results for analysis
  saveLocResults(qdata, graph, live_id);

  // we need to take all of the landmark matches and store them in the new
  // landmarks
  auto &inliers = *qdata.ransac_matches;
  auto &query_landmarks = *qdata.map_landmarks;
  auto migrated_landmark_ids = *qdata.migrated_landmark_ids;
  auto &rig_names = *qdata.rig_names;
  auto live_vtx = graph->at(live_id);

  // map to keep track of loaded landmarks.
  std::map<VertexId, std::shared_ptr<RigLandmarksMsg>> landmark_map;

  // Go through each rig.
  for (uint32_t rig_idx = 0; rig_idx < inliers.size(); ++rig_idx) {
    std::string &rig_name = rig_names.at(rig_idx);
    auto &rig_inliers = inliers[rig_idx];
    vtr_messages::msg::Matches matches_msg;

    // go through each channel.
    for (uint32_t channel_idx = 0; channel_idx < rig_inliers.channels.size();
         ++channel_idx) {
      // Get the inliers and observations
      auto &channel_inliers = rig_inliers.channels[channel_idx];
      auto &channel_obs =
          query_landmarks[rig_idx].observations.channels[channel_idx];
      // 1. iterate through the channel inliers...
      for (auto &match : channel_inliers.matches) {
        // get the observation to the landmark.
        auto &lm_obs = channel_obs.cameras[0].landmarks[match.second].to[0];
        VertexId q_lm_vertex = lm_obs.vid;
        // if we havent loaded these landmarks, then load them.
        if (landmark_map.find(q_lm_vertex) == landmark_map.end()) {
          auto vertex = graph->at(q_lm_vertex);

          auto locked_lm_msgs = vertex->retrieve<vtr_messages::msg::RigLandmarks>(
          rig_name + "_landmarks", "vtr_messages/msg/RigLandmarks");
          auto locked_msg = locked_lm_msgs->sharedLocked();
          landmark_map[q_lm_vertex] = locked_msg.get().getDataPtr();
        }

        // Get references / pointers to the landmarks.
        auto &query_landmarks = landmark_map[q_lm_vertex];
        auto channel_landmarks = query_landmarks->channels[channel_idx];
        // get the landmark track associated with the map landmark.
        auto &lm_track =
            migrated_landmark_ids[match.first];  // todo (Ben): verify this all
                                                 // works properly

        // match.first = idx into query lm
        // match.second = list of landmark ids
        // Get the  landmark track (list of matched landmarks across
        // experiences) associated with the query landmark.
        auto query_match = channel_landmarks.matches[lm_obs.index];
        // If this landmark has not been associated yet, then copy over the
        // matched landmarks track to the new landmark.
        if (!lm_track.to_id.empty()) {
          query_match.to_id = migrated_landmark_ids[match.first].to_id;
        }
        // add the map landmark as a to landmark in the matches.
        auto new_match = migrated_landmark_ids[match.first].from_id;
        query_match.to_id.push_back(new_match);

        // direct matches used for collaborative landmark tracking (first
        // pass)
        vtr_messages::msg::Match direct_match_msg;
        direct_match_msg.from_id = query_match.from_id;
        direct_match_msg.to_id.push_back(new_match);
        matches_msg.matches.push_back(direct_match_msg);
      }
    }


    // Save the matches to map landmarks
    std::string landmark_match_str(rig_name + "_landmarks_matches");
    using LM_Match_Msg = storage::LockableMessage<vtr_messages::msg::Matches>;
    auto lm_match_msg =
        std::make_shared<LM_Match_Msg>(std::make_shared<vtr_messages::msg::Matches>(matches_msg), *qdata.stamp);
    live_vtx->insert<vtr_messages::msg::Matches>(landmark_match_str, "vtr_messages/msg/Matches", lm_match_msg);
  }
}

void StereoPipeline::saveLocResults(CameraQueryCache &qdata,
                                    const Graph::Ptr &graph,
                                    const VertexId &live_id) {
  auto &inliers = *qdata.ransac_matches;
  auto status = *qdata.localization_status;
  status.keyframe_time = *qdata.stamp;
  status.query_id = live_id;
  pose_graph::VertexId map_id = *qdata.vid_loc;
  status.map_id = map_id;
  status.success = *qdata.loc_success;
  //status.localization_computation_time_ms = (*qdata.loc_timer).elapsedMs();

  // if (qdata.T_r_m.valid()) {
  //   status.t_query_map << *qdata.T_r_m;
  // }

  for (auto &rig : inliers) {
    for (auto &channel : rig.channels) {
      status.inlier_channel_matches.push_back(channel.matches.size());
    }
  }

  // get the run id and vertex
  auto vertex = graph->at(live_id);

  // fill in the status
  std::string loc_status_str("results_localization");
  using LM_Loc_Res_Msg = storage::LockableMessage<vtr_messages::msg::LocalizationStatus>;
  auto lm_loc_msg = std::make_shared<LM_Loc_Res_Msg>(std::make_shared<vtr_messages::msg::LocalizationStatus>(status), *qdata.stamp);

  vertex->insert<vtr_messages::msg::LocalizationStatus>(loc_status_str, "vtr_messages/msg/LocalizationStatus", lm_loc_msg);
}


} //vision
} //vtr