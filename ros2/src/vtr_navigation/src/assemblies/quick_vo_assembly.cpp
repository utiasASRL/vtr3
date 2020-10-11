
#include <vtr_navigation/assemblies/quick_vo_assembly.hpp>
#include <vtr_navigation/modules.hpp>
#if false
#include <asrl/messages/QuickVOStatus.pb.h>
#include <asrl/messages/lgmath_conversions.hpp>
#endif
namespace vtr {
namespace navigation {

bool QuickVoAssembly::verify() const {
  if (modules_.empty()) {
    LOG(ERROR) << "This module assembly requires at least one module drived "
                  "from base class VertexCreationModule to function!";
    return false;
  }

  // TODO: Verify that a vertex creation test module exists at the end.
  auto downcasted_module =
      std::dynamic_pointer_cast<VertexCreationModule>(modules_.back());
  if (downcasted_module == nullptr) {
    LOG(ERROR) << "The final module in this assembly is of type, "
               << typeid(*modules_.back().get()).name()
               << " which is not derived from base class VertexCreationModule";
    return false;
  }

  return true;
}

void QuickVoAssembly::run(QueryCache &qdata, MapCache &mdata,
                          const std::shared_ptr<const Graph> &graph) {
  // get the current time in seconds
  auto time_before = std::chrono::system_clock::now().time_since_epoch();
  double time_before_secs =
      static_cast<double>(time_before.count() *
                          std::chrono::system_clock::period::num) /
      std::chrono::system_clock::period::den;

  BaseAssembly::run(qdata, mdata, graph);
#if false
  // Verify the vertex we'll save the stream to
  if (!qdata.live_id.is_valid()) return;
  if (!graph->contains(*qdata.live_id)) return;
  if (!qdata.rig_images.is_valid()) return;
  Vertex::Ptr vertex = graph->at(*qdata.live_id);

  // get the current time in seconds
  auto time_now = std::chrono::system_clock::now().time_since_epoch();
  double time_now_secs =
      static_cast<double>(time_now.count() *
                          std::chrono::system_clock::period::num) /
      std::chrono::system_clock::period::den;

  // SAVE VO STATUS //
  asrl::status_msgs::QuickVOStatus status;

  // keyframe vid
  status.set_keyframe_vid(vertex->id());
  // was vo successful
  status.set_success(mdata.success && *mdata.success);
  // is this frame a keyframe
  status.set_is_kf(qdata.new_vertex_flag &&
                   *(qdata.new_vertex_flag) != CREATE_CANDIDATE);

  // image timestamp
  status.set_image_stamp(qdata.stamp ? (*qdata.stamp).nanoseconds_since_epoch()
                                     : 0);
  // latency (image timetstamp -> now)
  status.set_latency_ms(time_now_secs * 1e3 - status.image_stamp() * 1e-6);
  // quick-vo computation time
  status.set_compute_ms((time_now_secs - time_before_secs) * 1e3);

  // vo transform prior (used for matching)
  if (mdata.T_q_m_prior) *status.mutable_t_f_kf_prior() << *mdata.T_q_m_prior;
  // vo transform posterior (final optimized)
  if (mdata.T_q_m) *status.mutable_t_f_kf_optimized() << *mdata.T_q_m;

  // fill in the status
  auto run = graph->run((*qdata.live_id).majorId());
  std::string qvo_status_str("/results/quick_vo");
  if (!run->hasVertexStream(qvo_status_str)) {
    run->registerVertexStream(qvo_status_str, true);
  }
  vertex->insert<asrl::status_msgs::QuickVOStatus>(qvo_status_str, status,
                                                   *qdata.stamp);
#endif
}

void QuickVoAssembly::updateGraph(QueryCache &qdata, MapCache &mdata,
                                  const std::shared_ptr<Graph> &graph,
                                  const VertexId &live_id) {
  // let the modules update the graph first.
  BaseAssembly::updateGraph(qdata, mdata, graph, live_id);

  // sanity check
  if (qdata.candidate_landmarks.is_valid() == false ||
      qdata.rig_features.is_valid() == false ||
      qdata.rig_images.is_valid() == false) {
    return;
  }

  // get the current vertex
  auto vertex = graph->at(live_id);
  auto persistent_id = graph->toPersistent(live_id);

  // get the current run id
  const Graph::RunIdType rid = live_id.majorId();

  // now update the live frame
  const auto &features = *qdata.rig_features;
  const auto &images = *qdata.rig_images;
  const auto &stamp = *qdata.stamp;

  // Iterate through each rig.
  auto rig_img_itr = images.begin();
  for (uint32_t rig_idx = 0; rig_idx < features.size(); ++rig_idx) {
    auto &rig_name = features[rig_idx].name;

    // create a landmark and observation message for this rig.
    vtr_messages::msg::RigLandmarks landmarks;
    vtr_messages::msg::RigObservations observations;
    vtr_messages::msg::RigCounts obs_cnt, lm_cnt;
    observations.name = rig_name;
    landmarks.name = rig_name;

    if (mdata.ransac_matches.is_valid() == false) {
      // likely the first frame, insert all the landmarks seen by the first
      // frame
      addAllLandmarks(landmarks, observations, rig_idx, qdata, mdata, graph,
                      persistent_id);
    } else {
      // otherwise, only add new landmarks.
      addLandmarksAndObs(landmarks, observations, rig_idx, qdata, mdata, graph,
                         persistent_id);
    }

    // insert the data into the vertex.

    // insert the vehicle->sensor transform
    Eigen::Matrix<double, 6, 1> T_s_v_vec = qdata.T_sensor_vehicle->vec();
    vtr_messages::msg::Transform T_s_v;
    T_s_v.translation.x = T_s_v_vec(0);
    T_s_v.translation.y = T_s_v_vec(1);
    T_s_v.translation.z = T_s_v_vec(2);
    T_s_v.orientation.x = T_s_v_vec(3);
    T_s_v.orientation.y = T_s_v_vec(4);
    T_s_v.orientation.z = T_s_v_vec(5);

    // fill the vehicle->sensor transform
    std::string tsv_str = rig_name + "_T_sensor_vehicle";
    if (!graph->hasVertexStream(rid, tsv_str)) {
      graph->registerVertexStream<vtr_messages::msg::Transform>(rid, tsv_str,
                                                                true);
    }
    vertex->insert(tsv_str, T_s_v, stamp);

    // make an empty velocity vector
    vtr_messages::msg::Velocity velocity;
    velocity.translational.x = 0.0;
    velocity.translational.y = 0.0;
    velocity.translational.z = 0.0;
    velocity.rotational.x = 0.0;
    velocity.rotational.y = 0.0;
    velocity.rotational.z = 0.0;

    // fill the velocities
    std::string vel_str = "_velocities";
    if (!graph->hasVertexStream(rid, vel_str)) {
      graph->registerVertexStream<vtr_messages::msg::Velocity>(rid, vel_str,
                                                               true);
    }
    vertex->insert(vel_str, velocity, stamp);

    // fill the landmarks
    std::string lm_str = rig_name + "_landmarks";
    if (!graph->hasVertexStream(rid, lm_str)) {
      graph->registerVertexStream<vtr_messages::msg::RigLandmarks>(rid, lm_str,
                                                                   true);
    }
    // fill the landmark counts
    std::string lm_cnt_str = lm_str + "_counts";
    if (!graph->hasVertexStream(rid, lm_cnt_str)) {
      graph->registerVertexStream<vtr_messages::msg::RigCounts>(rid, lm_cnt_str,
                                                                true);
    }

    // Record the number of observations and landmarks
    for (const auto &channel_obs : observations.channels) {
      auto &new_channel_cnt = obs_cnt.channels.emplace_back();
      if (!channel_obs.cameras.size()) continue;
      new_channel_cnt.count = channel_obs.cameras[0].keypoints.size();
    }
    for (const auto &channel_lm : landmarks.channels) {
      lm_cnt.channels.emplace_back().count = channel_lm.lm_info.size();
    }
    vertex->insert(lm_str, landmarks, stamp);
    vertex->insert(lm_cnt_str, lm_cnt, stamp);

    // fill the observations
    std::string obs_str = rig_name + "_observations";
    std::string obs_cnt_str = obs_str + "_counts";
    if (!graph->hasVertexStream(rid, obs_str)) {
      graph->registerVertexStream<vtr_messages::msg::RigObservations>(
          rid, obs_str, true);
    }
    if (!graph->hasVertexStream(rid, obs_cnt_str)) {
      graph->registerVertexStream<vtr_messages::msg::RigCounts>(
          rid, obs_cnt_str, true);
    }
    vertex->insert(obs_str, observations, stamp);
    vertex->insert(obs_cnt_str, obs_cnt, stamp);

    // fill the visualization images
    std::string vis_str = rig_name + "_visualization_images";
    if (!graph->hasVertexStream(rid, vis_str)) {
      graph->registerVertexStream<vtr_messages::msg::Image>(rid, vis_str, true);
    }

    // find the channel that contains the grayscale versions of the images and
    // use the first image for visualization
    for (auto channel_img_itr = rig_img_itr->channels.begin();
         channel_img_itr != rig_img_itr->channels.end(); channel_img_itr++) {
      if (channel_img_itr->name == "grayscale" &&
          !channel_img_itr->cameras.empty()) {
        auto proto_image = messages::copyImages(channel_img_itr->cameras[0]);
        vertex->insert(vis_str, proto_image, stamp);
        break;
      }
    }
    ++rig_img_itr;
  }
}

void QuickVoAssembly::addAllLandmarks(
    vtr_messages::msg::RigLandmarks &landmarks,
    vtr_messages::msg::RigObservations &observations, const int &rig_idx,
    const QueryCache &qdata, const MapCache &,
    const std::shared_ptr<Graph> &graph,
    const vtr_messages::msg::GraphPersistentId &persistent_id) {
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

  // Convert all of the landmarks to a protobuf message.
  landmarks = messages::copyLandmarks(rig_landmarks);
  for (int channel_idx = 0; channel_idx < (int)landmarks.channels.size();
       ++channel_idx) {
    auto &channel_landmarks = landmarks.channels[channel_idx];
    for (int lm_idx = 0; lm_idx < (int)channel_landmarks.points.size();
         lm_idx++) {
      // Set the match to other landmarks up.
      auto &match = channel_landmarks.matches.emplace_back();
      auto &from = match.from_id;
      from.persistent = persistent_id;
      from.rig = rig_idx;
      from.channel = channel_idx;
      from.camera = -1;
      from.idx = lm_idx;
    }
  }
}

void QuickVoAssembly::addChannelObs(
    vtr_messages::msg::ChannelObservations &channel_obs,
    const vtr::vision::ChannelFeatures &channel_features,
    const vtr::vision::ChannelLandmarks &,
    const vtr_messages::msg::GraphPersistentId &persistent_id,
    const int &rig_idx, const int &channel_idx) {
  channel_obs.name = channel_features.name;
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
      from.persistent = persistent_id;
      from.rig = rig_idx;
      from.channel = channel_idx;
      from.camera = camera_idx;
      from.idx = idx;

      auto &to = match.to_id.emplace_back();
      to.persistent = persistent_id;
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

void QuickVoAssembly::addLandmarksAndObs(
    vtr_messages::msg::RigLandmarks &landmarks,
    vtr_messages::msg::RigObservations &observations, const int &rig_idx,
    const QueryCache &qdata, const MapCache &mdata,
    const std::shared_ptr<Graph> &,
    const vtr_messages::msg::GraphPersistentId &persistent_id) {
  // Get a reference to the query landmarks/features
  const auto &rig_landmarks = (*qdata.candidate_landmarks)[rig_idx];
  const auto &rig_features = (*qdata.rig_features)[rig_idx];

  // get a reference to the map landmarks/obs
  auto &rig_map_lm = (*mdata.map_landmarks)[rig_idx];

  // get a reference to the RANSAC matches / map landmarks/obs
  vision::RigMatches all_matches;
  auto &ransac_matches = (*mdata.ransac_matches)[rig_idx];

  // if there are triangulated matches, concatenate them with the RANSAC matches
  if (mdata.triangulated_matches.is_valid() == true) {
    auto &new_matches =
        (*mdata.triangulated_matches)[rig_idx];  // newly triangulated matches
    all_matches = messages::concatenateMatches(ransac_matches, new_matches);
  } else {
    all_matches = ransac_matches;
  }

  // Iterate through all of the matches in this channel.
  for (uint32_t channel_idx = 0; channel_idx < all_matches.channels.size();
       ++channel_idx) {
    const auto &channel_landmarks = rig_landmarks.channels[channel_idx];
    const auto &channel_features = rig_features.channels[channel_idx];
    const auto &channel_matches = all_matches.channels[channel_idx].matches;
    const auto &map_channel_lm = rig_map_lm.observations.channels[channel_idx];

    // Get the number of candidate landmarks for this channel.
    auto num_candidate_landmarks =
        rig_landmarks.channels[channel_idx].points.cols();

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

void QuickVoAssembly::addObsToOldLandmarks(
    vtr_messages::msg::ChannelObservations &new_obs,
    const vision::SimpleMatches &matches,
    const vision::ChannelFeatures &features,
    const vision::ChannelObservations &map_lm_obs,
    std::vector<bool> &new_landmark_flags,
    const vtr_messages::msg::GraphPersistentId &persistent_id,
    const int &rig_idx, const int &channel_idx) {
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
      from.persistent = persistent_id;
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

void QuickVoAssembly::addNewLandmarksAndObs(
    vtr_messages::msg::ChannelLandmarks &new_landmarks,
    vtr_messages::msg::ChannelObservations &new_observations,
    const std::vector<bool> &new_landmark_flags,
    const vision::ChannelLandmarks &landmarks,
    const vision::ChannelFeatures &features,
    const vtr_messages::msg::GraphPersistentId &persistent_id,
    const int &rig_idx, const int &channel_idx) {
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
      from.persistent = persistent_id;
      from.rig = rig_idx;
      from.channel = channel_idx;
      from.camera = -1;
      from.idx = new_landmarks.points.size() - 1;

      // make an observation
      for (uint32_t camera_idx = 0; camera_idx < features.cameras.size();
           ++camera_idx) {
        const auto &camera_features = features.cameras[camera_idx];
        const auto &keypoint = camera_features.keypoints[lm_idx];
        auto camera_obs = new_observations.cameras[camera_idx];

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
        from.persistent = persistent_id;
        from.rig = rig_idx;
        from.channel = channel_idx;
        from.camera = camera_idx;
        from.idx = new_landmarks.points.size() - 1;

        auto &to = lm_match.to_id.emplace_back();
        to.persistent = persistent_id;
        to.rig = rig_idx;
        to.channel = channel_idx;
        to.camera = camera_idx;
        to.idx = new_landmarks.points.size() - 1;
      }
    }
  }
}

#if false
void QuickVoAssembly::updateLandmarks(
    asrl::vision_msgs::RigLandmarks &landmarks,
    asrl::vision_msgs::RigObservations &observations, const int &rig_idx,
    const QueryCache &qdata, const MapCache &mdata,
    const std::shared_ptr<Graph> &, const VertexId &live_id) {
  // sanity check
  if (mdata.map_landmarks.is_valid() == false)
    return;

  // Get a reference to the query landmarks/features
  const auto &rig_landmarks_asrl = (*mdata.map_landmarks)[rig_idx].landmarks;

  // Update the landmark data in the protobuf message
  messages::updateLandmarks(landmarks, rig_landmarks_asrl);
}
#endif

}  // namespace navigation
}  // namespace vtr