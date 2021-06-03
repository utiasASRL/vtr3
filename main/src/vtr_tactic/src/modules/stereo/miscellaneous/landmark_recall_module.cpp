#include <vtr_tactic/modules/stereo/miscellaneous/landmark_recall_module.hpp>

namespace vtr {
namespace tactic {

void LandmarkRecallModule::runImpl(QueryCache &qdata, MapCache &,
                                   const Graph::ConstPtr &graph) {
  // check if the required data is in the cache
  if (!qdata.rig_features.is_valid()) return;

  // check if the target vertex id is valid
  if (config_->landmark_source == "map") {
    if (!qdata.map_id || !qdata.map_id->isValid()) return;
  } else if (config_->landmark_source == "live") {
    if (!qdata.live_id || !qdata.live_id->isValid()) {
      LOG(DEBUG) << "Invalid live id, likely the first frame.";
      return;
    }
  }

  // Set up a new data structure for the map landmarks.
  auto &map_landmarks = *qdata.map_landmarks.fallback();

  // store the sensor/vehicle transform
  T_s_v_ = *qdata.T_sensor_vehicle;

  auto &query_features = *qdata.rig_features;
  vertex_landmarks_.clear();
  T_map_i_cache_.clear();
  T_map_i_s_cache_.clear();

  VertexId map_id;
  if (config_->landmark_source == "map")
    map_id = *qdata.map_id;
  else if (config_->landmark_source == "live")
    map_id = *qdata.live_id;

  for (uint32_t rig_idx = 0; rig_idx < query_features.size(); ++rig_idx) {
    auto &rig_name = query_features[rig_idx].name;

    // make sure we have loaded the sensor transform for this vertex
    loadSensorTransform(map_id, rig_name, graph);

    // Retrieve landmarks for the target frame.
    map_landmarks.emplace_back(recallLandmarks(rig_name, map_id, graph));
  }

  // assign the T_s_v_map to the mdata
  qdata.T_sensor_vehicle_map.clear().fallback(T_s_v_map_);
}

void LandmarkRecallModule::initializeLandmarkMemory(
    vtr::vision::ChannelLandmarks &channel_lm, const uint32_t &num_landmarks,
    const vtr_messages::msg::DescriptorType &desc_type) {
  // copy over the descriptor type.
  channel_lm.appearance.feat_type = messages::copyDescriptorType(desc_type);
  step_size_ = channel_lm.appearance.feat_type.bytes_per_desc;
  // Set up the CV::Mat accordingly
  // TODO: This should be moved somewhere common?
  decltype(sizeof(float)) byte_depth;
  decltype(CV_8UC1) cv_type;
  std::tie(cv_type, byte_depth) =
      messages::featureCvType(channel_lm.appearance.feat_type.impl);

  if (cv_type == CV_32F) {
    channel_lm.appearance.descriptors =
        cv::Mat(num_landmarks, channel_lm.appearance.feat_type.dims, CV_32F);
  } else if (cv_type == CV_8UC1) {
    channel_lm.appearance.descriptors =
        cv::Mat(num_landmarks, channel_lm.appearance.feat_type.dims, CV_8UC1);
  } else {
    throw std::invalid_argument("landmark recall: unknown descriptors\n");
#if false
    std::stringstream stacktrace;
    stacktrace << el::base::debug::StackTrace();
    throw std::invalid_argument("unknown descriptors\n" + stacktrace.str());
#endif
  }
  // Get a pointer to the beginning of the descriptor mat.
  map_desc_ptr_ = &channel_lm.appearance.descriptors.data[0];
}

void LandmarkRecallModule::recallLandmark(
    vision::ChannelLandmarks &channel_lm,
    const vision::LandmarkMatch &landmark_obs, const uint32_t &landmark_idx,
    const uint32_t &num_landmarks, const std::string &rig_name,
    const VertexId &map_id, const std::shared_ptr<const Graph> &graph) {
  // Get the index to the vertex the landmark was first seen in.
  // TODO: For multi experience, we need to find an index where the run is
  // loaded.
  const vision::LandmarkId &index = landmark_obs.to[0];

  // grab the landmarks from the given vertex
  auto vid =
      graph->fromPersistent(messages::copyPersistentId(index.persistent));
  auto landmark_vertex = graph->at(vid);

  if (vertex_landmarks_.find(vid) == vertex_landmarks_.end()) {
    vertex_landmarks_[vid] =
        landmark_vertex->retrieveKeyframeData<vtr_messages::msg::RigLandmarks>(
            rig_name + "_landmarks");
  }

  auto landmarks = vertex_landmarks_[landmark_vertex->id()];
  if (landmarks == nullptr) {
    LOG(ERROR) << "Could not recall landmarks from vertex: "
               << landmark_vertex->id();
    return;
  }

  // Get the landmarks for this channel.
  const auto &landmark_channel = landmarks->channels[index.channel];
  const auto &descriptor_string = landmark_channel.descriptors;
  if (step_size_ * (index.index + 1) > descriptor_string.size()) {
    LOG(ERROR) << "bad landmark descriptors "
               << "map: " << map_id << " "
               << "vertex: " << vid << " "
               << "lmid: " << index << " ";
    return;
  }

  // recall the T_s_v's for the two vertices
  loadSensorTransform(vid, rig_name, graph);
  loadSensorTransform(map_id, rig_name, graph);

  // If this is the first landmark observation, then do some setup.
  if (map_desc_ptr_ == nullptr) {
    initializeLandmarkMemory(channel_lm, num_landmarks,
                             landmark_channel.desc_type);
  }

  // Extract the single point corresponding to this landmark, squash it into
  // this coord. frame, and add it.
  const auto &point = landmark_channel.points[index.index];
  auto new_point = Eigen::Vector3d(point.x, point.y, point.z);
  channel_lm.points.col(landmark_idx) =
      squashPoint(new_point, map_id, landmark_vertex->id(), graph);

  // copy over the descriptor
  float *desc_ptr =
      (float *)&descriptor_string.data()[step_size_ * index.index];
  memcpy(map_desc_ptr_, desc_ptr, step_size_);
  map_desc_ptr_ += step_size_;

  // copy over the keypoint_info
  channel_lm.appearance.keypoints.push_back(vision::Keypoint());
  auto &kp = channel_lm.appearance.keypoints.back();
  kp.response = landmark_channel.lm_info[index.index].response;
  kp.octave = landmark_channel.lm_info[index.index].scale;
  kp.angle = landmark_channel.lm_info[index.index].orientation;

  channel_lm.appearance.feat_infos.push_back(vision::FeatureInfo());
  auto &feat_info = channel_lm.appearance.feat_infos.back();
  feat_info.laplacian_bit = landmark_channel.lm_info[index.index].laplacian_bit;

  // precision doesn't get filled out in the landmark
  // feat_info.precision = #

  if (landmark_channel.matches.size() <= index.index) {
    LOG(ERROR) << "Uh oh, " << messages::copyLandmarkId(index).idx
               << " is out of range.";
    return;
  }

  // copy over the matches
  // TODO FOR LOC ONLY TO AVOID SEGFAULT RIGHT NOW...
  if (config_->landmark_matches) {  // set in navigator.xml so block never
                                    // tested offline
    channel_lm.matches.push_back(vtr::vision::LandmarkMatch());
    auto &match = channel_lm.matches.back();
    auto &match_msg = landmark_channel.matches[index.index];
    match.from = messages::copyLandmarkId(match_msg.from_id);
    for (const auto &landmark_to : match_msg.to_id) {
      match.to.emplace_back(messages::copyLandmarkId(landmark_to));
    }
  }

  // get the validity   // size check for backwards compatibility
  if (index.index < landmark_channel.valid.size()) {
    channel_lm.valid[landmark_idx] = landmark_channel.valid[index.index];
  } else {
    channel_lm.valid[landmark_idx] = true;
  }
}

LandmarkFrame LandmarkRecallModule::recallLandmarks(
    const std::string &rig_name, const VertexId &map_id,
    const std::shared_ptr<const Graph> &graph) {
  LandmarkFrame landmark_frame;

  // The observations and landmarks we are currently dealing with.
  auto &map_obs = landmark_frame.observations;
  auto &map_lm = landmark_frame.landmarks;

  // update the names
  map_lm.name = rig_name;
  map_obs.name = rig_name;

  // grab the observations from the vertex.
  auto vertex = graph->at(map_id);

  // TODO: Add a try catch, in case the rig is not in the graph...
  auto observations =
      vertex->retrieveKeyframeData<vtr_messages::msg::RigObservations>(
          rig_name + "_observations", true);
  if (observations == nullptr) return landmark_frame;

  // simply move the observations over.
  map_obs = messages::copyObservation(*observations.get());

  // Go through each channel
  for (uint32_t channel_idx = 0; channel_idx < map_obs.channels.size();
       channel_idx++) {
    // Create a new set of landmarks for this channel.
    const auto &channel_obs = map_obs.channels[channel_idx];
    map_lm.channels.emplace_back(vision::ChannelLandmarks());
    map_lm.channels.back().name = observations->channels[channel_idx].name;
    // Make sure there are actually observations here
    if (channel_obs.cameras.size() > 0) {
      // Grab the observations from the first camera.
      const auto &obs = channel_obs.cameras[0];

      // Determine the number of landmarks this channel is observing.
      auto num_landmarks = obs.landmarks.size();

      // Resize our structure accordingly
      auto &channel_lm = map_lm.channels.back();
      channel_lm.points.resize(3, num_landmarks);
      channel_lm.covariances.resize(9, num_landmarks);
      channel_lm.valid.resize(num_landmarks, false);

      // iterate through the landmarks
      step_size_ = 0;
      map_desc_ptr_ = nullptr;

      for (uint32_t landmark_idx = 0; landmark_idx < num_landmarks;
           ++landmark_idx) {
        recallLandmark(channel_lm, obs.landmarks[landmark_idx], landmark_idx,
                       num_landmarks, rig_name, map_id, graph);
      }  // end for landmark_idx
    }    // end for cameras > 0
  }      // end for channel_idx
  return landmark_frame;
}

lgmath::se3::Transformation LandmarkRecallModule::cachedVehicleTransform(
    const VertexId &map_vid, const VertexId &landmark_vid,
    const Graph::ConstPtr &graph) {
  // identity if the vertex ids are the same
  lgmath::se3::Transformation T_map_lm;
  if (map_vid == landmark_vid) {
    return T_map_lm;
  }

  // if we have the result in the cache already, use that
  auto T_map_lm_find = T_map_i_cache_.find({map_vid, landmark_vid});
  if (T_map_lm_find != T_map_i_cache_.end()) {
    return T_map_lm_find->second;
  }

  // start with the earliest cached transform (smallest id)
  auto start_vid = map_vid;
  T_map_lm_find = T_map_i_cache_.begin();
  if (T_map_lm_find != T_map_i_cache_.end()) {
    start_vid = T_map_lm_find->first.second;
    T_map_lm = T_map_lm_find->second;
  }

  // only search on temporal (vo) edges
  TemporalEvaluator::Ptr tempeval(new TemporalEvaluator());
  tempeval->setGraph((void *)graph.get());
  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  using DirectionEvaluator =
      pose_graph::eval::Mask::DirectionFromVertexDirect<Graph>;
  auto direval = std::make_shared<DirectionEvaluator>(start_vid, true);
  direval->setGraph((void *)graph.get());
  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);
  evaluator->setGraph((void *)graph.get());

  // compound and store transforms until we hit the target landmark_vid
  auto itr = ++graph->begin(start_vid, 0, evaluator);
  for (; itr != graph->end(); ++itr) {
    auto i_vid = itr->to();
    T_map_lm *= itr->T();
    T_map_i_cache_[{map_vid, i_vid}] = T_map_lm;
    if (i_vid <= landmark_vid) break;
  }
  return T_map_lm;
}

lgmath::se3::Transformation LandmarkRecallModule::cachedSensorTransform(
    const VertexId &map_vid, const VertexId &landmark_vid,
    const Graph::ConstPtr &graph) {
  // identity if the vertex ids are the same
  lgmath::se3::Transformation T_map_lm_s;
  if (map_vid == landmark_vid) {
    return T_map_lm_s;
  }

  // if we have the result in the cache already, use that
  auto T_map_lm_s_find = T_map_i_s_cache_.find({map_vid, landmark_vid});
  if (T_map_lm_s_find != T_map_i_s_cache_.end()) {
    return T_map_lm_s_find->second;
  }

  // pre-generate the T_s_v transforms
  auto T_s_v_map_ptr = T_s_v_map_.find(map_vid);
  EdgeTransform T_s_v_map = T_s_v_;
  if (T_s_v_map_ptr != T_s_v_map_.end()) {
    T_s_v_map = T_s_v_map_ptr->second;
  } else {
    LOG(WARNING) << "Couldn't find T_s_v for the map vertex!";
  }
  auto T_s_v_lm_ptr = T_s_v_map_.find(landmark_vid);
  EdgeTransform T_s_v_lm = T_s_v_;
  if (T_s_v_lm_ptr != T_s_v_map_.end()) {
    T_s_v_lm = T_s_v_lm_ptr->second;
  } else {
    LOG(WARNING) << "Couldn't find T_s_v for the landmark vertex!";
  }
  // save the sensor-frame transform in the cache
  auto T_map_lm = cachedVehicleTransform(map_vid, landmark_vid, graph);
  T_map_lm_s = T_s_v_map * T_map_lm * T_s_v_lm.inverse();
  T_map_i_s_cache_[{map_vid, landmark_vid}] = T_map_lm_s;
  return T_map_lm_s;
}

Eigen::Vector3d LandmarkRecallModule::squashPoint(
    const Eigen::Vector3d &point, const VertexId &map_vid,
    const VertexId &landmark_vid, const Graph::ConstPtr &graph) {
  if (landmark_vid != map_vid) {
    auto T_map_lm_s = cachedSensorTransform(map_vid, landmark_vid, graph);
    return (T_map_lm_s * point.homogeneous()).hnormalized();
  } else {
    return point;
  }
}

void LandmarkRecallModule::loadSensorTransform(const VertexId &vid,
                                               const std::string &rig_name,
                                               const Graph::ConstPtr &graph) {
  // Check to see if the transform associated with this landmark is already
  // accounted for.
  if (T_s_v_map_.find(vid) != T_s_v_map_.end()) return;

  // If not, we should try and extract the T_s_v transform for this vertex.
  auto map_vertex = graph->at(vid);
  auto rc_transforms =
      map_vertex->retrieveKeyframeData<vtr_messages::msg::Transform>(
          rig_name + "_T_sensor_vehicle");

  Eigen::Matrix<double, 6, 1> tmp;
  auto &mt = rc_transforms->translation;
  auto &mr = rc_transforms->orientation;
  tmp << mt.x, mt.y, mt.z, mr.x, mr.y, mr.z;
  T_s_v_map_[vid] = lgmath::se3::TransformationWithCovariance(tmp);
}
}  // namespace tactic
}  // namespace vtr
