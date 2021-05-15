#pragma clang diagnostic push
#pragma ide diagnostic ignored "openmp-use-default-none"
#include <vtr_tactic/modules/stereo/matching/mel_matcher_module.hpp>
#include <vtr_tactic/visualize.hpp>

#include <vtr_pose_graph/path/pose_cache.hpp>
#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>
#include <vtr_vision/messages/bridge.hpp>

#include <opencv2/core/version.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/opencv_modules.hpp>

namespace vtr {
namespace tactic {

void MelMatcherModule::runImpl(QueryCache &qdata, MapCache &mdata,
                               const Graph::ConstPtr &graph) {
  // sanity check
  if (!qdata.T_sensor_vehicle.is_valid()) {
    LOG(ERROR) << "T_sensor_vehicle not present";
    return;
  } else if (!qdata.T_sensor_vehicle_map.is_valid()) {
    LOG(ERROR) << "T_sensor_vehicle_map not present";
    return;
  } else if (!qdata.map_landmarks.is_valid()) {
    LOG(ERROR) << "map_landmarks not present";
    return;
  } else if (!qdata.landmark_offset_map.is_valid()) {
    LOG(ERROR) << "landmark_offset_map not present";
    return;
  } else if (!qdata.projected_map_points.is_valid()) {
    LOG(ERROR) << "projected_map_points not present";
    return;
  } else if (!qdata.migrated_points_3d.is_valid()) {
    LOG(ERROR) << "migrated_points_3d not present";
    return;
  } else if (!qdata.migrated_validity.is_valid()) {
    LOG(ERROR) << "migrated_validity not present";
    return;
  };

  auto T_s_v_q = *qdata.T_sensor_vehicle;
  auto T_s_v_m = (*qdata.T_sensor_vehicle_map)[*qdata.map_id];
  auto T_q_m_prior = T_s_v_q * (*qdata.T_r_m_prior) * T_s_v_m.inverse();

  use_tight_pixel_thresh_ =
      sqrt(T_q_m_prior.cov()(0, 0)) <
          config_->tight_matching_x_sigma &&  // TODO config these
      sqrt(T_q_m_prior.cov()(1, 1)) < config_->tight_matching_y_sigma &&
      sqrt(T_q_m_prior.cov()(5, 5)) < config_->tight_matching_theta_sigma;

  // initialize the structure of the output matches.
  auto &query_landmarks = *qdata.map_landmarks;
  auto &matches = *qdata.raw_matches.fallback();
  initializeMatches(query_landmarks, matches);

  // reset the temporary variables.
  reset();

  matchAcrossExperiences(qdata, mdata, graph);
}

void MelMatcherModule::initializeMatches(
    const std::vector<LandmarkFrame> &query_landmarks,
    std::vector<vision::RigMatches> &matches) {
  // initialize the matches
  for (uint32_t rig_idx = 0; rig_idx < query_landmarks.size(); ++rig_idx) {
    const auto &query_rig_landmarks = query_landmarks[rig_idx].landmarks;
    matches.emplace_back(vision::RigMatches());
    auto &rig_matches = matches.back();
    rig_matches.name = query_rig_landmarks.name;
    for (uint32_t channel_idx = 0;
         channel_idx < query_rig_landmarks.channels.size(); ++channel_idx) {
      const auto &query_channel_lm = query_rig_landmarks.channels[channel_idx];
      rig_matches.channels.emplace_back(vision::ChannelMatches());
      rig_matches.channels.back().name = query_channel_lm.name;
    }
  }
}

void MelMatcherModule::reset() {
  // Iterate through each vertex in the sub map.
  query_matched_.clear();
  query_matched_.resize(10000, false);
  map_matched_.clear();
}

void MelMatcherModule::matchAcrossExperiences(
    QueryCache &qdata, MapCache &mdata,
    const std::shared_ptr<const Graph> &graph) {
  // Do a BFS on the localization map starting at the root id.
  auto &localization_map = *qdata.localization_map;
  auto map_itr = graph->beginBfs(*qdata.map_id);

  total_match_count_ = 0;
  unsigned visited = 0;
  timer_.reset();

  // Iterate through the localization map window.
  for (; visited < localization_map->numberOfVertices() &&
         map_itr != graph->end();
       ++map_itr) {
    if (!localization_map->contains(map_itr->v()->id())) continue;
    // check for breaking criteria
    if (total_match_count_ >= config_->target_match_count ||
        timer_.elapsedMs() > config_->time_allowance) {
      break;
    }
    matchVertex(qdata, mdata, map_itr->v());
    ++visited;
  }
}

void MelMatcherModule::matchVertex(QueryCache &qdata, MapCache &mdata,
                                   Vertex::Ptr vertex) {
  auto &query_landmarks = *qdata.map_landmarks;
  auto &rig_names = *qdata.rig_names;
  // iterate through each rig.
  for (uint32_t rig_idx = 0; rig_idx < query_landmarks.size(); ++rig_idx) {
    // get the query rig landmarks and observations
    const auto &query_rig_landmarks = query_landmarks[rig_idx].landmarks;
    const std::string &rig_name = rig_names[rig_idx];
    auto map_rig_landmarks =
        vertex->retrieveKeyframeData<vtr_messages::msg::RigLandmarks>(
            rig_name + "_landmarks");
    if (map_rig_landmarks == nullptr) {
      LOG(ERROR) << "landmarks at " << vertex->id() << " could not be loaded";
      return;
    }
    for (uint32_t channel_idx = 0;
         channel_idx < query_rig_landmarks.channels.size(); ++channel_idx) {
      const auto &map_channel_lm = map_rig_landmarks->channels[channel_idx];
      vision::LandmarkId channel_id;
      channel_id.persistent =
          messages::copyPersistentId(vertex->persistentId());
      channel_id.rig = rig_idx;
      channel_id.channel = channel_idx;
      if (config_->match_on_gpu) {
        matchChannelGPU(qdata, channel_id, map_channel_lm);
      } else {
        matchChannel(qdata, channel_id, map_channel_lm);
      }
    }
  }  // end for rig
}

void MelMatcherModule::matchChannel(
    QueryCache &qdata, const vision::LandmarkId &channel_id,
    const vtr_messages::msg::ChannelLandmarks &map_channel_lm) {
  const auto &query_rig_data = (*qdata.map_landmarks)[channel_id.rig];
  const auto &query_channel_obs =
      query_rig_data.observations.channels[channel_id.channel];
  auto &landmark_offset_map = *qdata.landmark_offset_map;

  // if there are no actual observations, return
  if (query_channel_obs.cameras.size() <= 0) {
    return;
  }

  // make a temporary simplematches
  vision::SimpleMatches channel_matches;

// multi-thread
#pragma omp parallel for num_threads(config_->parallel_threads)

  // iterate through every query keypoint
  for (uint32_t q_kp_idx = 0;
       q_kp_idx < query_channel_obs.cameras[0].points.size(); ++q_kp_idx) {
    // check if we have matched already with a vertex that is closer to our
    // estimate
    if (query_matched_[q_kp_idx] == true) {
      // if we have matched this already, then don't try matching again
      continue;
    } else {
      // try and match the query keypoint to the map
      int best_match;
      try {
        best_match =
            matchQueryKeypoint(qdata, channel_id, q_kp_idx, map_channel_lm);
      } catch (...) {
        continue;
      }  // since this is in openmp, any exceptions are deadly

      // check to see if we found a match
      if (best_match != -1) {
        // yes, we found one
        auto &point_map_offset = landmark_offset_map[channel_id];

        // set up a new SimpleMatch
        vision::SimpleMatch map_match;
        map_match.first = point_map_offset + best_match;
        map_match.second = q_kp_idx;

#pragma omp critical(updatematch)
        {
          // initialize an iterator to the end of the list
          vision::SimpleMatches::iterator mit = channel_matches.end();

          if (config_->screen_matched_landmarks) {
            // does a match to the map already exist in our temporary channel
            // matches?
            std::find_if(channel_matches.begin(), channel_matches.end(),
                         [&](vision::SimpleMatch m) {
                           return m.first == map_match.first;
                         });
          }

          // did we find a match that already exists?
          if (mit == channel_matches.end()) {
            // if a match doesn't exist, then add it
            total_match_count_++;
            query_matched_[q_kp_idx] = true;
            // get the track for the best matched landmark
            const vtr_messages::msg::Match &lm_track =
                map_channel_lm.matches[best_match];
            map_matched_[messages::copyLandmarkId(lm_track.from_id)] = true;
            channel_matches.emplace_back(map_match);
          }
        }
      }
    }
  }

  // add our temporary channel matches to the rest
  auto &output_channel_matches =
      (*qdata.raw_matches)[channel_id.rig].channels[channel_id.channel];
  output_channel_matches.matches.insert(output_channel_matches.matches.end(),
                                        channel_matches.begin(),
                                        channel_matches.end());
}

void MelMatcherModule::matchChannelGPU(
    QueryCache &qdata, const vision::LandmarkId &channel_id,
    const vtr_messages::msg::ChannelLandmarks &map_channel_lm) {
  const auto &query_rig_data = (*qdata.map_landmarks)[channel_id.rig];
  const auto &query_channel_obs =
      query_rig_data.observations.channels[channel_id.channel];
  const auto &query_channel_lm =
      query_rig_data.landmarks.channels[channel_id.channel];

  // Map Landmark information
  auto &landmark_offset_map = *qdata.landmark_offset_map;
  auto &point_map_offset = landmark_offset_map[channel_id];

  const auto &projected_map_points = *qdata.projected_map_points;
  const auto &migrated_points_3d = *qdata.migrated_points_3d;
  const auto &migrated_validity = *qdata.migrated_validity;

  // if there are no actual observations, return
  if (query_channel_obs.cameras.empty()) {
    return;
  }

  // make some temporary matches
  std::vector<std::vector<cv::DMatch> > knnmatches;

  // a container for the CPU descriptors to upload to the GPU
  cv::Mat map_cpu_descs;

  // a container for the descriptor matcher
  cv::Ptr<cv::cuda::DescriptorMatcher> cudabfmatcher;

  // make a GPU based matcher depending on the descriptor type
  if (query_channel_lm.appearance.feat_type.impl ==
      vision::FeatureImpl::OPENCV_ORB) {
    // create the matcher
    cudabfmatcher =
        cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);

    // convert the protobuf descriptors to a GPU mat. Getting the descriptor
    // data out of the protobuf is a bit difficult, so we need an
    // intermediary...
    const auto &step_size =
        query_channel_lm.appearance.feat_type.bytes_per_desc;
    auto &map_descriptor_string = map_channel_lm.descriptors;
    uint8_t *map_descriptor = (uint8_t *)&map_descriptor_string.data()[0];

    // make the descriptor matrix
    map_cpu_descs = cv::Mat(map_channel_lm.points.size(), step_size, CV_8UC1,
                            map_descriptor);

  } else if (query_channel_lm.appearance.feat_type.impl ==
             vision::FeatureImpl::ASRL_GPU_SURF) {
    // create the matcher
    cudabfmatcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);

    // convert the protobuf descriptors to a GPU mat. Getting the descriptor
    // data out of the protobuf is a bit difficult, so we need an
    // intermediary...
    const auto &step_size =
        query_channel_lm.appearance.feat_type.bytes_per_desc / sizeof(float);
    auto &map_descriptor_string = map_channel_lm.descriptors;
    auto *map_descriptor = (float *)&map_descriptor_string.data()[0];

    // make the descriptor matrix
    map_cpu_descs = cv::Mat(map_channel_lm.points.size(), step_size, CV_32FC1,
                            map_descriptor);
  }

  // make some gpu based decriptors
  cv::cuda::GpuMat map_descs;
  cv::cuda::GpuMat qry_descs;

  // lock the mutex
  {
    std::lock_guard<std::mutex> lock(vision::__gpu_mutex__);

    // now upload the map descriptors
    map_descs.upload(map_cpu_descs);

    // upload the query descriptors to the GPU
    if (query_channel_lm.appearance.gpu_descriptors.empty()) {
      qry_descs.upload(query_channel_lm.appearance.descriptors);
    } else {
      // careful! this calls std::swap() on the memory, so the
      // frame.gpu_descriptors are now unallocated
      qry_descs = query_channel_lm.appearance.gpu_descriptors;
    }

    // do a brute force match on the GPU
    cudabfmatcher->knnMatch(map_descs, qry_descs, knnmatches,
                            config_->match_gpu_knn_match_num);
  }

  // make a temporary simplematches
  vision::SimpleMatches channel_matches;

// multi-thread
#pragma omp parallel for num_threads(config_->parallel_threads)

  for (unsigned qry_idx = 0; qry_idx < knnmatches.size(); ++qry_idx) {
    // sanity check
    if (knnmatches[qry_idx].empty()) {
      continue;
    }

    // get the query point index
    auto q_kp_idx = knnmatches[qry_idx][0].trainIdx;

    if (query_matched_[q_kp_idx]) {
      // if we have matched this already (or nothing matched), then don't try
      // matching again
      continue;
    } else {
      auto &query_kp_info = query_channel_lm.appearance.keypoints[q_kp_idx];
      auto &query_kp = query_channel_obs.cameras[0].points[q_kp_idx];

      // get the 3d point
      auto &query_point = query_channel_lm.points.col(q_kp_idx);

      // we don't yet have a best match
      int best_match = -1;

      // check depth
      if (query_point(2) > config_->max_landmark_depth) {
        continue;
      }

      // initalize the score
      double best_score = config_->descriptor_thresh_gpu;

      // for each landmark in the map (for this channel), see if there are any
      // that match this query keypoint
      for (unsigned m_kp_idx = 0; m_kp_idx < knnmatches[qry_idx].size();
           ++m_kp_idx) {
        auto lm_idx = knnmatches[qry_idx][m_kp_idx].queryIdx;
        const auto &lm_info_map = map_channel_lm.lm_info[lm_idx];

        // sanity check
        if (point_map_offset + lm_idx >= projected_map_points.cols() ||
            point_map_offset + lm_idx >= migrated_points_3d.cols()) {
          continue;
        }

        const Eigen::Vector2d &map_kp =
            projected_map_points.col(point_map_offset + lm_idx);
        const Eigen::Vector3d &map_point =
            migrated_points_3d.col(point_map_offset + lm_idx);

        // make the score compatible with our standard by subtracting
        // from 1.0. 1.0 is worst match, 0.0 is perfect
        float &distance = knnmatches[qry_idx][m_kp_idx].distance;
        double score = distance;
        if (query_channel_lm.appearance.feat_type.impl ==
            vision::FeatureImpl::OPENCV_ORB) {
          score =
              1.0 -
              (query_channel_lm.appearance.feat_type.bytes_per_desc * 8.0 -
               distance) /
                  (query_channel_lm.appearance.feat_type.bytes_per_desc * 8.0);
        } else if (query_channel_lm.appearance.feat_type.impl ==
                   vision::FeatureImpl::ASRL_GPU_SURF) {
          // do nothing
        }

        // check if we have the best score so far
        if (score <= best_score) {
          // if this landmark is not valid, then don't bother matching
          if ((unsigned)(point_map_offset + lm_idx) >=
                  migrated_validity.size() ||
              migrated_validity.at(point_map_offset + lm_idx) == false) {
            continue;
          }

          // get the track length
          const auto &vo_obs = map_channel_lm.num_vo_observations;
          int track_length = std::numeric_limits<int>::max();
          if (!vo_obs.empty()) {
            track_length = vo_obs[lm_idx];
          }

          // check all the conditions like track length, pixel distance etc.
          // before descriptor matching
          if (potential_match(query_kp_info, lm_info_map, track_length,
                              query_kp, map_kp, query_point(2), map_point(2),
                              map_channel_lm.matches[lm_idx])) {
            // yes! update it
            best_score = score;
            best_match = lm_idx;
            // we don't need to keep searching, this is the best score
            break;
          }
        } else {
          // if our score is above the threshold, we may as well quit now (the
          // descriptors are ordered by distance)
          break;
        }
      }
      // check to see if we found a match
      if (best_match != -1) {
        // yes, we found one
        auto &point_map_offset = landmark_offset_map[channel_id];

        // set up a new SimpleMatch
        vision::SimpleMatch map_match;
        map_match.first = point_map_offset + best_match;
        map_match.second = q_kp_idx;

#pragma omp critical(updatematch)
        {
          // initialize an iterator to the end of the list
          vision::SimpleMatches::iterator mit = channel_matches.end();

          if (config_->screen_matched_landmarks) {
            // does a match to the map already exist in our temporary channel
            // matches?
            std::find_if(channel_matches.begin(), channel_matches.end(),
                         [&](vision::SimpleMatch m) {
                           return m.first == map_match.first;
                         });
          }

          // did we find a match that already exists?
          if (mit == channel_matches.end()) {
            // if a match doesn't exist, then add it
            total_match_count_++;
            query_matched_[q_kp_idx] = true;
            // get the track for the best matched landmark
            const vtr_messages::msg::Match &lm_track =
                map_channel_lm.matches[best_match];
            map_matched_[messages::copyLandmarkId(lm_track.from_id)] = true;
            channel_matches.emplace_back(map_match);
          }
        }
      }
    }
  }

  // add our temporary channel matches to the rest
  auto &output_channel_matches =
      (*qdata.raw_matches)[channel_id.rig].channels[channel_id.channel];
  output_channel_matches.matches.insert(output_channel_matches.matches.end(),
                                        channel_matches.begin(),
                                        channel_matches.end());
}

int MelMatcherModule::matchQueryKeypoint(
    QueryCache &qdata, const vision::LandmarkId &channel_id, const int &q_kp_idx,
    const vtr_messages::msg::ChannelLandmarks &map_channel_lm) {
  // Query Landmark information
  const auto &query_rig_data = (*qdata.map_landmarks)[channel_id.rig];
  const auto &query_channel_lm =
      query_rig_data.landmarks.channels[channel_id.channel];
  auto &query_kp_info = query_channel_lm.appearance.keypoints[q_kp_idx];
  const auto &query_channel_obs =
      query_rig_data.observations.channels[channel_id.channel];
  const auto &step_size = query_channel_lm.appearance.feat_type.bytes_per_desc;
  auto &query_kp = query_channel_obs.cameras[0].points[q_kp_idx];
  auto &query_point = query_channel_lm.points.col(q_kp_idx);
  auto *query_descriptor = (float *)&query_channel_lm.appearance.descriptors
                               .data[step_size * q_kp_idx];

  // Map Landmark information
  auto &landmark_offset_map = *qdata.landmark_offset_map;
  auto &point_map_offset = landmark_offset_map[channel_id];
  const auto &projected_map_points = *qdata.projected_map_points;
  const auto &migrated_points_3d = *qdata.migrated_points_3d;
  const auto &migrated_validity = *qdata.migrated_validity;
  auto &map_descriptor_string = map_channel_lm.descriptors;

  // we don't yet have a best match
  int best_match = -1;

  // check depth
  if (query_point(2) > config_->max_landmark_depth) {
    return best_match;
  }

  // initalize the score
  double best_score = config_->descriptor_thresh_cpu;

  // for each landmark in the map (for this channel), see if there are any that
  // match this query keypoint
  for (unsigned lm_idx = 0; lm_idx < map_channel_lm.points.size(); ++lm_idx) {
    const auto &lm_info_map = map_channel_lm.lm_info[lm_idx];

    // sanity check
    if (point_map_offset + lm_idx >= projected_map_points.cols() ||
        point_map_offset + lm_idx >= migrated_points_3d.cols()) {
      continue;
    }

    const Eigen::Vector2d &map_kp =
        projected_map_points.col(point_map_offset + lm_idx);
    const Eigen::Vector3d &map_point =
        migrated_points_3d.col(point_map_offset + lm_idx);

    // if this landmark is not valid, then don't bother matching
    if ((unsigned)(point_map_offset + lm_idx) >= migrated_validity.size() ||
        migrated_validity.at(point_map_offset + lm_idx) == false) {
      continue;
    }

    // get the track length
    const auto &vo_obs = map_channel_lm.num_vo_observations;
    int track_length = std::numeric_limits<int>::max();
    if (vo_obs.size() > 0) {
      track_length = vo_obs[lm_idx];
    }

    // check all the conditions like track length, pixel distance etc. before
    // descriptor matching
    if (potential_match(query_kp_info, lm_info_map, track_length, query_kp,
                        map_kp, query_point(2), map_point(2),
                        map_channel_lm.matches[lm_idx])) {
      // if that passes, check descriptor distance
      auto *map_descriptor =
          (float *)&map_descriptor_string.data()[step_size * lm_idx];
      double score = vision::ASRLFeatureMatcher::distance(
          query_descriptor, map_descriptor,
          query_channel_lm.appearance.feat_type);

      // check if we have the best score so far
      if (score <= best_score) {
        // yes! update it
        best_score = score;
        best_match = lm_idx;
      }
    }
  }

  return best_match;
}

inline bool MelMatcherModule::potential_match(
    const cv::KeyPoint &query_lm_info,
    const vtr_messages::msg::FeatureInfo &lm_info_map,
    const int &map_track_length, const cv::Point &query_kp,
    const Eigen::Vector2d &map_kp, const double &query_depth,
    const double &map_depth, const vtr_messages::msg::Match &lm_track) {
  // 1. check track length
  if (map_track_length < config_->min_track_length) {
    return false;
  }

  // 2. check max dpeth
  if (map_depth > config_->max_landmark_depth ||
      query_depth > config_->max_landmark_depth) {
    return false;
  }

  // 3. check depth similarity
  double depth_diff = std::fabs(map_depth - query_depth);
  if (depth_diff > config_->max_depth_diff) {
    return false;
  }

  // 4. check octave
  if (query_lm_info.octave != lm_info_map.scale) {
    return false;
  }

  // 5. check response
  float highest_response =
      std::max(query_lm_info.response, lm_info_map.response);
  float lowest_response =
      std::max(query_lm_info.response, lm_info_map.response);
  if (lowest_response / highest_response < config_->min_response_ratio) {
    return false;
  }

  int matching_pixel_thresh = use_tight_pixel_thresh_
                                  ? config_->tight_matching_pixel_thresh
                                  : config_->matching_pixel_thresh;

  // 6. check pixel distance
  float pixel_distance_x = std::fabs(query_kp.x - map_kp(0));
  if (pixel_distance_x >= matching_pixel_thresh) {
    return false;
  }

  float pixel_distance_y = std::fabs(query_kp.y - map_kp(1));
  if (pixel_distance_y >= matching_pixel_thresh) {
    return false;
  }

  // TODO: Check uvd or uvu?

  // 7. screen landmarks that are already matched.
  if (config_->screen_matched_landmarks) {
    // check to see if we've mapped here before...
    bool previously_matched = false;
    for (const auto &track_idx : lm_track.to_id) {
      if (map_matched_.find(messages::copyLandmarkId(track_idx)) !=
          map_matched_.end()) {
        previously_matched = true;
        continue;
      }
    }

    if (previously_matched) {
      return false;
    }
  }

  return true;
}

}  // namespace tactic
}  // namespace vtr

#pragma clang diagnostic pop