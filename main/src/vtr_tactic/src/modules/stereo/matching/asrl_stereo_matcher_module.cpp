#include <vtr_tactic/modules/stereo/matching/asrl_stereo_matcher_module.hpp>

namespace vtr {
namespace tactic {

void ASRLStereoMatcherModule::runImpl(QueryCache &qdata, MapCache &mdata,
                                      const Graph::ConstPtr &graph) {
  // if we dont have map and query landarks (i.e. first frame, then return)
  if (qdata.candidate_landmarks.is_valid() == false ||
      qdata.map_landmarks.is_valid() == false) {
    return;
  }
  // match features and record how many we found
  auto num_matches = matchFeatures(qdata, mdata, graph);
  // what if there were too few?
  if (num_matches < (unsigned)config_->min_matches) {
    LOG(WARNING) << "Rematching because we didn't meet minimum matches!";
    // run again, and use the forced loose pixel thresh
    num_matches = matchFeatures(qdata, mdata, graph);
  }
}

unsigned ASRLStereoMatcherModule::matchFeatures(
    QueryCache &qdata, MapCache &, const std::shared_ptr<const Graph> &) {
  // make sure the raw matches are empty (we may have used this function before)
  qdata.raw_matches.clear();
  // output matches
  auto &matches = *qdata.raw_matches.fallback();

  // grab the query landmarks.
  std::vector<vision::RigLandmarks> &query_landmarks =
      *qdata.candidate_landmarks;

  // grab the map landmarks
  std::vector<LandmarkFrame> &map_landmarks = *qdata.map_landmarks;

  // grab the features contained in the query frame.
  std::vector<vision::RigFeatures> &query_features = *qdata.rig_features;

  // get the intrinsics of the rig (assuming that this remains the same for all
  // cameras)
  vision::CameraIntrinsic &K = qdata.rig_calibrations->front().intrinsics.at(0);

  // predicted inverse transformation matrix
  Eigen::Matrix<double, 3, 4> Ti;

  use_tight_pixel_thresh_ =
      qdata.T_r_m_prior.is_valid() &&
      sqrt(qdata.T_r_m_prior->cov()(0, 0)) < config_->tight_matching_x_sigma &&
      sqrt(qdata.T_r_m_prior->cov()(1, 1)) < config_->tight_matching_y_sigma &&
      sqrt(qdata.T_r_m_prior->cov()(5, 5)) <
          config_->tight_matching_theta_sigma;

  // force the loose pixel thresh
  if (force_loose_pixel_thresh_) {
    use_tight_pixel_thresh_ = false;
  }

  // keep a record of how many matches we found
  int total_matches = 0;

  // if we are using an se3 prediction method
  if (config_->prediction_method == se3 && qdata.T_r_m_prior.is_valid()) {
    // get the candidate transform given by a different function and transform
    // it to the camera frame
    auto T_q_m = (*qdata.T_sensor_vehicle) * (*qdata.T_r_m_prior) *
                 ((*qdata.T_sensor_vehicle_map)[*qdata.live_id].inverse());

    // pre-cache the inverse transform matrix
    Ti = K * T_q_m.matrix().inverse().topLeftCorner(3, 4);
  }

  // go through each rig
  for (uint32_t rig_idx = 0; rig_idx < query_landmarks.size(); ++rig_idx) {
    // grab the data for this rig
    const vision::RigLandmarks &query_rig_lm = query_landmarks[rig_idx];
    const vision::RigFeatures &query_rig_feat = query_features[rig_idx];
    const vision::RigLandmarks &map_rig_lm = map_landmarks[rig_idx].landmarks;
    const vision::RigObservations &map_rig_obs =
        map_landmarks[rig_idx].observations;

    // put a new set of matches on for this rig.
    matches.emplace_back(vision::RigMatches());
    auto &rig_matches = matches.back();
    rig_matches.name = query_rig_lm.name;

    // go through each channel.
    for (uint32_t channel_idx = 0; channel_idx < query_rig_lm.channels.size();
         ++channel_idx) {
      // get the data for this channel.
      const vision::ChannelLandmarks &qry_channel_lm =
          query_rig_lm.channels[channel_idx];
      const vision::ChannelFeatures &qry_channel_feat =
          query_rig_feat.channels[channel_idx];
      const vision::ChannelLandmarks &map_channel_lm =
          map_rig_lm.channels[channel_idx];
      const vision::ChannelObservations &map_channel_obs =
          map_rig_obs.channels[channel_idx];

      // If there is actually data here, then match.
      if (qry_channel_lm.appearance.descriptors.rows > 0 &&
          map_channel_lm.appearance.descriptors.rows > 0) {
        // make a new matcher
        vision::ASRLFeatureMatcher::Config matcher_config;
        vision::ASRLFeatureMatcher matcher(matcher_config);

        // make a new ChannelMatches
        vision::ChannelMatches channel_matches;
        channel_matches.name = qry_channel_lm.name;

// multi-thread
#pragma omp parallel for num_threads(config_->parallel_threads)

        // for each landmark in the query
        for (uint32_t qry_lm_idx = 0;
             qry_lm_idx < qry_channel_lm.appearance.feat_infos.size();
             ++qry_lm_idx) {
          // Grab the corresponding query keypoint
          const vision::Keypoint &kp_query =
              qry_channel_feat.cameras[0].keypoints[qry_lm_idx];
          const vision::FeatureInfo &lm_info_qry =
              qry_channel_lm.appearance.feat_infos[qry_lm_idx];

          // make a new temporary 2D point to hold the transformed projection
          cv::Point qry_pt = kp_query.pt;

          // if we are using an se3 prediction method
          if (config_->prediction_method == se3 &&
              qdata.T_r_m_prior.is_valid()) {
            // Grab the corresponding query 3D point
            const auto &pt_query3 = qry_channel_lm.points.col(qry_lm_idx);

            // transform the homogenised point
            Eigen::Vector3d qry_mod_pt = Ti * pt_query3.homogeneous();

            // copy the new normalised pixel position
            qry_pt.x = qry_mod_pt.hnormalized()(0);
            qry_pt.y = qry_mod_pt.hnormalized()(1);
          }

          // reset the best descriptor distance
          float best_dist = std::numeric_limits<float>::max();
          unsigned match_idx = 0;

          // look at each landmark in the map
          for (uint32_t map_lm_idx = 0;
               map_lm_idx < map_channel_lm.appearance.feat_infos.size();
               ++map_lm_idx) {
            // Grab the corresponding map keypoint
            const vision::Point &map_pt =
                map_channel_obs.cameras[0].points[map_lm_idx];

            // Nope! Not a keypoint! Just the appearance info for the descriptor
            // (the point is invalid)
            const vision::Keypoint &kp_map =
                map_channel_lm.appearance.keypoints[map_lm_idx];

            // The additional appearance info
            const vision::FeatureInfo &lm_info_map =
                map_channel_lm.appearance.feat_infos[map_lm_idx];

            // check that all non-descriptor checks are OK before checking the
            // descriptor
            if (checkConditions(kp_map, lm_info_map, kp_query, lm_info_qry,
                                qry_pt, map_pt)) {
              // finally check the descriptor distance. 0.0 is perfect
              // match, 1.0 is maximally distant
              float match_dist = 1.0;

              if (qry_channel_lm.appearance.feat_type.impl ==
                  vision::FeatureImpl::OPENCV_ORB) {
                match_dist = matcher.briefmatch(
                    &qry_channel_lm.appearance.descriptors.at<unsigned char>(
                        qry_lm_idx, 0),
                    &map_channel_lm.appearance.descriptors.at<unsigned char>(
                        map_lm_idx, 0),
                    qry_channel_lm.appearance.feat_type.bytes_per_desc);
              } else if (qry_channel_lm.appearance.feat_type.impl ==
                         vision::FeatureImpl::ASRL_GPU_SURF) {
                match_dist = matcher.surfmatch(
                    &qry_channel_lm.appearance.descriptors.at<float>(qry_lm_idx,
                                                                     0),
                    &map_channel_lm.appearance.descriptors.at<float>(map_lm_idx,
                                                                     0),
                    qry_channel_lm.appearance.feat_type.bytes_per_desc /
                        sizeof(float));
              }

              // check if the descriptor distant meets the threshold and is
              // better than any other
              if (match_dist < config_->descriptor_thresh &&
                  match_dist < best_dist) {
                best_dist = match_dist;
                match_idx = map_lm_idx;
              }
            }
          }

          // did we find a good match that met all the criteria?
          if (best_dist < std::numeric_limits<float>::max()) {
            // add it to the channel matches
            vision::SimpleMatch match;
            match.first = match_idx;
            match.second = qry_lm_idx;
#pragma omp critical(updatematch)
            channel_matches.matches.push_back(match);
            ++total_matches;
          }
        }
        // we've gone through all the query landmarks, add the channel matches
        // to the rig
        rig_matches.channels.emplace_back(channel_matches);

      } else {
        // Just put empty matches on otherwise.
        rig_matches.channels.emplace_back(vision::ChannelMatches());
        rig_matches.channels.back().name = qry_channel_lm.name;
      }
    }
  }

  if ((int)total_matches < config_->min_matches) {
    force_loose_pixel_thresh_ = true;
  } else {
    force_loose_pixel_thresh_ = false;
  }

  return total_matches;
}

bool ASRLStereoMatcherModule::checkConditions(
    const vision::Keypoint &kp_map, const vision::FeatureInfo &lm_info_map,
    const vision::Keypoint &kp_query, const vision::FeatureInfo &lm_info_qry,
    const cv::Point &qry_pt, const cv::Point &map_pt) {
  // check that the octave of the two keypoints are roughly similar
  if (config_->check_laplacian_bit &&
      lm_info_qry.laplacian_bit != lm_info_map.laplacian_bit) {
    return false;
  }

  // check that the responses of the two keypoints are roughly similar
  if (config_->check_response) {
    float highest_response = std::max(kp_query.response, kp_map.response);
    float lowest_response = std::min(kp_query.response, kp_map.response);
    if (lowest_response / highest_response < config_->min_response_ratio) {
      return false;
    }
  }

  // check that the octave of the two keypoints are roughly similar
  if (config_->check_octave && kp_query.octave != kp_map.octave) {
    return false;
  }

  // generate a window from the keypoint precision if the configuration says so
  float window_scale = config_->use_pixel_variance
                           ? std::sqrt(1.0 / lm_info_qry.precision)
                           : 1.0;

  // scale it by the desired window size
  float window_size = window_scale * (use_tight_pixel_thresh_
                                          ? config_->tight_matching_pixel_thresh
                                          : config_->matching_pixel_thresh);

  // now check that the keypoints meet the minimum position error metrics (given
  // the transformed point and the window if the window size is 0 or below, that
  // indicates we don't care about it
  if (window_size > 0 && (std::abs(qry_pt.x - map_pt.x) > window_size ||
                          std::abs(qry_pt.y - map_pt.y) > window_size))
    return false;

  // if we got here, all checks passed
  return true;
}

void ASRLStereoMatcherModule::visualizeImpl(
    QueryCache &qdata, MapCache &mdata, const std::shared_ptr<const Graph> &,
    std::mutex &vis_mtx) {
  // check if visualization is enabled
  if (config_->visualize_feature_matches &&
      qdata.raw_matches.is_valid() == true)
    visualize::showMatches(vis_mtx, qdata, mdata, *qdata.raw_matches,
                           " raw matches", true);
}

}  // namespace tactic
}  // namespace vtr
