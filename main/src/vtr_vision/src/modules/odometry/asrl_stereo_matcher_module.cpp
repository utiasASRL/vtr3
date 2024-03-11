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
 * \file asrl_stereo_matcher_module.cpp
 * \brief ASRLStereoMatcherModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/modules/odometry/asrl_stereo_matcher_module.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

auto ASRLStereoMatcherModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) 
    -> ConstPtr {
  ASRLStereoMatcherModule::Config::Ptr config = std::make_shared<ASRLStereoMatcherModule::Config>();
  // clang-format off
  config->check_laplacian_bit = node->declare_parameter<bool>(param_prefix + ".check_laplacian_bit", config->check_laplacian_bit);
  config->check_octave = node->declare_parameter<bool>(param_prefix + ".check_octave", config->check_octave);
  config->check_response = node->declare_parameter<bool>(param_prefix + ".check_response", config->check_response);
  config->min_response_ratio = node->declare_parameter<double>(param_prefix + ".min_response_ratio", config->min_response_ratio);
  config->min_matches = (unsigned) node->declare_parameter<int>(param_prefix + ".min_matches", config->min_matches);
  config->matching_pixel_thresh = node->declare_parameter<int>(param_prefix + ".matching_pixel_thresh", config->matching_pixel_thresh);
  config->tight_matching_pixel_thresh = node->declare_parameter<int>(param_prefix + ".tight_matching_pixel_thresh", config->tight_matching_pixel_thresh);
  config->tight_matching_x_sigma = node->declare_parameter<double>(param_prefix + ".tight_matching_x_sigma", config->tight_matching_x_sigma);
  config->tight_matching_y_sigma = node->declare_parameter<double>(param_prefix + ".tight_matching_y_sigma", config->tight_matching_y_sigma);
  config->tight_matching_theta_sigma = node->declare_parameter<double>(param_prefix + ".tight_matching_theta_sigma", config->tight_matching_theta_sigma);
  config->use_pixel_variance = node->declare_parameter<bool>(param_prefix + ".use_pixel_variance", config->use_pixel_variance);

  auto prediction_method = node->declare_parameter<std::string>(param_prefix + ".prediction_method", "se3");
  if (!prediction_method.compare("se3"))
    config->prediction_method = PredictionMethod::se3;
  else if (!prediction_method.compare("none"))
    config->prediction_method = PredictionMethod::none;
  else
    config->prediction_method = PredictionMethod::none;

  config->max_point_depth = node->declare_parameter<double>(param_prefix + ".max_point_depth", config->max_point_depth);
  config->descriptor_thresh = node->declare_parameter<double>(param_prefix + ".descriptor_thresh", config->descriptor_thresh);
  config->parallel_threads = node->declare_parameter<int>(param_prefix + ".parallel_threads", config->parallel_threads);
#ifdef VTR_DETERMINISTIC
  LOG_IF(config->parallel_threads != 1, WARNING) << "ASRL stereo matcher number of threads set to 1 in deterministic mode.";
  config->parallel_threads = 1;
#endif
  config->visualize_feature_matches = node->declare_parameter<bool>(param_prefix + ".visualize_feature_matches", config->visualize_feature_matches);
  // clang-format on

  return config;
}

void ASRLStereoMatcherModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);
  CLOG(DEBUG, "stereo.matcher") << "inside ASRL Stereo Matcher.";
  // if we dont have map and query landarks (i.e. first frame, then return)
  if (qdata.candidate_landmarks.valid() == false ||
      qdata.map_landmarks.valid() == false) {
    CLOG(DEBUG, "stereo.matcher") << "No valid landmarks, likely the first frame.";
    return;
  }
  CLOG(DEBUG, "stereo.matcher") << "After Accessing candidate landmarks.";

  // match features and record how many we found
  auto num_matches = matchFeatures(qdata, graph, false);
  // what if there were too few?
  if (num_matches < config_->min_matches) {
    CLOG(WARNING, "stereo.matcher") << "Rematching because we didn't meet minimum matches!";
    // run again, and use the forced loose pixel thresh
    num_matches = matchFeatures(qdata, graph, true);
  }
  CLOG(DEBUG, "stereo.matcher") << "After Running Match Features";

  if (config_->visualize_feature_matches &&
      qdata.raw_matches.valid())
    visualize::showMatches(*qdata.vis_mutex, qdata, *qdata.raw_matches,
                           " raw matches", true);
}

unsigned ASRLStereoMatcherModule::matchFeatures(CameraQueryCache &qdata,
                                                const Graph::ConstPtr &, bool force_loose_pixel_thresh) {
  // make sure the raw matches are empty (we may have used this function before)
  qdata.raw_matches.clear();
  // output matches
  auto &matches = *qdata.raw_matches.emplace();

  // grab the query landmarks.
  std::vector<vision::RigLandmarks> &query_landmarks = *qdata.candidate_landmarks;

  // grab the map landmarks
  std::vector<LandmarkFrame> &map_landmarks = *qdata.map_landmarks;

  // grab the features contained in the query frame.
  std::vector<vision::RigFeatures> &query_features = *qdata.rig_features;

  // get the intrinsics of the rig (assuming that this remains the same for all
  // cameras)
  vision::CameraIntrinsic &K = qdata.rig_calibrations->front().intrinsics.at(0);

  // predicted inverse transformation matrix
  Eigen::Matrix<double, 3, 4> Ti;

  bool use_tight_pixel_thresh =
      qdata.T_r_m_prior.valid() &&
      sqrt(qdata.T_r_m_prior->cov()(0, 0)) < config_->tight_matching_x_sigma &&
      sqrt(qdata.T_r_m_prior->cov()(1, 1)) < config_->tight_matching_y_sigma &&
      sqrt(qdata.T_r_m_prior->cov()(5, 5)) < config_->tight_matching_theta_sigma;

  // force the loose pixel thresh
  if (force_loose_pixel_thresh) {
    use_tight_pixel_thresh = false;
  }

  // keep a record of how many matches we found
  int total_matches = 0;

  // if we are using an se3 prediction method
  if (config_->prediction_method == PredictionMethod::se3 &&
      qdata.T_r_m_prior.valid()) {

    CLOG(DEBUG, "stereo.matcher") << "Using SE3 prediction";
    // get the candidate transform given by a different function and transform
    // it to the camera frame
    auto T_q_m = (*qdata.T_s_r) * (*qdata.T_r_m_prior) *
                 ((*qdata.T_sensor_vehicle_map)[*qdata.vid_odo].inverse());

    // pre-cache the inverse transform matrix
    Ti = K * T_q_m.matrix().inverse().topLeftCorner(3, 4);
  }

  // go through each rig
  for (uint32_t rig_idx = 0; rig_idx < query_landmarks.size(); ++rig_idx) {
    // grab the data for this rig
    const vision::RigLandmarks &query_rig_lm = query_landmarks[rig_idx];
    const vision::RigFeatures &query_rig_feat = query_features[rig_idx];
    const vision::RigLandmarks &map_rig_lm = map_landmarks[rig_idx].landmarks;
    const vision::RigObservations &map_rig_obs = map_landmarks[rig_idx].observations;

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
      //
      if ((qry_channel_lm.name != "RGB") &&
          (qry_channel_lm.appearance.descriptors.rows > 0) &&
          (map_channel_lm.appearance.descriptors.rows > 0)) {

        CLOG(DEBUG, "stereo.matcher") << "Using channel " << qry_channel_lm.name << " for matching";
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
          if (config_->prediction_method == PredictionMethod::se3 &&
              qdata.T_r_m_prior.valid()) {
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
                                qry_pt, map_pt, use_tight_pixel_thresh)) {
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
              } else if ((qry_channel_lm.appearance.feat_type.impl ==
                         vision::FeatureImpl::ASRL_GPU_SURF) ||
                         qry_channel_lm.appearance.feat_type.impl ==
                         vision::FeatureImpl::LEARNED_FEATURE) {
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
        CLOG(WARNING, "stereo.matcher") << "Adding empty matches";
        // Just put empty matches on otherwise.
        rig_matches.channels.emplace_back(vision::ChannelMatches());
        rig_matches.channels.back().name = qry_channel_lm.name;
      }
    }
  }

  CLOG(DEBUG, "stereo.matcher") << "Total matches: " << total_matches;
  return total_matches;
}

bool ASRLStereoMatcherModule::checkConditions(
    const vision::Keypoint &kp_map, const vision::FeatureInfo &lm_info_map,
    const vision::Keypoint &kp_query, const vision::FeatureInfo &lm_info_qry,
    const cv::Point &qry_pt, const cv::Point &map_pt,
    const bool use_tight_pixel_thresh) {
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
  float window_size = window_scale * (use_tight_pixel_thresh
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


}  // namespace vision
}  // namespace vtr