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
 * \file ransac_module.cpp
 * \brief RansacModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/modules/ransac/ransac_module.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

// void RansacModule::configFromROS(const rclcpp::Node::SharedPtr &node,
//                                  const std::string param_prefix) {
auto RansacModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) 
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->enable = node->declare_parameter<bool>(param_prefix + ".enable", config->enable);
  config->is_odometry = node->declare_parameter<bool>(param_prefix + ".is_odometry", config->is_odometry);

  config->iterations = node->declare_parameter<int>(param_prefix + ".iterations", config->iterations);
  config->flavor = node->declare_parameter<std::string>(param_prefix + ".flavor", config->flavor);
  config->sigma = node->declare_parameter<double>(param_prefix + ".sigma", config->sigma);
  config->threshold = node->declare_parameter<double>(param_prefix + ".threshold", config->threshold);
  config->early_stop_ratio = node->declare_parameter<double>(param_prefix + ".early_stop_ratio", config->early_stop_ratio);
  config->early_stop_min_inliers = node->declare_parameter<int>(param_prefix + ".early_stop_min_inliers", config->early_stop_min_inliers);
  config->visualize_ransac_inliers = node->declare_parameter<bool>(param_prefix + ".visualize_ransac_inliers", config->visualize_ransac_inliers);
  config->use_migrated_points = node->declare_parameter<bool>(param_prefix + ".use_migrated_points", config->use_migrated_points);
  config->min_inliers = node->declare_parameter<int>(param_prefix + ".min_inliers", config->min_inliers);
  config->enable_local_opt = node->declare_parameter<bool>(param_prefix + ".enable_local_opt", config->enable_local_opt);
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
#ifdef VTR_DETERMINISTIC
  LOG_IF(config->num_threads!=1, WARNING) << "RANSAC number of threads set to 1 in deterministic mode.";
  config->num_threads = 1;
#endif
  // clang-format on
  if (config->num_threads < 1) config->num_threads = 1;

  return config;

}

void RansacModule::flattenMatches(const vision::RigMatches &src_matches,
                                  vision::SimpleMatches &dst_matches) {
  for (uint32_t channel_idx = 0; channel_idx < src_matches.channels.size();
       ++channel_idx) {
    auto &channel_matches = src_matches.channels[channel_idx];
    auto &map_indices = map_channel_offsets_[channel_idx];
    auto &query_indices = query_channel_offsets_[channel_idx];
    for (auto &match : channel_matches.matches) {
      vision::SimpleMatch flattened_match(match.first + map_indices.first,
                                          match.second + query_indices.first);
      dst_matches.emplace_back(flattened_match);
    }
  }
}

void RansacModule::mirrorStructure(const vision::RigMatches &src_matches,
                                   vision::RigMatches &dst_matches) {
  dst_matches.name = src_matches.name;
  for (uint32_t channel_idx = 0; channel_idx < src_matches.channels.size();
       ++channel_idx) {
    auto &channel_matches = src_matches.channels[channel_idx];
    dst_matches.channels.push_back(vision::ChannelMatches());
    dst_matches.channels[channel_idx].name = channel_matches.name;
  }
}

void RansacModule::inflateMatches(const vision::SimpleMatches &src_matches,
                                  vision::RigMatches &dst_matches) {
  int num_channels = map_channel_offsets_.size();
  for (auto &inlier : src_matches) {
    // 1. determine the channel
    int channel_idx = 0;
    for (; channel_idx < num_channels; ++channel_idx) {
      // 1. this is not our channel
      if (inlier.first < map_channel_offsets_[channel_idx].second &&
          inlier.second < query_channel_offsets_[channel_idx].second) {
        auto &inflated_channel_matches =
            dst_matches.channels[channel_idx].matches;
        auto &map_indices = map_channel_offsets_[channel_idx];
        auto &query_indices = query_channel_offsets_[channel_idx];
        vision::SimpleMatch inflated_match(inlier.first - map_indices.first,
                                           inlier.second - query_indices.first);
        inflated_channel_matches.emplace_back(inflated_match);
        break;
      }
    }
  }
}

void RansacModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) {

  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  // if the map is not yet initialized, don't do anything
  if (/* *qdata.map_status == MAP_NEW || */
      qdata.raw_matches.valid() == false) {
    CLOG(INFO, "stereo.ransac") << "No valid matches, likely the first frame.";
    return;
  }

  bool &success = (config_->is_odometry)? *qdata.odo_success : *qdata.loc_success;


  CLOG(DEBUG, "stereo.ransac") << "Now processing second frame!";

  // make sure the offsets are not holding any old info
  map_channel_offsets_.clear();
  query_channel_offsets_.clear();

  CLOG(DEBUG, "stereo.ransac") << "Generating RANSAC sampler!";

  // Set up the ransac implementation
  auto sampler = generateRANSACSampler(qdata);



  CLOG(DEBUG, "stereo.ransac") << "Generating Filter Matches!";

  // filter the raw matches as necessary
  auto filtered_matches = generateFilteredMatches(qdata);

  // \todo (Old) we eventually need multi-rig support.
  int rig_idx = 0;

  // the rig matches that will be used for ransac
  auto &rig_matches = filtered_matches[rig_idx];


  CLOG(DEBUG, "stereo.ransac") << "Setting up Vanila RANSAC!";

  // \todo (Old) Set up config.
  vision::VanillaRansac<Eigen::Matrix4d> ransac(
      sampler, config_->sigma, config_->threshold, config_->iterations,
      config_->early_stop_ratio, config_->early_stop_min_inliers,
      config_->enable_local_opt, config_->num_threads);

  CLOG(DEBUG, "stereo.ransac") << "Generate RANSAC model!";

  // Problem specific
  auto ransac_model = generateRANSACModel(qdata);

  // If a model wasn't successfully generated, clean up and return error
  if (ransac_model == nullptr) {
    vision::SimpleMatches inliers;
    auto &matches = *qdata.ransac_matches.clear().emplace();
    matches.push_back(vision::RigMatches());
    CLOG(ERROR, "stereo.ransac") << "Model Has Failed!!!" << std::endl;
    success = false;
    return;
  }

  ransac.setCallback(ransac_model);

  // flatten the rig matches to a vector of matches for ransac.
  vision::SimpleMatches flattened_matches;
  flattenMatches(rig_matches, flattened_matches);

  Eigen::Matrix4d solution;
  vision::SimpleMatches inliers;

  if (flattened_matches.size() < (unsigned)config_->min_inliers) {
    LOG(ERROR) << "Insufficient number of raw matches: "
               << flattened_matches.size();
  } else {
    // Run RANSAC
    // \todo (Old) For now we are only using matches from the grayscale
    // solution. Alter the RANSAC code to accept vectors of matches / points.
    if (ransac.run(flattened_matches, &solution, &inliers) == 0) {
      success = false;
    } else {
      // Success, set the output (in the vehicle frame)

      VertexId map_id;
      VertexId live_id;
      //if (config_->landmark_source == "map")
      //map_id = *qdata.vid_loc;
      //else if (config_->landmark_source == "live")
      live_id = *qdata.vid_odo;


      auto T_s_v_q = *qdata.T_s_r;
      // auto T_s_v_m = config_->use_migrated_points
      //                    ? (*qdata.T_sensor_vehicle_map)[*qdata.map_id]
      //                    : (*qdata.T_sensor_vehicle_map)[*qdata.live_id];

      auto T_s_v_m = config_->use_migrated_points
                         ? (*qdata.T_sensor_vehicle_map)[map_id]
                         : (*qdata.T_sensor_vehicle_map)[live_id];


      *qdata.T_r_m =
          T_s_v_q.inverse() * lgmath::se3::Transformation(solution) * T_s_v_m;
      qdata.T_r_m->setZeroCovariance();
      success = true;
    }
  }

  if (inliers.size() <= static_cast<uint32_t>(config_->min_inliers)) {
    LOG(ERROR) << "RansacModule::" << __func__ << "(): " << inliers.size()
               << "/" << flattened_matches.size() << " is not enough inliers! ";
    inliers.clear();
    success = false;
  }

  // Inflate matches
  auto &matches = *qdata.ransac_matches.clear().emplace();
  matches.push_back(vision::RigMatches());
  mirrorStructure(rig_matches, matches[rig_idx]);
  inflateMatches(inliers, matches[rig_idx]);


  // check if visualization is enabled
  if (config_->visualize_ransac_inliers) {
    if (config_->use_migrated_points){
      //visualize::showMelMatches(*qdata.vis_mutex, qdata, graph,
      //                          "multi-exp-loc");
    }  else if (qdata.ransac_matches.valid())
      visualize::showMatches(*qdata.vis_mutex, qdata, *qdata.ransac_matches,
                             " RANSAC matches");
  }

}

std::vector<vision::RigMatches> RansacModule::generateFilteredMatches(
    CameraQueryCache &qdata) {
  return *qdata.raw_matches;
}


}  // namespace vision
}  // namespace vtr
