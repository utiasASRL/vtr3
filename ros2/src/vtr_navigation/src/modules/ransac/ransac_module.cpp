#include <vtr_navigation/modules/ransac/ransac_module.hpp>
#include <vtr_navigation/visualize.hpp>
#include <vtr_vision/outliers.hpp>

namespace vtr {
namespace navigation {
#if false
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
#endif
void RansacModule::run(QueryCache &qdata, MapCache &mdata,
                       const std::shared_ptr<const Graph> &) {
#if false
  // if the map is not yet initialized, don't do anything
  if (*mdata.map_status == MAP_NEW || mdata.raw_matches.is_valid() == false)
    return;

  // make sure the offsets are not holding any old info
  map_channel_offsets_.clear();
  query_channel_offsets_.clear();

  // Set up the ransac implementation
  auto sampler = generateRANSACSampler(qdata, mdata);

  // filter the raw matches as necessary
  auto filtered_matches = generateFilteredMatches(qdata, mdata);

  // \todo (Old) we eventually need multi-rig support.
  int rig_idx = 0;

  // the rig matches that will be used for ransac
  auto &rig_matches = filtered_matches[rig_idx];

  // \todo (Old) Set up config.
  vision::VanillaRansac<Eigen::Matrix4d> ransac(
      sampler, config_->sigma, config_->threshold, config_->iterations,
      config_->early_stop_ratio, config_->early_stop_min_inliers,
      config_->enable_local_opt, config_->num_threads);

  // Problem specific
  auto ransac_model = generateRANSACModel(qdata, mdata);

  // If a model wasn't successfully generated, clean up and return error
  if (ransac_model == nullptr) {
    vision::SimpleMatches inliers;
    auto &matches = *mdata.ransac_matches.fallback();
    matches.push_back(vision::RigMatches());
    LOG(ERROR) << "Model Has Failed!!!" << std::endl;
    mdata.success = false;
    mdata.steam_failure = true;
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
      mdata.success = false;
    } else {
      // Success, set the output (in the vehicle frame)
      auto T_s_v_q = *qdata.T_sensor_vehicle;
      auto T_s_v_m = config_->use_migrated_points
                         ? (*mdata.T_sensor_vehicle_map)[*mdata.map_id]
                         : (*mdata.T_sensor_vehicle_map)[*qdata.live_id];

      mdata.T_q_m =
          T_s_v_q.inverse() * lgmath::se3::Transformation(solution) * T_s_v_m;
      mdata.T_q_m->setZeroCovariance();
      mdata.success = true;
    }
  }

  if (inliers.size() <= static_cast<uint32_t>(config_->min_inliers)) {
    LOG(ERROR) << "RansacModule::" << __func__ << "(): " << inliers.size()
               << "/" << flattened_matches.size() << " is not enough inliers! ";
    inliers.clear();
    mdata.success = false;
  }

  // Inflate matches
  auto &matches = *mdata.ransac_matches.fallback();
  matches.push_back(vision::RigMatches());
  mirrorStructure(rig_matches, matches[rig_idx]);
  inflateMatches(inliers, matches[rig_idx]);
#endif
}
#if false
std::vector<vision::RigMatches> RansacModule::generateFilteredMatches(
    QueryCache &qdata, MapCache &mdata) {
  return *mdata.raw_matches;
}
#endif
void RansacModule::visualizeImpl(QueryCache &qdata, MapCache &mdata,
                                 const std::shared_ptr<const Graph> &graph,
                                 std::mutex &vis_mtx) {
#if false
  // check if visualization is enabled
  if (config_->visualize_ransac_inliers) {
    if (config_->use_migrated_points) {
      visualize::showMelMatches(vis_mtx, qdata, mdata, graph, "multi-exp-loc");
    } else if (mdata.ransac_matches.is_valid() == true) {
      visualize::showMatches(vis_mtx, qdata, mdata, *mdata.ransac_matches,
                             " RANSAC matches");
    }
  }
#endif
}

}  // namespace navigation
}  // namespace vtr