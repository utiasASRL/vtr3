#include <vtr_navigation/assemblies/localizer_assembly.hpp>

#include <vtr_messages/msg/localization_status.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_lgmath_extensions/conversions.hpp>

namespace vtr {
namespace navigation {

void LocalizerAssembly::run(QueryCache &qdata, MapCache &mdata,
                 const std::shared_ptr<const Graph> &graph) {

  // run the modules.
  mdata.loc_timer.fallback();
  BaseAssembly::run(qdata, mdata, graph);

}

void LocalizerAssembly::updateGraph(QueryCache &qdata, MapCache &mdata,
                         const std::shared_ptr<Graph> &graph,
                         const VertexId &live_id) {
  // let the modules update the graph first.
    BaseAssembly::updateGraph(qdata, mdata, graph, live_id);

    // sanity check
    if (!mdata.ransac_matches.is_valid()) {
      LOG(ERROR) << "LocalizerAssembly::" << __func__
                 << "() ransac matches not present";
      return;
    } else if (!mdata.map_landmarks.is_valid()) {
      LOG(ERROR) << "LocalizerAssembly::" << __func__
                 << "() map landmarks not present";
      return;
    } else if (!mdata.localization_status.is_valid()) {
      LOG(ERROR) << "LocalizerAssembly::" << __func__
                 << "() localization status not present";
      return;
    } else if (!mdata.migrated_landmark_ids.is_valid()) {
      LOG(ERROR) << "LocalizerAssembly::" << __func__
                 << "() migrated landmark ID's not present";
      return;
    }

    // save the localization results for analysis
    saveResults(qdata, mdata, graph, live_id);

    // we need to take all of the landmark matches and store them in the new
    // landmarks
    auto &inliers = *mdata.ransac_matches;
    auto &query_landmarks = *mdata.map_landmarks;
    auto migrated_landmark_ids = *mdata.migrated_landmark_ids;
    auto &rig_names = *qdata.rig_names;
    auto live_vtx = graph->at(live_id);

    // map to keep track of loaded landmarks.
    std::map<VertexId, std::shared_ptr<vtr_messages::msg::RigLandmarks>>
        landmark_map;

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
          auto persistent_msg = messages::copyPersistentId(lm_obs.persistent);
          VertexId q_lm_vertex = graph->fromPersistent(persistent_msg);
          // if we havent loaded these landmarks, then load them.
          if (landmark_map.find(q_lm_vertex) == landmark_map.end()) {
            auto vertex = graph->at(q_lm_vertex);
            landmark_map[q_lm_vertex] =
                vertex->retrieveKeyframeData<vtr_messages::msg::RigLandmarks>(rig_name + "_landmarks");
          }

          // Get references / pointers to the landmarks.
          auto &query_landmarks = landmark_map[q_lm_vertex];
          auto channel_landmarks = query_landmarks->channels[channel_idx];
          // get the landmark track associated with the map landmark.
          auto &lm_track = migrated_landmark_ids[match.first];            // todo (Ben): verify this all works properly

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
      auto run = graph->run((*qdata.live_id).majorId());
      std::string landmark_match_str(rig_name + "_landmarks_matches");
      if (!run->hasVertexStream(landmark_match_str)) {
        run->registerVertexStream<vtr_messages::msg::Matches>(landmark_match_str,true);
      }
      live_vtx->insert(landmark_match_str, matches_msg, *qdata.stamp);
    }
}

void LocalizerAssembly::saveResults(QueryCache &qdata, MapCache &mdata,
                 const std::shared_ptr<Graph> &graph,
                 const VertexId &live_id) {
  auto &inliers = *mdata.ransac_matches;
  auto status = *mdata.localization_status;
  status.keyframe_time = (*qdata.stamp).nanoseconds_since_epoch;
  status.query_id = live_id;
  pose_graph::VertexId map_id = *mdata.map_id;
  status.map_id = map_id;
  status.success = *mdata.success;
  status.localization_computation_time_ms = (*mdata.loc_timer).elapsedMs();

  if (mdata.T_q_m.is_valid()) {
    status.t_query_map << *mdata.T_q_m;
  }

  for (auto &rig : inliers) {
    for (auto &channel : rig.channels) {
      status.inlier_channel_matches.push_back(channel.matches.size());
    }
  }

  // get the run id and vertex
  auto vertex = graph->at(live_id);

  // fill in the status
  auto run = graph->run((*qdata.live_id).majorId());
  std::string loc_status_str("results_localization");
  if (!run->hasVertexStream(loc_status_str)) {
    run->registerVertexStream<vtr_messages::msg::LocalizationStatus>(loc_status_str,true);
  }
  vertex->insert(loc_status_str, status, *qdata.stamp);
}

}  // namespace navigation
}  // namespace vtr
