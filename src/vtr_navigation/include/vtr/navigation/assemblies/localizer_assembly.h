#pragma once

#include <vtr/navigation/assemblies/base_assembly.h>

#include <asrl/messages/LocalizationStatus.pb.h>
#include <asrl/messages/lgmath_conversions.hpp>
#include <asrl/vision/messages/bridge.hpp>

namespace vtr {
namespace navigation {

/** \brief Assembly to run modules associated with the localization pipeline.
 */
class LocalizerAssembly : public BaseAssembly {
 public:
  static constexpr auto type_str_ = "localizer";

  /** \brief there are no requirements on the assembly, so just return true.
   */
  bool verify() const { return true; }

  /** \brief Localize the frame data against the map vertex using the (sub)graph
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph) {
    // run the modules.
    mdata.loc_timer.fallback();
    BaseAssembly::run(qdata, mdata, graph);
  }

  /** \brief Updates the graph after the modules have run.
   *
   * In the case of the localizer assembly, this includes updating the landmarks
   * in the live run to include matches to landmarks in the map.
   */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           const VertexId &live_id) {
    // let the modules update the graph first.
    BaseAssembly::updateGraph(qdata, mdata, graph, live_id);

    // sanity check
    if (mdata.ransac_matches.is_valid() == false) {
      LOG(ERROR) << "LocalizerAssembly::" << __func__
                 << "() ransac matches not present";
      return;
    } else if (mdata.map_landmarks.is_valid() == false) {
      LOG(ERROR) << "LocalizerAssembly::" << __func__
                 << "() map landmarks not present";
      return;
    } else if (mdata.localization_status.is_valid() == false) {
      LOG(ERROR) << "LocalizerAssembly::" << __func__
                 << "() localization status not present";
      return;
    } else if (mdata.migrated_landmark_ids.is_valid() == false) {
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
    std::map<VertexId, std::shared_ptr<asrl::vision_msgs::RigLandmarks>>
        landmark_map;

    // Go through each rig.
    for (uint32_t rig_idx = 0; rig_idx < inliers.size(); ++rig_idx) {
      std::string &rig_name = rig_names.at(rig_idx);
      auto &rig_inliers = inliers[rig_idx];
      asrl::vision_msgs::Matches matches_msg;

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
          auto persistent_msg =
              asrl::messages::copyPersistentId(lm_obs.persistent);
          VertexId q_lm_vertex = graph->fromPersistent(persistent_msg);
          // if we havent loaded these landmarks, then load them.
          if (landmark_map.find(q_lm_vertex) == landmark_map.end()) {
            auto vertex = graph->at(q_lm_vertex);
            landmark_map[q_lm_vertex] =
                vertex->retrieveKeyframeData<asrl::vision_msgs::RigLandmarks>(
                    "/" + rig_name + "/landmarks");
          }

          // Get references / pointers to the landmarks.
          auto &query_landmarks = landmark_map[q_lm_vertex];
          auto channel_landmarks =
              query_landmarks->mutable_channels()->Mutable(channel_idx);
          // get the landmark track associated with the map landmark.
          auto &lm_track = migrated_landmark_ids[match.first];

          // match.first = idx into query lm
          // match.second = list of landmark ids
          // Get the  landmark track (list of matched landmarks across
          // experiences) associated with the query landmark.
          auto query_match =
              channel_landmarks->mutable_matches()->Mutable(lm_obs.index);
          // If this landmark has not been associated yet, then copy over the
          // matched landmarks track to the new landmark.
          if (lm_track->to().size() > 0) {
            query_match->mutable_to()->CopyFrom(
                migrated_landmark_ids[match.first]->to());
          }
          // add the map landmark as a to landmark in the matches.
          auto &new_match = *query_match->add_to();
          new_match.CopyFrom(migrated_landmark_ids[match.first]->from());

          // direct matches used for collaborative landmark tracking (first
          // pass)
          auto &direct_match_msg = *matches_msg.add_matches();
          direct_match_msg.mutable_from()->CopyFrom(query_match->from());
          direct_match_msg.add_to()->CopyFrom(new_match);
        }
      }

      // Save the matches to map landmarks
      std::string lm_match_str = "/" + rig_name + "/landmarks/matches";
      if (!graph->hasVertexStream(live_id.majorId(), lm_match_str)) {
        graph->registerVertexStream(live_id.majorId(), lm_match_str, true);
      }
      live_vtx->insert<asrl::vision_msgs::Matches>(lm_match_str, matches_msg,
                                                   *qdata.stamp);
    }
  }

 private:
  void saveResults(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<Graph> &graph,
                   const VertexId &live_id) {
    auto &inliers = *mdata.ransac_matches;
    // double run_time = (*mdata.loc_timer).elapsedMs();
    auto status = *mdata.localization_status;
    status.set_keyframe_time((*qdata.stamp).nanoseconds_since_epoch());
    status.set_query_id(live_id);
    status.set_map_id(*mdata.map_id);
    status.set_success(*mdata.success);
    status.set_localization_computation_time_ms((*mdata.loc_timer).elapsedMs());
    if (mdata.T_q_m.is_valid()) {
      *status.mutable_t_query_map() << (*mdata.T_q_m);
    }
    for (auto &rig : inliers) {
      for (auto &channel : rig.channels) {
        status.add_inlier_channel_matches(channel.matches.size());
      }
    }

    // get the run id and vertex
    auto vertex = graph->at(live_id);
    const Graph::RunIdType rid = live_id.majorId();

    // fill in the status
    std::string loc_results_str("/results/localization");
    if (!graph->hasVertexStream(rid, loc_results_str)) {
      graph->registerVertexStream(rid, loc_results_str, true);
    }
    vertex->insert<asrl::status_msgs::LocalizationStatus>(loc_results_str, status,
                                                    *qdata.stamp);
  }
};

}  // namespace navigation
}  // namespace vtr
