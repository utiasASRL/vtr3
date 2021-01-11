#if false
#include "asrl/navigation/modules/localization/CollaborativeLandmarks.hpp"
#include "asrl/vision/Types.hpp"
#include "asrl/pose_graph/evaluator/Common.hpp"
#include "asrl/vision/messages/bridge.hpp"
#include "asrl/common/timing/SimpleTimer.hpp"
#include <algorithm>

namespace asrl {
namespace navigation {

typedef CollaborativeLandmarksModule::State State;
typedef CollaborativeLandmarksModule::MatchMap MatchMap;
typedef CollaborativeLandmarksModule::MultiMatch MultiMatch;

// Get the sequential distance between two vertices
unsigned vertexDistance(VertexId a, VertexId b, const Graph & graph) {
  // Check for invalid vertex ids or missing runs
  if (a == VertexId::Invalid() || b == VertexId::Invalid() ||
      !graph.contains(a.majorId()) || !graph.contains(b.majorId()))
    return graph.numberOfVertices();
  // Make a the smaller of the two
  std::tie(a,b) = std::minmax(a,b);
  // Get <run_a,0> -> <run_b,0> (the run lengths between them)
  unsigned distance = 0;
  for (RunId rid = a.majorId(); rid < b.majorId(); ++rid) {
    try { distance += graph.run(rid)->vertices().size(); }
    catch(...) { }
  }
  // Add <run_b,0> -> b then subtract <run_a,0> -> a
  return distance + b.minorId() - a.minorId();
}

/// Decay the state from its current vid to a new vid
State decayState(State state, double decay_factor,
                 const VertexId & new_vid, const Graph & graph) {
  // Compute the decay we need for state.vid -> new_vid
  unsigned distance = vertexDistance(state.vid, new_vid, graph);
  distance = std::max(1u, distance); // Guarantee at least 1 decay (reprocessing)
  double decay = exp(-2.*decay_factor*distance);
  // Apply the decay to the state
  state.Qlive *= decay;
  for (auto & rstate : state.runs) {
    rstate.second.P *= decay;
    rstate.second.Q *= decay;
  }
  // And update the state's vid
  state.vid = new_vid;
  return state;
}

State dropStale(State state, double thresh, bool verbose = false) {
  // Remove any stale runs from the state map
  for (auto rit = state.runs.begin(); rit != state.runs.end(); /*++ inside*/) {
    bool stale = rit->second.P < thresh && rit->second.Q < thresh;
    LOG_IF(stale && verbose, INFO)
        << "Removing stale run " << rit->first << " from the collaborative state.";
    if (stale) rit = state.runs.erase(rit);
    else ++rit;
  }
  return state;
}

unsigned landmarkCounts(const vision_msgs::RigCounts & rig_msg) {
  unsigned count = 0;
  for (const vision_msgs::ChannelCount & channel_msg : rig_msg.channels()) {
    if (channel_msg.has_count()) {
      count += channel_msg.count();
    }
  }
  return count;
}

/// Collaborative filtering for experience triage.
/// Treats experiences as 'users' and landmark-match connected components as 'items'
/// Based on: http://dl.acm.org/citation.cfm?id=1864729
/// Liu, N. N., Zhao, Min, et al., Online Evolutionary Collaborative Filtering
/// Also consider: http://courses.ischool.berkeley.edu/i290-dm/s11/SECURE/a1-koren.pdf
/// 0. Get the local match history from the localization submap,
///    and find the connected components in the matches that count as an 'item'
/// 1. Follow the VO matches to previously matched landmarks
/// 2. Update the run-run terms, decayed from the previous vertex + VO matches
/// 3. Select the N most likely experiences
/// 4. Order the landmarks from the selected experiences based on likelihood
void CollaborativeLandmarksModule::run(
  QueryCache & qdata,
  MapCache & mdata,
  const std::shared_ptr<const Graph> & graph) {
  // start the master run timer to get alllll the time
  common::timing::SimpleTimer timer;
  double load_time = 0.;

  const VertexId & live_id = *qdata.live_id;
  auto & rig_names = *qdata.rig_names;
  std::vector<LandmarkFrame> & query_landmarks = *mdata.map_landmarks;
  pose_graph::RCGraphBase & submap = **mdata.localization_map;
  std::unordered_set<VertexId> submap_ids;

  // Clear the match map memory
  lm_counts_.clear();
  std::unordered_map<RunId, unsigned> match_counts;
  std::unordered_set<RunId> available_runs;
  live_lm_count_ = 0;

  // 0. Visit every vertex in the local submap (we count on submap extraction...)
  // ----------------------------------------------------------------------------
  for (auto vit = submap.begin(); vit != submap.end(); ++vit) {
    Vertex & vi = *vit->v();
    submap_ids.insert(vi.id());
    available_runs.insert(vi.id().majorId());
    // Go through every rig that we're using
    for (const std::string & rig_name : rig_names) {
      // update the number of landmarks found at this vertex
      common::timing::SimpleTimer load_timer;
      auto count_msg = vi.retrieveKeyframeData<vision_msgs::RigCounts>("/" + rig_name + "/landmarks/counts");
      load_time += load_timer.elapsedMs();
//      LOG_IF(!count_msg, ERROR) << "No landmark count messages for " << vi.id() << " " << rig_name;
      if (!count_msg) continue;
      lm_counts_[vi.id().majorId()] += landmarkCounts(*count_msg);

      if (submap_ids_.count(vi.id())) continue; // we already have this vertex in the cache from last time
      // update the landmark matches to other runs from this vertex
      load_timer.reset();
      auto matches_msg = vi.retrieveKeyframeData<vision_msgs::Matches>("/" + rig_name + "/landmarks/matches");
      load_time += load_timer.elapsedMs();
      if (!matches_msg) continue; // privileged runs don't have matches
      for (const vision_msgs::Match & match : matches_msg->matches()) {
        if (!match.to_size()) continue; // shouldn't happen, but just in case
        const vision_msgs::FeatureId & to_fid = match.to(0);
        try {
          VertexId to_vid = graph->fromPersistent(to_fid.persistent());
          match_cache_.emplace({{messages::copyLandmarkId(match.from()), messages::copyLandmarkId(to_fid)},
                                {{vi.id().majorId(),to_vid.majorId()},{vi.id()}}});
        } catch(...) { LOG(ERROR) << "Could not find vertex " << to_fid.DebugString(); }
      }
    }
  }

  // 1. Go through the live landmarks, and follow their matches into the match map
  // -----------------------------------------------------------------------------
  for (unsigned rig_idx = 0; rig_idx < rig_names.size(); ++rig_idx) {
    const vision::RigLandmarks & q_rig_lms = query_landmarks.at(rig_idx).landmarks;
    for (const vision::ChannelLandmarks & q_channel_lms : q_rig_lms.channels) {
      live_lm_count_ += q_channel_lms.points.cols();
      for (const vision::LandmarkMatch & lm_match : q_channel_lms.matches) {
        if (lm_match.to.empty()) continue;
        vision_msgs::FeatureId to_match = messages::copyLandmarkId(lm_match.to.back());

        // find the matched-to feature in the cache
        auto vp_it = match_cache_.find(messages::copyLandmarkId(to_match));
        if (vp_it == match_cache_.end()) { // match to just its run if not found
          ++match_counts[graph->fromPersistent(to_match.persistent()).majorId()];
        } else { // match to all the runs that saw it if found
          for(const auto & rid : vp_it->value.runs) ++match_counts[rid];
        }
      }
    }
  }

  // Clear all the old connected landmarks from the cache
  auto is_stale = [&](const MatchCache::ValuePack & vp) {
    return !submap_ids.count(vp.value.owner);
  };
  match_cache_.shrink(is_stale);
  submap_ids_ = std::move(submap_ids);

  // 2.A Update the run-run similarity terms (the state)
  // ---------------------------------------------------

  // Decay the state to the live vid
  state_ = decayState(std::move(state_), config_->similarity_decay, live_id, *graph);
  // Remove any stale runs from the state map
  state_ = dropStale(std::move(state_), 1e-3, config_->verbose);

  // 2.B Make a copy of the state, and apply the VO-followed match counts
  // --------------------------------------------------------------------

  State state_vo = state_;
  // Increment the landmark counter for the live run
  state_vo.Qlive += live_lm_count_;
  // Increment the landmark counter for all the map runs
  for (const auto & cnt : lm_counts_) state_vo.runs[cnt.first].Q += cnt.second;
  // Increment the match counter for all the runs
  for (const auto & cnt : match_counts) state_vo.runs[cnt.first].P += cnt.second;

  // 3. Compute and sort the scores for the runs
  // -------------------------------------------
  ScoredRids distance_rids;
  for (const auto & rpair : state_vo.runs) {
    if (rpair.second.P < 1.) continue; // not even 1 match... nah (also block legacy runs)
    if (!available_runs.count(rpair.first)) continue; // No vertices available, don't bother
    float distance = 1 - rpair.second.P/sqrt(rpair.second.Q*state_vo.Qlive);
    distance_rids.emplace(distance, rpair.first);
  }

  // Decide if we should be in the loop based configuration,
  bool in_the_loop = config_->in_the_loop
      && !distance_rids.empty();

  // Figure out the recommended experiences from the sorted runs,
  // recommend them in the cache if this module is in the loop
  RunIdSet recommended = fillRecommends(
        in_the_loop ? &*mdata.recommended_experiences.fallback() : nullptr,
        distance_rids, config_->num_exp);

  // 4. Recommend landmarks
  // ----------------------

  scored_lms_.clear();
  if (config_->recommend_landmarks && in_the_loop) {
    // Get the run similarities and denominator for the scores
    std::unordered_map<RunId,float> run_similarity;
    float denom = 0;
    for (const auto & dr : distance_rids) {
      float similarity = 1-dr.first;
      run_similarity.emplace(dr.second, similarity);
      denom += similarity;
    }

    // Add each vertex's generic landmarks (not matched)
    for (VertexId vid : submap_ids) {
      auto sim_it = run_similarity.find(vid.majorId());
      if (sim_it == run_similarity.end()) continue;
      auto pid = messages::copyPersistentId(graph->toPersistent(vid));
      scored_lms_.emplace(sim_it->second, vision::LandmarkId(pid, -1, -1));
    }

    // Get the landmark scores
    // TODO: more efficient to sort after building w/ a vector
    for (const MatchCache::ValuePack & cm : match_cache_.connected()) {
      // TODO If isStale, skip it (if we don't remove stales above)
      // Find the score for this landmark (sum of similarities of connected runs)
      float score = 0.f; bool found_rec = false;
      for (const RunId & rid : cm.value.runs) {
        auto similarity_it = run_similarity.find(rid);
        if (similarity_it != run_similarity.end())
          score += similarity_it->second;
        found_rec = found_rec || recommended.count(rid);
      }
      score /= denom;
      if (!found_rec) continue; // We're done early if none will be recommended
      // Add all of the landmarks from recommended experiences to the scored list
      for (const vision::LandmarkId & lid : cm.keys) {
        VertexId vid; try { vid = graph->fromPersistent(messages::copyPersistentId(lid.persistent)); }
        catch (std::out_of_range) { continue; }
        if (recommended.count(vid.majorId()))
          scored_lms_.emplace(score, lid);
      }
    }
  }

  // Done! Clean up
  // --------------

  // Stop the timer
  double run_time = timer.elapsedMs() - load_time;

  // Convert the BoW descriptor to a message
  Vertex::Ptr query_vertex = graph->at(live_id);
  status_msgs::ExpRecogStatus & status_msg = status_msg_to_save_;
  status_msg.Clear();

  // Populate the results status message
  status_msg.mutable_keyframe_time()->CopyFrom(query_vertex->keyFrameTime());
  status_msg.set_query_id(query_vertex->id());
  status_msg.set_algorithm(status_msgs::ExpRecogStatus_Algorithm_COLLAB);
  status_msg.set_in_the_loop(in_the_loop);
  // The bag-of-words cosine distances for each run
  for (const auto & run_score : distance_rids) {
    auto & dist_msg = *status_msg.add_cosine_distances();
    dist_msg.set_run_id(run_score.second);
    dist_msg.set_cosine_distance(run_score.first);
  }
  // The recommended runs for localization
  for (const RunId & rid : recommended)
    status_msg.add_recommended_ids(rid);
  // The computation and data load times
  status_msg.set_computation_time_ms(run_time);
  status_msg.set_load_time_ms(load_time);

  // Debug string
  LOG_IF(config_->verbose, INFO) << "CF-vo " << "ks: " << match_cache_.id_map().size() << " "
                                 << "vs: " << match_cache_.connected().size() << " " << status_msg;
}

/// Update the state post-matching, save results
/// 1. Follow ransac-confirmed localization matches
/// 2. Update the state with these matches (instead of followed vo matches)
/// 3. Save Results
void CollaborativeLandmarksModule::updateGraph(
  QueryCache & qdata,
  MapCache & mdata,
  const std::shared_ptr<Graph> & graph,
  VertexId live_id) {

  // Start the timer for alllll of the things
  common::timing::SimpleTimer timer;

  // we need to take all of the landmark matches and update connected store them in the new landmarks
  const auto & inliers = *mdata.ransac_matches;
  const auto & migrated_landmark_ids = *mdata.migrated_landmark_ids;

  // 1. Go through all of the inlier matches to find matches to runs
  // ---------------------------------------------------------------
  std::unordered_map<RunId, unsigned> run_cnts;
  for(const auto & rin : inliers) { for(const auto & cin : rin.channels) { for(auto &match : cin.matches) {
    const vision_msgs::Match & matched_lm = *migrated_landmark_ids[match.first];
    VertexId vid = graph->fromPersistent(matched_lm.from().persistent());

    // add landmark id -> that landmark's run, owned by that landmark's vertex
    auto cm_it_new = match_cache_.emplace({{messages::copyLandmarkId(matched_lm.from())},
                                           ConnectedMatches{{vid.majorId()},vid}});
    // find all the runs that matched to this connected landmark, count them as matches
    for (const auto & rid : cm_it_new.first->value.runs) run_cnts[rid]++;
  } } }

  // 2. Update the state using the ransac-confirmed matches (forget the vo matches)
  // ------------------------------------------------------------------------------

  // Increment the match counter for all the runs
  for (const auto & cnt : run_cnts) {
    state_.runs[cnt.first].P += cnt.second;
    // in case we didn't pick this up, add the counts
    if (!lm_counts_.count(cnt.first)) lm_counts_.emplace(cnt);
  }

  // Increment the landmark counter for the live run
  state_.Qlive += live_lm_count_;
  // Increment the landmark counter for all the map runs
  float pessimism = 0.1;
  for (const auto & cnt : lm_counts_) {
    // Did we try to match against this run?
    bool tried_run = !mdata.recommended_experiences
        || mdata.recommended_experiences->count(cnt.first);
    if (tried_run) {
      state_.runs[cnt.first].Q += cnt.second;
    } else {
      unsigned match_cnt = run_cnts.count(cnt.first) ? run_cnts[cnt.first] : 0;
      unsigned lm_cnt = std::max(match_cnt, cnt.second);
      unsigned attempts = match_cnt + unsigned((lm_cnt-match_cnt)*pessimism);
      state_.runs[cnt.first].Q += attempts;
    }
  }

  // todo temp
  if (!scored_lms_.empty()) {
//    std::map<vision::LandmarkId, float> scores;
    std::multimap<float, vision::LandmarkId> scores;
    std::set<vision::LandmarkId> matches;

    // Get all the matches
    for(const auto & rin : inliers) { for(const auto & cin : rin.channels) { for(auto &match : cin.matches) {
      const vision_msgs::Match & matched_lm = *migrated_landmark_ids[match.first];
      matches.emplace(messages::copyLandmarkId(matched_lm.from()));
    } } }

    // Sort the matches
    for (const auto & sl : scored_lms_) {
//      sum_all += sl.first;
      if (matches.count(sl.second)) {
//        sum_match += sl.first;
        scores.emplace(sl);
      }
    }

    std::map<vision::LandmarkId, float> lm_scores;
    double sum_match = 0.;
    for (const auto & sl : scored_lms_) lm_scores.emplace(sl.second, sl.first);
    for (const vision::LandmarkId & lid : matches) {
      auto score_it = lm_scores.find(lid);
      if (score_it == lm_scores.end())
        score_it = lm_scores.find(vision::LandmarkId(lid.persistent, -1, -1));
      if (score_it == lm_scores.end()) continue;
      sum_match += score_it->second;
    }
    double sum_all = 0.;
    for (const auto & mlid : migrated_landmark_ids) {
      const vision::LandmarkId & lid = messages::copyLandmarkId(mlid->from());
      auto score_it = lm_scores.find(lid);
      if (score_it == lm_scores.end())
        score_it = lm_scores.find(vision::LandmarkId(lid.persistent, -1, -1));
      if (score_it == lm_scores.end()) continue;
      sum_all += score_it->second;
    }

    LOG(INFO) << "CF-lm: range: " << scored_lms_.begin()->first << "-" << scored_lms_.rbegin()->first
              << " [" << sum_all/migrated_landmark_ids.size() << "] "
              << (double(scored_lms_.size())/migrated_landmark_ids.size()) << " % "
              << "match: " << scores.begin()->first << "-" << scores.rbegin()->first
              << " [" << (sum_match/matches.size()) << "] "
              << (double(scores.size())/matches.size()) << " % ";
  }


  // 3. Save the results
  // -------------------

  // Update the profile time with this functions duration
  double update_time = timer.elapsedMs();
  status_msg_to_save_.set_computation_time_ms(status_msg_to_save_.computation_time_ms()+update_time);

  // Build the print string
  if (config_->verbose) {
    // Score the runs so that they are sorted best->worst for printing
    ScoredRids distance_rids;
    for (const auto & rpair : state_.runs) {
      if (rpair.second.P < 1.) continue; // not even 1 match... nah.
      float distance = 1 - rpair.second.P/sqrt(rpair.second.Q*state_.Qlive);
      distance_rids.emplace(distance, rpair.first);
    }
    // Build a string of P/Q ratios for the best runs
    std::ostringstream p_str; p_str << std::fixed << std::setprecision(0);
    for (const auto & dist_rid : distance_rids) {
      if (p_str.str().size() > 150) { p_str << "... "; break; }
      const auto & rpair = *state_.runs.find(dist_rid.second);
      p_str << rpair.first << ":" << rpair.second.P << "/" << rpair.second.Q << " ";
    }
    LOG(INFO) << "CF-loc up: " << update_time << " ms P/Q: " << p_str.str();
  }

  // Save the status/results message
  if (status_msg_to_save_.query_id() == live_id) {
    Vertex::Ptr vertex = graph->at(live_id);
    RunId rid = vertex->id().majorId();
    std::string results_stream = "/results/collaborative_landmarks";
    if (!graph->hasVertexStream(rid, results_stream))
      graph->registerVertexStream(rid, results_stream, true);
    vertex->insert(results_stream, status_msg_to_save_, vertex->keyFrameTime());
  }
}

} // navigation
} // asrl
#endif
