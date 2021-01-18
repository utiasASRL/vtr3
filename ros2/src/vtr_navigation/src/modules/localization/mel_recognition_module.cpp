#include <vtr_navigation/modules/localization/mel_recognition_module.hpp>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>
#include <vtr_vision/features/bow/incremental_bow_trainer.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_messages/msg/matches.hpp>
#include <vtr_messages/msg/bow_descriptor.hpp>

#include <algorithm>

namespace vtr {
namespace navigation {

void MelRecognitionModule::run(QueryCache &qdata, MapCache &mdata,
                               const std::shared_ptr<const Graph> &graph) {
  // Make sure we have a config
  if (!config_) {
    LOG(ERROR) << "Module exp_recog has been run with no config set!";
    return;
  }

  // Status
  LOG_IF(config_->verbose, INFO)
    << "exp_recog >>> vertex " << *qdata.live_id << " <<<";

  // Start the timer
  common::timing::SimpleTimer timer;
  double start_ms = 0;

  // Initialize flags, local variables, etc.
  if (!initialize(qdata, mdata)) {
    // We don't process the first frame, because we want VO tracks
    LOG(DEBUG) << "exp_recog is skipping the first frame.";
    return;
  }

  // initialize BoW stream. try-catch needed for runs with stream missing (e.g. teach)
  if (!streams_init_) {
    for (const auto &r : graph->runs()) {
      try {
        if (r.second->isVertexStreamSet("bagofwords") == false) {
          r.second->setVertexStream<vtr_messages::msg::BowDescriptor>("bagofwords");
        }
      } catch (std::exception &e) {
        LOG_IF(r.second->id() != 0, DEBUG)
          << "Could not find the bagofwords stream for vertex " << r.second->id();
      }
      try {
        if (r.second->isVertexStreamSet("vocabulary") == false) {
          r.second->setVertexStream<vtr_messages::msg::RigFeatures>("vocabulary");
        }
      } catch (std::exception &e) {
        LOG_IF(r.second->id() != 0, DEBUG)
          << "Could not find the bagofwords stream for vertex " << r.second->id();
      }
    }
    streams_init_ = true;
  }

  // Make sure the candidate landmarks exist
  if (!qdata.candidate_landmarks) {
    LOG(WARNING) << "The candidate landmarks for " << *qdata.live_id << " are invalid.";
    return;
  }

  // Build the landmark ptr vector (workaround)
  // This is built so that the live qdata.candidate_landmarks
  // and previous mdata.map_landmarks have the same form
  const vision::SuiteLandmarks &live_suite_lms = *qdata.candidate_landmarks;
  RigLandmarkConstPtrs live_rig_lm_ptrs;
  live_rig_lm_ptrs.reserve(live_suite_lms.size());
  for (const vision::RigLandmarks &live_rig_lms : live_suite_lms) {
    live_rig_lm_ptrs.push_back(&live_rig_lms);
  }

  // Initialize the BOW construction container
  constructed_bow_to_save_ = std::make_shared<BowConstruct>();
  BowConstruct &construct = *constructed_bow_to_save_;
  construct.vid = *qdata.live_id;
  initializeConstruct(live_rig_lm_ptrs, &construct);

  // Get the vertices we'll localize against. The root is the active vertex for VO,
  // we're living in refined vo for now, have to do it this way :(
  start_ms = timer.elapsedMs();
  MapVertices map_vertices = getMapVertices(graph, construct.vid, construct.vid);
  double load_time = timer.elapsedMs() - start_ms;
  LOG_IF(config_->verbose, INFO)
    << "exp_recog[getMapVertices] took " << load_time << " ms.";

  // Re-use discretization from VO, and mark new observations as unmatched
  start_ms = timer.elapsedMs();
  bool followed_tracks = followVoTracks(construct, mdata, graph);
  // todo: currently this always fails as mdata.ransac_matches isn't valid.
  // The same problem seems to exist in VTR2.
  if (!followed_tracks) {
    // we have failed :( remove this so it doesn't get saved
    constructed_bow_to_save_.reset();
    return;
  }
  LOG_IF(config_->verbose, INFO)
    << "exp_recog[followVoTracks] took " << timer.elapsedMs() - start_ms << " ms.";

  // Match against the vocabulary from the local map
  start_ms = timer.elapsedMs();
  unsigned num_map_matches = matchAgainstMap(live_rig_lm_ptrs, &construct, map_vertices, graph);
  LOG_IF(config_->verbose, INFO)
    << "exp_recog[matchAgainstVocabulary] found "
    << num_map_matches << " matches in "
    << timer.elapsedMs() - start_ms << " ms.";

  // Cluster features that weren't matched
  start_ms = timer.elapsedMs();
  clusterUnmatchedObs(live_rig_lm_ptrs, &construct, graph);
  LOG_IF(config_->verbose, INFO)
    << "exp_recog[clusterUnmatchedObs] took " << timer.elapsedMs() - start_ms << " ms.";

  // Turn the BoW descriptor into a sliding-window descriptor
  const vision::BowDescriptor &compare_bow
      = config_->sliding_window
        ? slidingWindowify(construct.bow)
        : construct.bow;

  // Match against the map experiences, see who is best :D
  // TODO (old) maybe this piece can move to the localization pipeline
  {
    // Calculate the scores (cosine distance) to map BoW descriptors
    ExperienceDifferences differences =
        recognizeExperience(compare_bow, construct.vid.majorId(), map_vertices);

    // export to the localization pipeline
    RunIdSet best_exp = filterExperience(differences);
    if (config_->in_the_loop) {
      if (map_vertices.size() > (unsigned) config_->num_desired_experiences) {
        mdata.recommended_experiences = best_exp;
      } else {
        LOG_IF(config_->verbose, INFO) << "Not setting mask, too few experiences.";
      }
    }
    recordResults(graph->at(construct.vid), timer.elapsedMs(),
                  differences, best_exp);
  }

  // We can now save this as the previous for next time
  persistent_prev_id_ = construct.vid;
  persistent_prev_obs2word_ = std::make_shared<BowLibrary>(construct.obs2word);

  // Overall timing
  LOG_IF(config_->verbose, INFO)
    << "exp_recog <<< done in " << timer.elapsedMs() << " ms >>>" << std::endl;
}

bool MelRecognitionModule::initialize(QueryCache &qdata, MapCache &mdata) {
  // Set some status flags for how the frame should be handled
  VertexId live_id = *qdata.live_id,
      prev_id = live_id - 1;
  bool first_frame = live_id.minorId() == 0;

  // Check to see if we're processing the same live id twice (e.g. during merge)
  bool is_repeated = live_id <= persistent_prev_id_;
  // We can continue as long as the live id hasn't been processed before
  return !is_repeated;

  // Check to see if we need to clear out the previous cached data (if not sequential)
  bool is_sequential = persistent_prev_id_ == prev_id;
  bool clear_last_frame = first_frame || !is_sequential || !persistent_prev_obs2word_.get();
  if (clear_last_frame) {
    persistent_prev_id_ = VertexId::Invalid();
    persistent_prev_obs2word_.reset();
    //window_bow_.reset();
  }

  // Warn if this unlikely scenario should arise
  if (is_sequential && clear_last_frame) {
    LOG(WARNING) << "exp_recog missing cached data for " << live_id - 1;
  }

  // We can continue as long as it's not the first frame
  return !first_frame;
}

void MelRecognitionModule::initializeConstruct(const RigLandmarkConstPtrs &query_suite_lm,
                                               BowConstruct *construct_ptr) {
  // Check input
  if (!construct_ptr) return;
  BowConstruct &construct = *construct_ptr;

  // Resize the matched landmarks container (flags for matched/worded) to the query size
  SuiteMask &suite_mask = construct.matched_landmarks;
  suite_mask.clear();
  suite_mask.resize(query_suite_lm.size());
  for (unsigned rig_id = 0; rig_id < query_suite_lm.size(); ++rig_id) {
    const std::vector<vision::ChannelLandmarks> &query_rig_lm = query_suite_lm[rig_id]->channels;
    RigMask &rig_mask = suite_mask[rig_id];
    rig_mask.resize(query_rig_lm.size());
    for (unsigned channel_id = 0; channel_id < query_rig_lm.size(); ++channel_id) {
      unsigned num_lm = query_rig_lm[channel_id].appearance.keypoints.size();
      ChannelMask &channel_mask = rig_mask[channel_id];
      channel_mask.resize(num_lm, {false, false});
    }
  }
}

// ^^^ MAIN AND INITIALIZATION ^^^ //

// vvv RE-USING VO TRACKS TO AVOID RE-MATCHING MATCHED THINGS vvv //

bool MelRecognitionModule::followVoTracks(BowConstruct &live_construct,
                                          MapCache &mdata,
                                          const std::shared_ptr<const Graph> &graph) {
  VertexId live_id = live_construct.vid, prev_vo_id = live_id - 1;

  // status
  LOG_IF(config_->verbose, INFO)
    << "Following raw VO tracks from "
    << live_id << " to " << prev_vo_id;

  if (!mdata.ransac_matches || !mdata.map_landmarks) {
    LOG(ERROR) << "Could not load VO track data from cache";
    return false;
  }

  //const vision::SuiteMatches & suite_matches = *mdata.raw_matches;
  const vision::SuiteMatches &suite_matches = *mdata.ransac_matches;
  const LandmarkFrames &map_frames = *mdata.map_landmarks;
  SuiteMask &suite_is_matched = live_construct.matched_landmarks;

  // Re-used landmark id that keeps track of the hyper indices
  vision::LandmarkId live_obs_id;
  live_obs_id.persistent = messages::copyPersistentId(graph->toPersistent(live_id));
  live_obs_id.camera = 0;

  // Loop over all of the live frame observations
  for (unsigned rig_idx = 0; rig_idx < suite_matches.size(); ++rig_idx) {
    live_obs_id.rig = rig_idx;

    // refs
    const vision::RigMatches &rig_matches = suite_matches[rig_idx];
    const vision::RigObservations &map_rig_obs = map_frames[rig_idx].observations;

    // mask
    RigMask &rig_is_matched = suite_is_matched[rig_idx];

    for (unsigned channel_idx = 0; channel_idx < rig_matches.channels.size(); ++channel_idx) {
      live_obs_id.channel = channel_idx;

      // refs
      const vision::ChannelMatches &channel_matches = rig_matches.channels[channel_idx];
      const vision::ChannelObservations &map_channel_obs = map_rig_obs.channels[channel_idx];
      const vision::LandmarkMatches &map_camera_obs_lm = map_channel_obs.cameras[0].landmarks;

      // mask
      ChannelMask &channel_is_matched = rig_is_matched[channel_idx];

      // This is where we actually do the work. All that ^ is because we decided to be soooo general :P
      unsigned num_followed = 0;
      for (const vision::SimpleMatch &match : channel_matches.matches) {
        // The landmark & word ids in full, glorious hyper index format
        // Note: the simple matches are <map,live>
        live_obs_id.index = match.second;
        const vision::LandmarkId &prev_obs_id = map_camera_obs_lm[match.first].from;

        // it's tracked, but maybe not in the vocab yet
        channel_is_matched[match.second].tracked = true;

        // let's re-use the word from the previous keyframe
        std::stringstream following_ss;
        following_ss << "following: " << live_obs_id << " to " << prev_obs_id;

        // if we can't find it in the last frame's vocab map,
        // then we need to find the match ourselves
        if (!persistent_prev_obs2word_.get()) continue;
        const auto &word_it = persistent_prev_obs2word_->find(prev_obs_id);
        if (word_it == persistent_prev_obs2word_->end()) continue;

        const vision::LandmarkId &word = word_it->second;
        following_ss << " to " << word;
        // LOG(INFO) << following_ss.string();

        ++live_construct.bow[word];
        // and update our map for future use
        live_construct.obs2word[live_obs_id] = word;

        // Woo! this one is matched :)
        // This means we won't have to look for it later, or cluster it into the vocabulary
        channel_is_matched[match.second].worded = true;
        ++num_followed;
      } //

      LOG_IF(config_->verbose && !channel_matches.matches.empty(), INFO)
        << "Followed " << num_followed << " / "
        << channel_matches.matches.size() << " matches on channel "
        << channel_idx << ".";
    }   // loop over all observations
  }     //

  return true;
}

// ^^^ RE-USING VO TRACKS TO AVOID RE-MATCHING MATCHED THINGS ^^^ //

// vvv MATCHING AGAINST THE LOCAL MULTI-EXPERIENCE VOCABULARY vvv //

MelRecognitionModule::MapVertices
MelRecognitionModule::getMapVertices(const Graph::ConstPtr &graph,
                                     const VertexId &root_id,
                                     const VertexId &live_node) {
  // Set up our evaluators
  auto temporal_weight
      = pose_graph::eval::Weight::TemporalDirect<Graph>::MakeShared();
  temporal_weight->setGraph((void *) graph.get());

  // Create the output
  MapVertices map_vertices;

  // Iterate over a temporal window
  auto map_itr = graph->beginDijkstra(root_id, config_->temporal_depth,
                                      pose_graph::eval::Mask::Const::MakeShared(true, true),
                                      temporal_weight);
  for (; map_itr != graph->end(); ++map_itr) {

    // Iterate over the spatial neighbours
    VertexId map_id = map_itr->v()->id();
    //LOG(INFO) << "Visiting vertex " << map_id;

    // Only past vertices, and not the previous (used vo tracks already)
    if (map_id >= VertexId(live_node.majorId(), live_node.minorId() - 1)) continue;
    // Record the vertex
    const auto &map_vertex = map_itr->v();

    map_vertices[map_id.majorId()].push_back(map_vertex);
  }

  // status
  if (config_->verbose) {
    std::stringstream neighbour_ss;
    neighbour_ss << "Adding map: ";
    for (const auto &run_list : map_vertices) {
      for (const Vertex::Ptr &map_vertex : run_list.second) {
        neighbour_ss << map_vertex->id() << " ";
      }
    }
    LOG(INFO) << neighbour_ss.str();
  }

  return map_vertices;
}

/// @brief Matches the current landmarks across multiple experiences.
int MelRecognitionModule::matchAgainstMap(const RigLandmarkConstPtrs &query_suite_lm,
                                          BowConstruct *construct_ptr,
                                          const MapVertices &map_vertices,
                                          const std::shared_ptr<const Graph> &graph) {
  // iterate over experiences
  std::stringstream match_ss;
  if (config_->verbose) match_ss << "Found experience:matches : ";
  unsigned total_matched = 0;
  for (const auto &map_experience : map_vertices) {
    unsigned experience_matched = 0;
    // iterate over keyframes
    for (const auto &map_vertex : map_experience.second) {
      experience_matched += matchVertex(query_suite_lm, construct_ptr, map_vertex, graph);
    }
    if (config_->verbose) match_ss << map_experience.first << ":" << experience_matched << " ";
    total_matched += experience_matched;
  }

  LOG_IF(config_->verbose, INFO) << match_ss.str();
  return total_matched;
}

/// @brief Finds matches between the query landmarks and map landmarks found in a given vertex.
int MelRecognitionModule::matchVertex(const RigLandmarkConstPtrs &query_suite_lm,
                                      BowConstruct *construct_ptr,
                                      const Vertex::Ptr &map_vertex,
                                      const std::shared_ptr<const Graph> &graph) {
  // Check input
  if (!construct_ptr) return 0;
  BowConstruct &construct = *construct_ptr;

  auto live_persistent = messages::copyPersistentId(graph->toPersistent(construct.vid));
  auto map_persistent = messages::copyPersistentId(graph->toPersistent(map_vertex->id()));
  vision::LandmarkId query_lm_id(live_persistent, 0, 0, 0, 0);
  vision::LandmarkId vocab_lm_id(map_persistent, 0, 0, 0, 0);

  // TODO parallelize the matching over rigs

  // iterate over rigs
  unsigned total_matched = 0;
  for (uint32_t rig_id = 0; rig_id < query_suite_lm.size(); ++rig_id) {
    query_lm_id.rig = rig_id;
    vocab_lm_id.rig = rig_id;
    const std::string &rig_name = query_suite_lm[rig_id]->name;
    const std::vector<vision::ChannelLandmarks> &query_rig_lm = query_suite_lm[rig_id]->channels;

    // load vocabulary data from the graph.
    // Note: in VTR2 we had separate vocabs for each rig but don't think this is necessary
    std::string stream_name = "vocabulary";
    map_vertex->load(stream_name);
    auto rig_vocab_msg
        = map_vertex->retrieveKeyframeData<vtr_messages::msg::RigFeatures>(stream_name);

    // check loaded messages
    if (!rig_vocab_msg) {
      LOG_IF(map_vertex->id().minorId() != 0, DEBUG)
        << "exp_recog could not load stream " << stream_name
        << " for vertex " << map_vertex->id();
      continue;
    }

    // double check the number of channels
    if ((unsigned) rig_vocab_msg->channels.size() != query_rig_lm.size()) {
      LOG(WARNING) << "live has " << query_rig_lm.size()
                   << " channels, but vocab has " << rig_vocab_msg->channels.size();
    }
    unsigned max_num_channels = std::min((unsigned) rig_vocab_msg->channels.size(),
                                         (unsigned) query_rig_lm.size());

    // iterate over channels
    for (uint32_t channel_id = 0; channel_id < max_num_channels; ++channel_id) {
      query_lm_id.channel = channel_id;
      vocab_lm_id.channel = channel_id;
      const vision::Features &query_channel_lm = query_rig_lm[channel_id].appearance;
      const auto &channel_vocab_msg = rig_vocab_msg->channels.at(channel_id);
      if (channel_vocab_msg.name != query_rig_lm[channel_id].name) {
        LOG(ERROR) << "channel " << channel_id
                   << " live name is: " << query_rig_lm[channel_id].name
                   << " but vocab name is: " << channel_vocab_msg.name;
        continue;
      }

      if (channel_vocab_msg.cameras.empty()) {
        LOG(ERROR) << "channel " << channel_vocab_msg.name
                   << " has no cameras.";
        continue;
      }

      // Match this channel
      total_matched += matchChannel(query_channel_lm, query_lm_id, construct_ptr,
                                    channel_vocab_msg.cameras.at(0),
                                    vocab_lm_id);
    }
  }

  return total_matched;
}

/// @brief Finds matches between query and map for a given channel.
int MelRecognitionModule::matchChannel(
    const vision::Features &query_channel_lm,
    vision::LandmarkId query_lm_id,
    BowConstruct *construct_ptr,
    const vtr_messages::msg::Features &map_channel_vocab_msg,
    vision::LandmarkId map_vocab_lm_id
) {

  // Check input
  if (!construct_ptr) return 0;
  BowConstruct &construct = *construct_ptr;
  ChannelMask &channel_matched = construct.matched_landmarks[query_lm_id.rig][query_lm_id.channel];

  // helpful query refs
  const auto &step_size = query_channel_lm.feat_type.bytes_per_desc;

  // iterate through the vocabulary
  unsigned total_matched = 0;
  for (int32_t map_vocab_id = 0; map_vocab_id < map_channel_vocab_msg.keypoint_info.size(); ++map_vocab_id) {
    map_vocab_lm_id.index = map_vocab_id;
    // map landmark refs
    const auto &map_keypoint_info = map_channel_vocab_msg.keypoint_info.at(map_vocab_id);
    const auto &map_descriptor_string = map_channel_vocab_msg.descriptors;
    auto *map_descriptor = (float *) &map_descriptor_string.data()[step_size * map_vocab_id];

    // iterate through every query landmark
    for (uint32_t query_id = 0; query_id < query_channel_lm.feat_infos.size(); ++query_id) {

      // If we didn't track this with vo, or we have converted it to a word already
      if (channel_matched[query_id].tracked == false ||
          channel_matched[query_id].worded == true)
        continue;

      // query landmark information
      query_lm_id.index = query_id;
      if (query_id >= query_channel_lm.keypoints.size()) {
        LOG(ERROR) << "ER: query is " << query_id << " but kps is " << query_channel_lm.keypoints.size();
        continue;
      }
      if (query_id >= (uint32_t) query_channel_lm.descriptors.rows) {
        LOG(ERROR) << "ER: query is " << query_id << " but desc is " << query_channel_lm.descriptors.rows;
        continue;
      }
      const cv::KeyPoint &query_kp = query_channel_lm.keypoints[query_id];
      const auto *query_descriptor = query_channel_lm.descriptors.ptr<float>(query_id);

      // Check the scale/octave
      if (config_->compare_octave &&
          query_kp.octave != map_keypoint_info.scale)
        continue;
      // Check the laplacian bit (light-on-dark, or reverse)
      if (config_->compare_laplacian &&
          std::signbit(query_kp.response) != map_keypoint_info.laplacian_bit)
        continue;

      // For openfabmap we used a euclidean norm cluster size of 0.54 (extended).
      // They recommend 0.6 for surf extended (128 bytes), and 0.45 for basic (64 bytes).
      // We use surf basic (64 bytes).
      // Which is 0.45 = ||a-b|| = sqrt(2-2*a.b), so a.b = 1-0.5*0.45^2 = 0.9, or 0.1 when 1-cosine_dist.
      // however, since we check the octave, and lighting changes, could probably up that a bit.

      // if the pre-checks pass, check descriptor distance
      typedef vision::ASRLFeatureMatcher FeatureMatcher;
      double distance = FeatureMatcher::distance(query_descriptor, map_descriptor,
                                                 query_channel_lm.feat_type);

      // we have a match!!
      if (distance <= config_->cluster_size) {
        construct.obs2word[query_lm_id] = map_vocab_lm_id;
        ++construct.bow[map_vocab_lm_id];
        channel_matched[query_id].worded = true;
        ++total_matched;
      }
    }
  }

  return total_matched;
}

// ^^^ MATCHING AGAINST THE LOCAL MULTI-EXPERIENCE VOCABULARY ^^^ //

// vvv CLUSTERING FOR NEW VOCABULARY WORDS vvv //

/// This function copies a subset of features into a new container
/// @returns the copied features
vision::Features maskFeatures(
    const vision::Features &from,  ///< the source for the features
    std::vector<unsigned> indices   ///< the indices to copy
) {
  vision::Features to;
  unsigned size = indices.size();

  // Initialize and allocate
  to.descriptors = cv::Mat(0, from.descriptors.cols, from.descriptors.type());
  to.descriptors.reserve(size);
  to.feat_infos.reserve(size);
  to.feat_type = from.feat_type;
  to.keypoints.reserve(size);
  to.name = from.name;

  // Copy
  for (unsigned index : indices) {
    if (index < (unsigned) from.descriptors.rows)
      to.descriptors.push_back(from.descriptors.row(index));
    if (index < from.feat_infos.size())
      to.feat_infos.push_back(from.feat_infos[index]);
    if (index < from.keypoints.size())
      to.keypoints.push_back(from.keypoints[index]);
  }
  if ((unsigned) indices.size() != (unsigned) to.descriptors.rows ||
      (unsigned) indices.size() != (unsigned) to.feat_infos.size() ||
      (unsigned) indices.size() != (unsigned) to.keypoints.size()) {
    throw std::runtime_error("feature mask was unsuccessful :(");
  }
  return to;
}

void
MelRecognitionModule::clusterUnmatchedObs(
    const RigLandmarkConstPtrs &query_suite_lm,
    BowConstruct *construct_ptr,
    const std::shared_ptr<const Graph> &graph
) {

  // Check input
  if (!construct_ptr) return;
  BowConstruct &construct = *construct_ptr;

  // Clustering the live vertex
  auto persistent = messages::copyPersistentId(graph->toPersistent(construct.vid));

  // Loop over all unmatched landmarks, clustering them and adding them to the vocabulary
  vision::SuiteFeatures &suite_vocabulary2 = construct.vocabulary;
  SuiteMask &suite_matched = construct.matched_landmarks;
  vision::BowDescriptor &bow = construct.bow;
  for (uint32_t rig_id = 0; rig_id < query_suite_lm.size(); ++rig_id) {
    // refs
    const vision::RigLandmarks &rig_landmarks = *query_suite_lm[rig_id];
    RigMask &rig_matched = suite_matched[rig_id];

    // vocabulary allocation
    suite_vocabulary2.emplace_back();
    vision::RigFeatures &rig_vocabulary2 = suite_vocabulary2.back();
    rig_vocabulary2.name = query_suite_lm[rig_id]->name;

    for (uint32_t channel_id = 0; channel_id < rig_landmarks.channels.size(); ++channel_id) {
      // refs
      const vision::ChannelLandmarks &channel_landmarks = rig_landmarks.channels[channel_id];
      ChannelMask &channel_matched = rig_matched[channel_id];

      // vocabulary allocation
      rig_vocabulary2.channels.emplace_back();
      vision::ChannelFeatures &channel_vocabulary2 = rig_vocabulary2.channels.back();
      channel_vocabulary2.cameras.emplace_back();
      vision::Features &camera_vocabulary2 = channel_vocabulary2.cameras.back();
      channel_vocabulary2.name = channel_landmarks.name;

      // Make sure we've got stuff here
      if (!channel_landmarks.appearance.descriptors.rows)
        continue;
      const vision::Features &channel_features = channel_landmarks.appearance;
      const cv::Mat &channel_descriptors = channel_features.descriptors;

      // *** This is where we actually start doing things :) *** //

      // How many tracked, unworded landmarks do we have?
      std::vector<unsigned> channel_unmatched;
      channel_unmatched.reserve(channel_matched.size());
      for (unsigned i = 0; i < channel_matched.size(); ++i) {
        if (channel_matched[i].tracked == true && channel_matched[i].worded == false)
          channel_unmatched.emplace_back(i);
      }

      // Handle the case where they all matched
      if (channel_unmatched.empty()) {
        if (config_->verbose)
          LOG(INFO) << "Clustered 0 clusters from "
                    << channel_unmatched.size() << " unmatched out of "
                    << channel_matched.size() << " descriptors.";
        continue;
      }

      // Initialize the consolidated descriptors (to cluster all at once)
      cv::Mat unmatched_descriptors(0, channel_descriptors.cols, channel_descriptors.type());
      unmatched_descriptors.reserve(channel_unmatched.size());
      // Copy unmatched descriptors into a consolidated cv::Mat for clustering
      for (const auto &i : channel_unmatched) {
        unmatched_descriptors.push_back(channel_descriptors.row(i));
      }

      // Cluster the unmatched features
      vision::IncrementalBOWTrainer trainer(config_->cluster_size, true);
      std::vector<unsigned> closest_center_uids;
      std::vector<unsigned> center_uids
          = trainer.clusterFirstByIndex(unmatched_descriptors,
                                        &closest_center_uids);

      // convert the cluster center unmatched-ids into a raw ids to the candidate landmarks
      std::vector<unsigned> vocab_mask;
      vocab_mask.reserve(center_uids.size());
      for (unsigned center_uid : center_uids) vocab_mask.push_back(channel_unmatched[center_uid]);
      // mask only the cluster centers from the candidate landmarks, these are our vocabulary
      try {
        camera_vocabulary2 = maskFeatures(channel_features, vocab_mask);
      } catch (const std::runtime_error &) {
        LOG(ERROR) << "exp_recog failed to mask vocab features! ";
        continue;
      }

      // convert the closest center unmatched-ids into vocabulary ids to the new vocab features
      std::vector<unsigned> uid2word(closest_center_uids.size(), unsigned(-1));
      for (unsigned vocab_id = 0; vocab_id < center_uids.size(); ++vocab_id) {
        unsigned center_uid = center_uids[vocab_id];
        uid2word[center_uid] = vocab_id;
      }

      // create the bow descriptor, and record the mapping for future tracks
      for (unsigned uid = 0; uid < closest_center_uids.size(); ++uid) {
        // 1. Record this word in the BoW descriptor
        // unmatched-id to the closest cluster center unmatched-id
        unsigned center_uid = closest_center_uids[uid];
        // cluster center unmatched-id to word id
        unsigned word_id = uid2word[center_uid];
        // word id to landmark id
        vision::LandmarkId word_lm_id(persistent, rig_id, channel_id, word_id, 0);
        // record the word in the BoW descriptor
        ++bow[word_lm_id];
        // 2. Cache this assignment for the next frame, so we can follow vo tracks
        // unmatched id to raw candidate landmark Id
        unsigned candidate_id = channel_unmatched[uid];
        // candidate id to a full landmark id (include rig, channel, etc)
        vision::LandmarkId candidate_lm_id(persistent, rig_id, channel_id, candidate_id, 0);
        // record the word in the map for following vo tracks next time
        construct.obs2word[candidate_lm_id] = word_lm_id;
      }
      LOG_IF(config_->verbose, INFO)
        << "Clustered " << center_uids.size() << " clusters from "
        << channel_unmatched.size() << " unworded out of "
        << channel_matched.size() << " descriptors.";
    }
  }
}

const vision::BowDescriptor &
MelRecognitionModule::slidingWindowify(const vision::BowDescriptor &live_bow) {
  // Initialize it if empty
  if (!window_bow_.get()) {
    window_bow_ = std::make_shared<SlidingWindowBow>(config_->temporal_depth);
  }

  // Append to it
  window_bow_->push(live_bow);

  // Get the windowed version back
  return window_bow_->bow();
}

// ^^^ CLUSTERING FOR NEW VOCABULARY WORDS ^^^ //

vision::BowDescriptor recallBow(const Vertex::Ptr &v) {
  // Get the BoW descriptor from the map vertex and convert to c++
  v->load("bagofwords");
  const auto &bow_msg
      = v->retrieveKeyframeData<vtr_messages::msg::BowDescriptor>("bagofwords");

  // Only complain if it's not the first vertex, which never has one
  if (!bow_msg) {                 //todo (Ben): gonna have a rosbag2 problem here as well
    LOG_IF(v->id().minorId() != 0, DEBUG)
      << "Could not find the bagofwords stream for vertex " << v->id();
    return vision::BowDescriptor();
  }

  return messages::copyBowDescriptor(*bow_msg);
}

ExperienceDifferences
MelRecognitionModule::recognizeExperience(
    const vision::BowDescriptor &query_bow,
    uint32_t live_run_id,
    const MapVertices &map_vertices) const {

  ExperienceDifferences differences;
  // Since we searched for map vertices out from the root spatial neighbours,
  // we can just use the bow descriptor from the first vertex in the list for each experience
  for (const auto &map_itr : map_vertices) {
    uint32_t run_id = map_itr.first;
    if (run_id == live_run_id) continue; // Don't need to match against the live run :P

    // Accumulate the bow over the local window (or just use the first)
    std::list<Vertex::Ptr> window_vertices = map_itr.second;
    SlidingWindowBow map_window_bow(window_vertices.size());
    for (const Vertex::Ptr &map_vertex : window_vertices) {
      // Add this vertex's bow descriptor to the window
      vision::BowDescriptor &&vertex_bow = recallBow(map_vertex);
      map_window_bow.push(vertex_bow);
      // If not sliding window, wait until we have the first valid bow
      if (!config_->sliding_window && !map_window_bow.bow().empty()) {
        break;
      }
    }
    vision::BowDescriptor map_bow = map_window_bow.bow();

    // Calculate the cosine distance between
    float distance = SparseBow::distance(query_bow, map_bow);
    differences[run_id] = distance;

    if (config_->verbose)
      LOG(INFO) << "Run " << run_id << ", vertex " << run_id
                << " had a cosine distance of " << distance;
  }

  return differences;
}

RunIdSet
MelRecognitionModule::filterExperience(
    const ExperienceDifferences &differences) const {

  // Lambda cost, prefers lower cosine distances,
  // with the exception of always preferring the privileged experience
  // Must answer the question "is a better than b?"
  auto compare_differences = [](const ExperienceDifference &a, const ExperienceDifference &b) {
    //if (a.first == 0) return true; // a < b
    //if (b.first == 0) return false; // b < a
    return a.second < b.second;
  };

  // The number of allowed experiences that will be in the mask
  unsigned output_size = std::min<unsigned>(config_->num_desired_experiences,
                                            differences.size());
  std::vector<ExperienceDifference> least_to_most_different(output_size);

  // Sort the top 'n' experiences
  std::partial_sort_copy(differences.rbegin(), differences.rend(), //< reverse so it prefers later if tied
                         least_to_most_different.begin(), least_to_most_different.end(),
                         compare_differences);

  // Extract the run ids from the top experiences and store them in
  // the final mask we will use for localization
  RunIdSet least_different;
  auto extract_run_id = [](const ExperienceDifference &s) { return s.first; };
  std::transform(least_to_most_different.begin(), least_to_most_different.end(),
                 std::inserter(least_different, least_different.begin()), extract_run_id);

  // Verbose message about the mask
  if (config_->verbose) {
    std::stringstream log_ss;
    log_ss << "masked: ";
    for (const auto &exp : least_different)
      log_ss << exp << " ";
    LOG(INFO) << log_ss.str();
  }

  return least_different;
}

void MelRecognitionModule::recordResults(
    const Vertex::Ptr &query_vertex,
    double run_time,
    const ExperienceDifferences &differences,
    const RunIdSet &recommended) {
  // Convert the BoW descriptor to a message
  vtr_messages::msg::ExpRecogStatus &status_msg = status_msg_to_save_;
  status_msg = vtr_messages::msg::ExpRecogStatus();

  // The keyframe time
  status_msg.keyframe_time = query_vertex->keyFrameTime();
  // The query id
  status_msg.set__query_id(query_vertex->id());
  // The algorithm
  status_msg.set__algorithm(vtr_messages::msg::ExpRecogStatus::ALGORITHM_BAGOFWORDS);
  // Whether we're running online
  status_msg.set__in_the_loop(config_->in_the_loop);
  // The bag-of-words cosine distances for each run
  for (const ExperienceDifference &difference : differences) {
    vtr_messages::msg::RunToCosineDistance dist_msg;
    dist_msg.set__run_id(difference.first);
    dist_msg.set__cosine_distance(difference.second);
    status_msg.cosine_distances.push_back(dist_msg);
  }
  // The recommended runs for localization
  for (uint32_t rid : recommended)
    status_msg.recommended_ids.push_back(rid);
  // The computation time
  status_msg.set__computation_time_ms(run_time);
}

// vvv SAVING THINGS TO THE GRAPH vvv //

void MelRecognitionModule::updateGraph(
    QueryCache &qdata,
    MapCache &mdata,
    const Graph::Ptr &graph,
    VertexId vid
) {
  // Check that our stored data matches the request
  if (!constructed_bow_to_save_.get() ||
      constructed_bow_to_save_->vid != vid)
    return;

  const BowConstruct &construct = *constructed_bow_to_save_;

  LOG_IF(config_->verbose, INFO)
    << "exp_recog saving to " << vid;

  // Get the run and vertex
  const Vertex::Ptr &vertex = graph->at(vid);
  const Graph::RunIdType rid = vid.majorId();

  // Convert the vocabulary to a message
  const vision::SuiteFeatures &vocabulary2 = construct.vocabulary;
  for (const vision::RigFeatures &rig_vocabulary2 : vocabulary2) {
    const std::string &rig_name = rig_vocabulary2.name;
    vtr_messages::msg::RigFeatures rc_rig_vocabulary2
        = messages::copyFeatures(rig_vocabulary2);

    // Save to the graph
    std::string vocab_stream = "vocabulary";
    if (!graph->hasVertexStream(rid, vocab_stream)) {
      graph->registerVertexStream<vtr_messages::msg::RigFeatures>(rid, vocab_stream, true);
    }
    vertex->insert(vocab_stream, rc_rig_vocabulary2, vertex->keyFrameTime());

    // Status message
    if (config_->verbose) {
      unsigned vocab_size = 0;
      for (const vision::ChannelFeatures &channel_vocabulary2 : rig_vocabulary2.channels)
        vocab_size += channel_vocabulary2.cameras.empty() ? 0 : channel_vocabulary2.cameras[0].descriptors.rows;
      LOG(INFO) << "Saved vocabulary of size: " << vocab_size << " for rig: " << rig_name;
    }
  }

  // Convert the BoW descriptor to a message
  const vision::BowDescriptor &bow = construct.bow;
  vtr_messages::msg::BowDescriptor rc_bow
      = messages::copyBowDescriptor(bow);

  // Save to the graph
  std::string bow_stream = "bagofwords";
  if (!graph->hasVertexStream(rid, bow_stream)) {
    graph->registerVertexStream<vtr_messages::msg::BowDescriptor>(rid, bow_stream, true);
  }
  vertex->insert(bow_stream, rc_bow, vertex->keyFrameTime());

  // Status message
  if (config_->verbose)
    LOG(INFO) << "Saved bow of size: " << bow.size();

  // Save the status/results message
  if (status_msg_to_save_.query_id == vid) {
    RunId rid = vertex->id().majorId();
    std::string results_stream = "experience_picker";
    if (!graph->hasVertexStream(rid, results_stream)) {
      graph->registerVertexStream<vtr_messages::msg::ExpRecogStatus>(rid, results_stream, true);
    }
    vertex->insert(results_stream, status_msg_to_save_, vertex->keyFrameTime());
  }
}

// ^^^ SAVING THINGS TO THE GRAPH ^^^ //

}
}
