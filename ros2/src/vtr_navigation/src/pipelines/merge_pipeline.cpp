#include <vtr_navigation/pipelines/merge_pipeline.hpp>
#include <vtr_navigation/types.hpp>
#include <vtr_pose_graph/path/pose_cache.hpp>

namespace vtr {
namespace navigation {

void MergePipeline::convertData(QueryCachePtr q_data, MapCachePtr m_data) {
  BranchPipeline::convertData(q_data, m_data);
}

auto MergePipeline::processData(QueryCachePtr q_data, MapCachePtr m_data,
                                bool first_frame) -> KeyframeRequest {
  // Run VO
  KeyframeRequest is_kf =
      BranchPipeline::processData(q_data, m_data, first_frame);

  // check if the map is initialized and a valid T_q_m exists
  if ((m_data->map_status.is_valid() && *m_data->map_status == MAP_NEW) ||
      !(m_data->T_q_m.is_valid())) {
    return is_kf;
  }

  bool has_loc = tactic->getLocalizationChain().isLocalized();
  bool vo_success = *m_data->success.fallback(false);

  // Update our position relative to this run (for the UI/State Machine
  // primarily)
  if (vo_success || first_frame) {
    this->_updateRunLoc(q_data, m_data);
  }

  // Update localization using the recent VO estimate (integrate VO)
  // Note: T_q_m is still set to an estimate with larger covariance on VO
  // failure by the BranchPipeline, so it is safe to use here (except first
  // frame that doesn't have loc).
  if (has_loc) {
    std::lock_guard<std::mutex> lck(chain_update_mutex_);
    tactic->getLocalizationChain().updateVO(*m_data->T_q_m, true);
    this->_updateTrunkLoc();
  }

  framesSinceLoc_ += 1;

  // Also localize as fast as possible, but slow a bit once we have a loc
  if (!has_loc || framesSinceLoc_ >= 4)
    is_kf = std::max(is_kf, KeyframeRequest::IF_AVAILABLE);

  // If we're not making a real keyframe, increase the count (used for forcing
  // keyframes)
  if (is_kf != KeyframeRequest::YES) {
    framesSinceKf_ += 1;
  }
  return (force_keyframe_ ? KeyframeRequest::YES : is_kf);
}

void MergePipeline::makeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                                 bool first_frame) {
  std::lock_guard<std::mutex> lock(chain_update_mutex_);
  bool has_loc = tactic->getLocalizationChain().isLocalized();

  // Should we really make a keyframe, and not just localize
  bool vertex_requested = q_data->new_vertex_flag.is_valid() &&
                          (*(q_data->new_vertex_flag) != CREATE_CANDIDATE &&
                           *(q_data->new_vertex_flag) != DO_NOTHING);
  if (first_frame || force_keyframe_ || vertex_requested) {
    LOG(DEBUG) << "Making a keyframe...";

    BranchPipeline::makeKeyFrame(q_data, m_data, first_frame);
    if (first_frame) {
      tactic->getLocalizationChain().resetTwigAndBranch();
    }
    LOG(DEBUG) << "Leaf to Twig";
    framesSinceKf_ = 0;
    force_keyframe_ = false;
    if (m_data->T_q_m.is_valid()) {
      LOG(DEBUG) << "updating VO";
      // Make sure that T_leaf_petiole is set to this leaf in the chain
      tactic->getLocalizationChain().updateVO(*m_data->T_q_m, true);
      LOG(DEBUG) << "finished updating VO";
    }
    // Tell the localization chain that we've upgraded this leaf to a twig
    // (keyframe/vertex)
    tactic->getLocalizationChain().convertLeafToPetiole(*q_data->live_id,
                                                        first_frame);
    LOG(DEBUG) << "Finished petiole conversion";
    // The leaf and twig are the same
    T_leaf_petiole_initial_ = EdgeTransform(true);
  } else {
    // Just localizing now, we don't need a new keyframe
    LOG(DEBUG) << "Just localizing...";
    T_leaf_petiole_initial_ = *m_data->T_q_m;

    // we no longer have a valid candidate set of data (normally branch does
    // this for us)
    candidate_q_data = nullptr;
    candidate_m_data = nullptr;
  }

  // Double-check that we have a covariance, we've seen some funky stuff
  if (!T_leaf_petiole_initial_.covarianceSet()) {
    LOG(WARNING) << "VO T_q_m has an unset covariance for localization";
    T_leaf_petiole_initial_.setCovariance(
        Eigen::Matrix<double, 6, 6>::Identity());
  }

  // If we have no localization, we're just trying from scratch each time
  const auto &default_loc_cov =
      tactic->config().pipeline_config.default_loc_cov;
  if (!has_loc) {
    // should we use an estimate of the transform?
    if (tactic->config().pipeline_config.use_integrated_loc_prior) {
      auto pose_graph = tactic->poseGraph();
      VertexId vid = tactic->closestVertexID();
      pose_graph::PoseCache<Graph> pose_cache(pose_graph, vid);
      m_data->T_q_m_prior = pose_cache.T_root_query(tactic->currentVertexID());
    } else {
      m_data->T_q_m_prior = EdgeTransform();
      m_data->T_q_m_prior->setCovariance(default_loc_cov);
    }
  } else {
    // Use the localization chain to get the localization prior
    m_data->T_q_m_prior = T_leaf_petiole_initial_ *
                          tactic->getLocalizationChain().T_petiole_trunk();
  }

  // Double-check this covariance, we've seen some funky stuff
  if (!m_data->T_q_m_prior->covarianceSet()) {
    LOG(WARNING) << "Covariance from T_leaf_trunk was unset.";
    m_data->T_q_m_prior->setCovariance(default_loc_cov);
  } else if (m_data->T_q_m_prior->cov()(0, 0) > default_loc_cov(0, 0) ||
             m_data->T_q_m_prior->cov()(1, 1) > default_loc_cov(1, 1) ||
             m_data->T_q_m_prior->cov()(5, 5) > default_loc_cov(5, 5)) {
    LOG(WARNING) << "Shrinking covariance, greater than default: "
                 << m_data->T_q_m_prior->cov().diagonal().transpose();
    m_data->T_q_m_prior->setCovariance(default_loc_cov);
  }

  // This is the vertex we're going to localize against
  m_data->map_id = tactic->closestVertexID();
}

void MergePipeline::processKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                                    bool first_frame) {
  // Are we a real or fake keyframe?
  if (prev_live_id_ != *q_data->live_id) {
    prev_live_id_ = *q_data->live_id;
    BranchPipeline::processKeyFrame(q_data, m_data, first_frame);
  }

  // check that this cache has valid data for keyframe generation
  if (q_data->rig_calibrations.is_valid() == false) {
    return;
  }

  // check if there is anything in the chain path. If not, there's nothing to do
  if (tactic->getLocalizationChain().begin() ==
      tactic->getLocalizationChain().end()) {
    LOG_EVERY_N(10, INFO) << "Not running loc assembly because chain empty";
    return;
  }

  // Get stuff from the tactic
  const auto &loc = tactic->getLocalizer();
  const auto &pose_graph = tactic->poseGraph();
  // bool is_kf = q_data->new_vertex_flag.is_valid() &&
  // *(q_data->new_vertex_flag) != CREATE_CANDIDATE;
  bool has_loc = tactic->getLocalizationChain().isLocalized();

  MapCachePtr loc_data = std::make_shared<MapCache>();

  // the map is initialized in this configuration
  loc_data->map_status = MAP_INITIALIZED;

  // We're localizing against the closest privileged keyframe/vertex
  *loc_data->map_id = *m_data->map_id;
  loc_data->T_q_m_prior.fallback(true);
  const auto &default_loc_cov =
      tactic->config().pipeline_config.default_loc_cov;
  if (first_frame == true) {
    loc_data->T_q_m_prior->setCovariance(default_loc_cov);
  } else if (m_data->T_q_m_prior.is_valid()) {
    loc_data->T_q_m_prior = *m_data->T_q_m_prior;
  }

  // Run the localizer against the closest vertex
  loc->run(*q_data, *loc_data, pose_graph);

  // No updateGraph() for you! Not allowed because the user has to accept this
  // when completing merge
  // loc->updateGraph(*q_data, *loc_data, pose_graph,*q_data->live_id);

  // Only continue if we had a successful localization
  if (*loc_data->steam_failure == true) {
    locSuccesses_ = std::max(0, locSuccesses_ - 1);
    tactic->incrementLocCount(-1);
    LOG(ERROR) << "Steam error; failed localization " << locSuccesses_
               << "/5 (need 3) at "
               << tactic->getLocalizationChain().trunkVertexId();
    return;
  }

  // Default to the prior transform if we cannot localize
  EdgeTransform T_q_m = *loc_data->T_q_m_prior;
  std::lock_guard<std::mutex> lck(chain_update_mutex_);

  unsigned int trunkSeq;
  framesSinceLoc_ = 0;

  if (*loc_data->success) {
    T_q_m = *loc_data->T_q_m;
    locSuccesses_ = std::min(5, locSuccesses_ + 1);
    LOG(DEBUG) << "Localized " << locSuccesses_ << "/5 (need 3) at "
               << tactic->getLocalizationChain().trunkVertexId();

    tactic->incrementLocCount(1);
    tactic->getLocalizationChain().setLocalization(
        T_leaf_petiole_initial_.inverse() * T_q_m, true);

    // The correct trunk sequence is wherever we localized
    trunkSeq = tactic->getLocalizationChain().trunkSequenceId();
  } else {
    locSuccesses_ = std::max(0, locSuccesses_ - 1);
    LOG(INFO) << "Failed localization " << locSuccesses_ << "/5 (need 3) at "
              << tactic->getLocalizationChain().trunkVertexId();

    tactic->incrementLocCount(-1);
    // Move the trunk sequence along, looping around when we hit the end
    trunkSeq = this->_getNextTrunkSeq();
    LOG(DEBUG) << "Moving to sequence: " << trunkSeq;
  }

  // If we have fewer than 3/5 of the last localization attempts, start
  // searching again
  if (locSuccesses_ < 3) {
    if (has_loc && *loc_data->success == false) {
      // If we were previously localized, then this is an error
      LOG(WARNING)
          << "Lost localization to target chain... Drive closer to the path.";
    }

    // Reset the trunk sequence to clear the localization flag
    tactic->getLocalizationChain().resetTrunk(trunkSeq);
  } else {
    this->_updateTrunkLoc();

    auto Tp = tactic->persistentLoc().T;
    LOG(DEBUG) << "Matching: " << tactic->currentVertexID() << " <--> "
               << tactic->closestVertexID()
               << (has_loc ? "" : " for the first time.");
    auto tactic_loc_status = tactic->status().localization_;
    std::stringstream stat_str;
    stat_str << "Localization is: ";
    if (tactic_loc_status == mission_planning::LocalizationStatus::Confident)
      stat_str << "confident! ";
    if (tactic_loc_status ==
        mission_planning::LocalizationStatus::DeadReckoning)
      stat_str << "dead reckoning :(! ";
    if (tactic_loc_status == mission_planning::LocalizationStatus::LOST)
      stat_str << "lost :(((! ";
    auto &Tt = tactic->targetLoc().T;
    double dx = Tt.r_ba_ina()(0), dy = Tt.r_ba_ina()(1), dz = Tt.vec()(3);
    LOG(DEBUG) << "offset: " << dx << ", " << dy << " " << dz;
    if (Tp.covarianceSet())
      stat_str << "var: " << Tp.cov().diagonal().transpose();
    LOG(DEBUG) << stat_str.str();

    if (*loc_data->success) {
      // If the localization transform is set, and the persistent localization
      // covariance is set
      if (T_q_m.covarianceSet() && Tp.covarianceSet()) {
        auto this_loc_status = tactic->tfStatus(T_q_m);
        // AND if the localization is confident, but the persistent localization
        // is not confident, and we have not put down a keyframe in the last
        // 3s...
        if (framesSinceKf_ >= 16 &&
            this_loc_status ==
                mission_planning::LocalizationStatus::Confident &&
            tactic_loc_status !=
                mission_planning::LocalizationStatus::Confident) {
          // Force a keyframe addition on the next frame
          LOG(INFO) << "Forcing a keyframe to improve localization.";
          force_keyframe_ = true;
        }
      }
    }
  }
}

/// Move the trunk vertex when we encounter a localization failure
uint32_t MergePipeline::_getNextTrunkSeq() {
  return (tactic->getLocalizationChain().trunkSequenceId() + 3) %
         uint32_t(tactic->getLocalizationChain().sequence().size() - 1);
}

/// Update the localization with respect to the current run
void MergePipeline::_updateRunLoc(QueryCachePtr, MapCachePtr m_data) {
  tactic->updatePersistentLocalization(tactic->currentVertexID(),
                                       *m_data->T_q_m);
}

/// Update the localization with respect to the privileged chain
void MergePipeline::_updateTrunkLoc() {
  tactic->updateTargetLocalization(
      tactic->getLocalizationChain().trunkVertexId(),
      tactic->getLocalizationChain().T_leaf_trunk());
}

}  // namespace navigation
}  // namespace vtr
