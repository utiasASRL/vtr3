#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_navigation/pipelines/branch_pipeline.hpp>

namespace vtr {
namespace navigation {

void BranchPipeline::convertData(QueryCachePtr q_data, MapCachePtr m_data) {
  auto converter = tactic->getDataConverter();
  converter->run(*q_data, *m_data, tactic->poseGraph());
}

auto BranchPipeline::processData(QueryCachePtr q_data, MapCachePtr m_data,
                                 bool first_frame) -> KeyframeRequest {
  // If it's the first frame, just add the vertex and update the graph
  if (first_frame) {
    if (q_data->rig_features.is_valid()) {
      // if there are rig feature, that means we should make a keyframe
      return KeyframeRequest::YES;
    } else {
      std::string msg{"rig features required when processing the first frame."};
      LOG(ERROR) << msg;
      throw std::runtime_error{msg};
      /// \note some modules still require running even if there isn't image
      /// data, run those modules here if needed
      // if there aren't rig features, don't make a keyframe from this cache
      return KeyframeRequest::NO;
    }
  }

  // Get stuff from the tactic
  auto qvo = tactic->getQuickVo();
  auto pose_graph = tactic->poseGraph();

  // We perform VO against the last keyframe (current vertex)
  q_data->live_id = tactic->currentVertexID();

  // Update graph with converter stuff
  // TODO: find a better place for this...
  tactic->getDataConverter()->updateGraph(*q_data, *m_data, tactic->poseGraph(),
                                          *(q_data->live_id));

  // we need to update the new T_q_m prediction
  auto kf_stamp = pose_graph->at(*q_data->live_id)->keyFrameTime();

  auto T_q_m_est = estimateTransformFromKeyframe(kf_stamp, *q_data->stamp,
                                                 q_data->rig_images.is_valid());

  m_data->T_q_m_prior = T_q_m_est;

#if false  // None of these seems to be useful, maybe we can remove them all
  // add the homography prior if it is valid
  if (candidate_m_data != nullptr && candidate_m_data->H_q_m.is_valid()) {
    m_data->H_q_m_prior = *candidate_m_data->H_q_m;
  }

  // add the previous T_q_m if it is valid
  if (candidate_m_data != nullptr && candidate_m_data->T_q_m.is_valid()) {
    m_data->T_q_m_prev = *candidate_m_data->T_q_m;
  }

  // add the previous timestamp if it is valid
  if (candidate_q_data != nullptr && candidate_q_data->stamp.is_valid()) {
    m_data->stamp_prev = *candidate_q_data->stamp;
  }

  // add the previous trajectory if it is valid
  if (candidate_q_data != nullptr && candidate_q_data->trajectory.is_valid()) {
    m_data->trajectory_prev = *candidate_q_data->trajectory;
  }

  // add the homography prior if it is valid
  if (candidate_m_data != nullptr &&
      candidate_m_data->plane_coefficients.is_valid()) {
    m_data->plane_coefficients = *candidate_m_data->plane_coefficients;
  }
#endif
  // Run quick vo
  qvo->run(*q_data, *m_data, tactic->poseGraph());

  // If it failed, revert whatever garbage is in T_q_m to the initial prior
  // estimate
  if (*m_data->success == false) {
    LOG(ERROR) << "VO FAILED, Reverting to trajectory estimate";
    m_data->T_q_m = T_q_m_est;
  }

  // if we have a non-failed frame, keep a copy as a keyframe candidate
  // check the vertex creation flag
  if (*(q_data->new_vertex_flag) == DO_NOTHING) {
    return KeyframeRequest::NO;  // do what it says on the tin
  } else if (*(q_data->new_vertex_flag) != FAILURE) {
    // keep the candidate caches
    candidate_q_data = q_data;
    candidate_m_data = m_data;

    // keep a pointer to the trajectory
    trajectory_ = candidate_q_data->trajectory.ptr();
    trajectory_time_point_ = common::timing::toChrono(*candidate_q_data->stamp);

    // Only update the robot if we don't have a chain (pure branching mode)
    // TODO: Better way to separate this from MetricLocalization?
    if (tactic->getLocalizationChain().sequence().size() == 0)
      tactic->updatePersistentLocalization(*q_data->live_id, *(m_data->T_q_m));

  } else {
    LOG(ERROR) << "VO KEYFRAME FAILURE!" << std::endl;
  }

  // Should we create a keyframe?
  if (*(q_data->new_vertex_flag) == CREATE_CANDIDATE)
    return KeyframeRequest::NO;

  return KeyframeRequest::YES;
}

#if 0
void BranchPipeline::computeT_0_q(QueryCachePtr q_data, MapCachePtr m_data) {

  // sanity check
  if(q_data->live_id.is_valid() == false ||
      m_data->T_q_m.is_valid() == false) {
    return;
  }

  auto T_l_q = lgmath::se3::Transformation();
  if (m_data->T_q_m.is_valid() && *m_data->map_id == *q_data->live_id) {
    T_l_q = m_data->T_q_m->inverse();
  }
  asrl::common::timing::SimpleTimer timer;

  auto trunk_v_id = *q_data->live_id;
  auto T_trunk_q = T_l_q;
  if (m_data->localization_chain.is_valid() && m_data->localization_chain->isLocalized()) {
    trunk_v_id = m_data->localization_chain->trunkVertexId();
    T_trunk_q = m_data->localization_chain->T_leaf_trunk().inverse();
  }

  // Reset T_0_trunk if we're at a new trunk.
  if (T_0_trunk_cache_ != nullptr &&
      T_0_trunk_cache_->first != trunk_v_id) {
    T_0_trunk_cache_.reset();
  }

  if (T_0_trunk_cache_ == nullptr) {
    try {
      T_0_trunk_cache_ = std::make_shared<std::pair<VertexId, lgmath::se3::Transformation>>();
      T_0_trunk_cache_->first = trunk_v_id;
      auto evaluator = std::make_shared<PrivilegedEvaluator>();
      evaluator->setGraph(tactic->poseGraph().get());
      auto subgraph = tactic->poseGraph()->getSubgraph(VertexId(0,0), evaluator);
      auto path_0_trunk = subgraph->breadthFirstSearch(trunk_v_id, VertexId(0,0));
      T_0_trunk_cache_->second = pose_graph::Eval::ComposeTfAccumulator(path_0_trunk->begin(VertexId(0,0)), path_0_trunk->end(), lgmath::se3::Transformation());
    }
    catch (...) {
      return;
    }
  }

  // Compound to get T_0_q.
  q_data->T_0_q = T_0_trunk_cache_->second * T_trunk_q;
}
#endif

void BranchPipeline::makeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                                  bool first_frame) {
  auto qvo = tactic->getQuickVo();
  auto pose_graph = tactic->poseGraph();

  if (first_frame) {
    // id container
    VertexId live_id;

    // check that we have features to process, and therefore this is the 'true'
    // first frame
    if (q_data->rig_features.is_valid()) {
      LOG(INFO) << "Creating the first keyframe of this run.";

      // insert the first vertex
      live_id = addDanglingVertex(*q_data);

      // update the graph with new landmarks.
      qvo->updateGraph(*q_data, *m_data, pose_graph, live_id);

    } else {
      // otherwise, just return
      return;
    }

    // If we are in Branch mode (no chain), also localize against the persistent
    // localization
    if (tactic->getLocalizationChain().sequence().size() == 0) {
      auto& loc = tactic->persistentLoc();

      if (loc.v.isSet()) {
        q_data->live_id = live_id;

        MapCachePtr loc_data(new MapCache);
        loc_data->map_id = loc.v;
        loc_data->T_q_m_prior = loc.T;

        // the map is initialized in this configuration
        loc_data->map_status = MAP_INITIALIZED;

        tactic->getLocalizer()->run(*q_data, *loc_data, pose_graph);
        LOG(INFO) << "Branching from existing experience: " << loc.v << " --> "
                  << live_id;

        if (!(*loc_data->steam_failure) && *loc_data->success) {
          if (live_id == loc.v) {
            LOG(WARNING) << "Attempted to add edge from vertex to itself: "
                         << live_id << "<->" << loc.v;
          } else {
            LOG(INFO) << "Adding new branch with offset: "
                      << (*loc_data->T_q_m).inverse().vec().transpose();
            (void)pose_graph->addEdge(live_id, loc.v,
                                      (*loc_data->T_q_m).inverse(),
                                      pose_graph::Spatial, true);
          }
        } else {
          LOG(WARNING) << "[BranchPipeline] Couldn't localize, so we are "
                          "branching with just the prior localization!";
          (void)pose_graph->addEdge(live_id, loc.v, loc.T.inverse(),
                                    pose_graph::Spatial, true);
        }
      } else {
        // new map without previous runs.
        LOG(INFO) << "Added the first keyframe to a NEW map.";
      }
    }

  } else {
    // check if we have valid candidate data
    if (candidate_q_data != nullptr && candidate_m_data != nullptr) {
      // make a keyframe from either our new or a recent candidate
      makeKeyframeFromCandidate();

      // we no longer have a valid candidate set of data
      candidate_q_data = nullptr;
      candidate_m_data = nullptr;

      // the map is now definitely initialized
      m_data->map_status = MAP_INITIALIZED;

      // if there was a failure (and we don't just want to create a new vertex
      // from the current), try and recompute the current frame based on the
      // just added candidate. Otherwise, no need to try reprocessing
      //      if (*(q_data->new_vertex_flag) == FAILURE) {
      //        LOG(INFO) << "last frame was not a keyframe, using that as a
      //        keyframe, bailing"; return;
      //      }
    } else {
      LOG(INFO) << "Forcing keyframe, last frame was also a keyframe";
      // This is bad, we just processed the previous frame as a vertex and can't
      // match to it with the current one. Probably a hardware frame drop. We
      // don't have a valid candidate, so we need to query the trajectory to get
      // an estimated frame and dump the current data

      forceKeyframe(q_data, m_data);

      // If we are in a mode that uses the chain, then update the chain.
      if (!tactic->getLocalizationChain().sequence().empty()) {
        tactic->updateLocalization(q_data, m_data);
        m_data->map_id = tactic->closestVertexID();
        m_data->T_q_m_prior = tactic->getLocalizationChain().T_leaf_trunk();
        q_data->live_id = tactic->currentVertexID();
      }
    }
  }

  // Only update the robot if we don't have a chain (pure branching mode)
  // TODO: Better way to separate this from MetricLocalization?
  if (tactic->getLocalizationChain().sequence().size() == 0) {
    tactic->updatePersistentLocalization(tactic->currentVertexID(),
                                         EdgeTransform(true));
  }
}

void BranchPipeline::processKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                                     bool first_frame) {
  if (first_frame) return;

  auto rvo = tactic->getRefinedVo();
  // run refinement on the candidate
  q_data->live_id = tactic->currentVertexID();

  rvo->run(*q_data, *m_data, tactic->poseGraph());
  rvo->updateGraph(*q_data, *m_data, tactic->poseGraph(), *q_data->live_id);
}

void BranchPipeline::makeKeyframeFromCandidate() {
  // Get stuff from the tactic
  auto qvo = tactic->getQuickVo();
  auto pose_graph = tactic->poseGraph();

  if (candidate_q_data == nullptr || candidate_m_data == nullptr ||
      (pose_graph->at(tactic->currentVertexID())
           ->keyFrameTime()
           .nanoseconds_since_epoch ==
       candidate_q_data->stamp->nanoseconds_since_epoch)) {
    LOG(INFO) << "Not adding a keyframe because we just added one";
    return;
  }

  // Add the candidate's vertex and update the pose graph
  if ((*candidate_m_data->T_q_m).covarianceSet() == false) {
    LOG(ERROR) << "Attempting to add an edge with no covariance!!";
  }
  auto live_id =
      addConnectedVertex(*candidate_q_data, *(candidate_m_data->T_q_m));
  LOG(INFO) << "Added edge at " << live_id;
  qvo->updateGraph(*candidate_q_data, *candidate_m_data, pose_graph, live_id);

  // Update the current vertex.
  candidate_q_data->live_id = tactic->currentVertexID();
}

EdgeTransform BranchPipeline::estimateTransformFromKeyframe(
    const vtr_messages::msg::TimeStamp& kf_stamp,
    const vtr_messages::msg::TimeStamp& curr_stamp, bool check_expiry) {
  EdgeTransform T_q_m;
  // The elapsed time since the last keyframe
  auto curr_time_point = common::timing::toChrono(curr_stamp);
  auto dt_duration = curr_time_point - common::timing::toChrono(kf_stamp);
  double dt = std::chrono::duration<double>(dt_duration).count();

  // Make sure the trajectory is current
  if (check_expiry && trajectory_) {
    auto traj_dt_duration = curr_time_point - trajectory_time_point_;
    double traj_dt = std::chrono::duration<double>(traj_dt_duration).count();
    if (traj_dt > tactic->config().extrapolate_timeout) {
      LOG(WARNING) << "The trajectory expired after " << traj_dt
                   << " s for estimating the transform from keyframe at "
                   << common::timing::toIsoString(trajectory_time_point_)
                   << " to " << common::timing::toIsoString(curr_time_point);
      trajectory_.reset();
    }
  }

  /// \todo (yuchen) (old) CHECK THE EXTRAPOLATION FLAG

  // we need to update the new T_q_m prediction
  Eigen::Matrix<double, 6, 6> cov =
      Eigen::Matrix<double, 6, 6>::Identity() * pow(dt, 2.0);
  // scale the rotational uncertainty to be one order of magnitude lower than
  // the translational uncertainty.
  cov.block(3, 3, 3, 3) /= 10;
  if (trajectory_ != nullptr) {
    // Query the saved trajectory estimator we have with the candidate frame
    // time
    auto candidate_time =
        steam::Time(static_cast<int64_t>(kf_stamp.nanoseconds_since_epoch));
    auto candidate_eval = trajectory_->getInterpPoseEval(candidate_time);
    // Query the saved trajectory estimator we have with the current frame time
    auto query_time =
        steam::Time(static_cast<int64_t>(curr_stamp.nanoseconds_since_epoch));
    auto curr_eval = trajectory_->getInterpPoseEval(query_time);

    // find the transform between the candidate and current in the vehicle frame
    T_q_m = candidate_eval->evaluate().inverse() * curr_eval->evaluate();
    // give it back to the caller, TODO: (old) We need to get the covariance out
    // of the trajectory.

    // This ugliness of setting the time is because we don't have a reliable and
    // tested way of predicting the covariance. This is used by the stereo
    // matcher to decide how tight it should set its pixel search
    T_q_m.setCovariance(cov);

    LOG(DEBUG) << "Estimated T_q_m (based on keyframe) from steam trajectory.";
  } else {
    // since we don't have a trajectory, we can't accurately estimate T_q_m
    T_q_m.setCovariance(2 * 2 * cov);

    LOG(DEBUG) << "Estimated T_q_m is identity with high covariance.";
  }
  return T_q_m;
}

#if 0
void BranchPipeline::reprocessData(QueryCachePtr q_data, MapCachePtr m_data,
                                   bool first_frame) {
  // clear out the added data to try again
  q_data->live_id.clear();
  q_data->trajectory.clear();
  q_data->new_vertex_flag.clear();
  m_data->map_id.clear();
  m_data->map_landmarks.clear();
  m_data->raw_matches.clear();
  m_data->ransac_matches.clear();
  m_data->success.clear();
  m_data->landmark_map.clear();
  m_data->migrated_points.clear();
  m_data->migrated_covariance.clear();
  m_data->landmark_offset_map.clear();
  m_data->pose_map.clear();
  // don't clear everything though, we still need features and landmarks in the
  // query cache

  // now try again with the current data
  BranchPipeline::processData(q_data, m_data, first_frame);
  return;
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief force add a Keyframe to the graph because the current data has failed
/// a vertex creation test and there are not enough matches to generate a
/// transform estimate. Generate a transform estimate from a prior trajectory
/// (if one is available) and force it into the graph
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BranchPipeline::forceKeyframe(QueryCachePtr q_data, MapCachePtr m_data) {
  // Get stuff from the tactic
  auto qvo = tactic->getQuickVo();
  auto pose_graph = tactic->poseGraph();

  // clear out the match data in preparation for putting the vertex in the graph
  m_data->raw_matches.clear();
  m_data->ransac_matches.clear();
  m_data->triangulated_matches.clear();

  // Add the candidate's vertex and update the pose graph
  if ((*m_data->T_q_m).covarianceSet() == false) {
    LOG(ERROR) << __func__ << " Attempting to add an edge with no covariance!!";
  }

  // log force keyframe error
  auto now = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = now - last_forced_kf_printed_;
  if (diff.count() > 1.0) {
    LOG(ERROR) << "++++++++++++++++++++++++++ FORCING KEYFRAME "
                  "+++++++++++++++++++++++++++++++++";
    // TODO: check to make sure this is not crazy???
    // LOG(ERROR) << *m_data->T_q_m;
    // LOG(ERROR) << "++++++++++++++++++++++++++ FORCING KEYFRAME
    // +++++++++++++++++++++++++++++++++";
    LOG(ERROR) << "++++++++++++++++++++++++++ ALSO LOGGED "
               << num_forced_kf_logged_ << " others ++++++++++++++";
    last_forced_kf_printed_ = now;
    num_forced_kf_logged_ = 0;
  } else {
    ++num_forced_kf_logged_;
  }

  // Add the candidate's vertex and update the pose graph
  auto live_id = addConnectedVertex(*q_data, *m_data->T_q_m_prior);
  qvo->updateGraph(*q_data, *m_data, pose_graph, live_id);

  // the map is now definitely NOT initialized
  *m_data->map_status = MAP_NEW;

  // we no longer have valid priors
#if false
  m_data->H_q_m.clear();
  m_data->plane_coefficients.clear();
#endif
  m_data->T_q_m.clear();
  q_data->stamp.clear();
  q_data->trajectory.clear();

  // *don't* run refinement on the candidate
  return;
}

}  // namespace navigation
}  // namespace vtr
