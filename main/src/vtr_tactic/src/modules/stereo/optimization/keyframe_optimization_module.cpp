#include <vtr_tactic/modules/stereo/optimization/keyframe_optimization_module.hpp>

namespace vtr {
namespace tactic {
namespace stereo {

void KeyframeOptimizationModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  SteamModule::configFromROS(node, param_prefix);
  keyframe_config_ = std::make_shared<Config>();
  auto casted_config =
      std::static_pointer_cast<SteamModule::Config>(keyframe_config_);
  *casted_config = *config_;  // copy over base config
  // clang-format off
  keyframe_config_->pose_prior_enable = node->declare_parameter<bool>(param_prefix + ".pose_prior_enable", keyframe_config_->pose_prior_enable);
  keyframe_config_->depth_prior_enable = node->declare_parameter<bool>(param_prefix + ".depth_prior_enable", keyframe_config_->depth_prior_enable);
  keyframe_config_->depth_prior_weight = node->declare_parameter<double>(param_prefix + ".depth_prior_weight", keyframe_config_->depth_prior_weight);
  keyframe_config_->use_migrated_points = node->declare_parameter<bool>(param_prefix + ".use_migrated_points", keyframe_config_->use_migrated_points);
  // clang-format on
}

bool KeyframeOptimizationModule::isLandmarkValid(const Eigen::Vector3d &point) {
  // check depth
  return point(2) > 0.0 && point(2) < keyframe_config_->max_point_depth;
}

std::shared_ptr<steam::OptimizationProblem>
KeyframeOptimizationModule::generateOptimizationProblem(
    QueryCache &qdata, MapCache &mdata,
    const std::shared_ptr<const Graph> &graph) {
  // initialize the steam problem.
  resetProblem(*qdata.T_r_m);

  // Initialize the landmark container
  std::vector<steam::se3::LandmarkStateVar::Ptr> landmarks_ic;

  // get the cache data
  auto &map_landmarks = *qdata.map_landmarks;
  auto &calibrations = *qdata.rig_calibrations;
  auto calibration_itr = calibrations.begin();

  // get the ransac matches
  auto ransac_matches = *qdata.ransac_matches;

  // iterate through the inliers of every rig
  for (uint32_t rig_idx = 0; rig_idx < ransac_matches.size();
       ++rig_idx, ++calibration_itr) {
    // monocular or stereo?
    bool monocular = calibration_itr->intrinsics.size() == 1 ? true : false;

    // get the inliers and calibration for this rig.
    auto &rig_ransac_matches = (*qdata.ransac_matches)[rig_idx];
    auto &calibration = *calibration_itr;

    // Setup camera intrinsics, TODO: This should eventually be different for
    // each rig.
    StereoCalibPtr sharedStereoIntrinsics;
    MonoCalibPtr sharedMonoIntrinsics;

    // setup the calibration
    if (monocular) {
      throw std::runtime_error{"Monocular camera code not ported!"};
#if 0
      sharedMonoIntrinsics = toMonoSteamCalibration(calibration);
#endif
    } else {
      sharedStereoIntrinsics = toStereoSteamCalibration(calibration);
    }

    // Create a transform evaluator for T_q_m, in the camera sensor frame.
    steam::se3::TransformEvaluator::Ptr tf_qs_mv;
    steam::se3::TransformEvaluator::Ptr tf_qs_ms;

    if (config_->use_T_q_m_prior) {
      tf_qs_mv = steam::se3::compose(tf_sensor_vehicle_map_[*(qdata.live_id)],
                                     tf_query_);
      tf_qs_ms = steam::se3::composeInverse(
          tf_qs_mv, tf_sensor_vehicle_map_[*(qdata.map_id)]);
    } else {
      tf_qs_mv = steam::se3::compose(tf_sensor_vehicle_, tf_query_);
      tf_qs_ms = steam::se3::composeInverse(
          tf_qs_mv, tf_sensor_vehicle_map_[*(qdata.live_id)]);
    }

    // iterate through every channel
    for (uint32_t channel_idx = 0;
         channel_idx < rig_ransac_matches.channels.size(); ++channel_idx) {
      auto &channel = rig_ransac_matches.channels[channel_idx];

      // iterate through each match
      for (uint32_t match_idx = 0; match_idx < channel.matches.size();
           ++match_idx) {
        try {  // steam throws?

          // get the match
          const auto &match = channel.matches[match_idx];

          // set up the data for the landmark based on the configuration.
          std::vector<cv::Point> query_kps;
          Eigen::Matrix<double, 3, Eigen::Dynamic> *map_points;
          Eigen::MatrixXd meas_covariance;

          if (keyframe_config_->use_migrated_points == true) {
            // set the map points from the migrated points
            map_points = &(*qdata.migrated_points_3d);

            // check the validity
            bool map_point_valid = qdata.migrated_validity->at(match.first);
            if (!map_point_valid) {
              continue;  // if this landmark isn't valid, skip it
            }

            // convenience accessor
            const auto &channel_obs =
                map_landmarks[rig_idx].observations.channels[channel_idx];

            meas_covariance = Eigen::MatrixXd(channel_obs.cameras.size() * 2,
                                              channel_obs.cameras.size() * 2);
            meas_covariance.setZero();
            // for each camera in the channel, add the observation
            for (uint32_t idx = 0; idx < channel_obs.cameras.size(); idx++) {
              query_kps.push_back(
                  channel_obs.cameras[idx].points[match.second]);
              meas_covariance.block(idx * 2, idx * 2, 2, 2) =
                  channel_obs.cameras[idx].covariances[match.second];
            }

          } else {
            // set the map points from the landmark the migrated points
            map_points =
                &map_landmarks[rig_idx].landmarks.channels[channel_idx].points;

            // check the validity
            bool map_point_valid =
                map_landmarks[rig_idx].landmarks.channels[channel_idx].valid.at(
                    match.first);
            if (!map_point_valid) {
              continue;  // if this landmark isn't valid, skip it
            }

            // convenience accessor
            auto &channel_features =
                (*qdata.rig_features)[rig_idx].channels[channel_idx];

            meas_covariance =
                Eigen::MatrixXd(channel_features.cameras.size() * 2,
                                channel_features.cameras.size() * 2);
            meas_covariance.setZero();
            // for each camera in the channel, add the observation
            for (uint32_t idx = 0; idx < channel_features.cameras.size();
                 idx++) {
              query_kps.push_back(
                  channel_features.cameras[idx].keypoints[match.second].pt);
              meas_covariance.block(idx * 2, idx * 2, 2, 2) =
                  channel_features.cameras[idx]
                      .feat_infos[match.second]
                      .covariance;
            }
          }

          // get the map point
          const auto &map_point = map_points->col(match.first);

          // check that the potential landmark is sane
          if (!isLandmarkValid(map_point)) {
            continue;
          }

          // create and add the landmark
          landmarks_ic.push_back(steam::se3::LandmarkStateVar::Ptr(
              new steam::se3::LandmarkStateVar(map_point)));

          // add the depth prior
          if (keyframe_config_->depth_prior_enable) {
            addDepthCost(landmarks_ic.back());
          }

          // Get landmark reference
          auto &landVar = landmarks_ic.back();

          // lock the landmark
          landVar->setLock(true);

          // set up the mono and stereo noise for each potential type
          steam::BaseNoiseModelX::Ptr noise_mono;
          steam::BaseNoiseModel<4>::Ptr noise_stereo;

          try {
            // If this is with migrated points, then use the dynamic model.
            if (keyframe_config_->use_migrated_points == true) {
              // TODO: Calculate directly instead of in landmark migration.
              auto *migrated_cov = &(*qdata.migrated_covariance);
              const Eigen::Matrix3d &cov = Eigen::Map<Eigen::Matrix3d>(
                  migrated_cov->col(match.first).data());

              // set up the noise for the stereo/mono configurations
              if (monocular) {
                throw std::runtime_error{"Monocular camera code not ported!"};
#if 0
                typedef vtr::steam_extensions::mono::LandmarkNoiseEvaluator
                    NoiseEval;
                auto &landmark_noise = *qdata.mono_landmark_noise.fallback();
                auto noise_eval = boost::make_shared<NoiseEval>(
                    landVar->getValue(), cov, meas_covariance,
                    sharedMonoIntrinsics, tf_qs_ms);
                landmark_noise[match.first] = noise_eval;
                noise_mono.reset(new steam::DynamicNoiseModelX(noise_eval));
#endif
              } else {
                typedef steam::stereo::LandmarkNoiseEvaluator NoiseEval;
                auto &landmark_noise = *qdata.stereo_landmark_noise.fallback();
                auto noise_eval = boost::make_shared<NoiseEval>(
                    landVar->getValue(), cov, meas_covariance,
                    sharedStereoIntrinsics, tf_qs_ms);
                landmark_noise[match.first] = noise_eval;
                noise_stereo.reset(new steam::DynamicNoiseModel<4>(noise_eval));
              }

            } else {
              if (monocular) {
                throw std::runtime_error{"Monocular camera code not ported!"};
#if 0
                noise_mono.reset(new steam::StaticNoiseModelX(meas_covariance));
#endif
              } else {
                noise_stereo.reset(
                    new steam::StaticNoiseModel<4>(meas_covariance));
              }
            }

          } catch (std::invalid_argument &e) {
            LOG(ERROR) << "landmark measurement noise is bad!! " << e.what();
            landmarks_ic.pop_back();
            continue;
          }

          // Construct the measurement vector for the current camera
          Eigen::MatrixXd data(query_kps.size() * 2, 1);
          for (uint32_t idx = 0; idx < query_kps.size(); idx++) {
            data(idx * 2) = query_kps.at(idx).x;
            data(idx * 2 + 1) = query_kps.at(idx).y;
          }

          if (monocular) {
            throw std::runtime_error{"Monocular camera code not ported!"};
#if 0
            // Construct error function for observation to the fixed landmark.
            vtr::steam_extensions::MonoCameraErrorEval::Ptr errorfunc(
                new vtr::steam_extensions::MonoCameraErrorEval(
                    data, sharedMonoIntrinsics, tf_qs_ms, landVar));
            steam::WeightedLeastSqCostTermX::Ptr cost(
                new steam::WeightedLeastSqCostTermX(errorfunc, noise_mono,
                                                    sharedLossFunc_));

            if (!std::isnan(cost->cost())) {
              // add the cost term
              cost_terms_->add(cost);
            }
#endif
          } else {
            // Construct error function for observation to the fixed landmark.
            steam::StereoCameraErrorEval::Ptr errorfunc(
                new steam::StereoCameraErrorEval(data, sharedStereoIntrinsics,
                                                 tf_qs_ms, landVar));
            steam::WeightedLeastSqCostTerm<4, 6>::Ptr cost(
                new steam::WeightedLeastSqCostTerm<4, 6>(
                    errorfunc, noise_stereo, sharedLossFunc_));
            // add the cost term
            cost_terms_->add(cost);
          }

          // steam throws?
        } catch (std::exception &e) {
          LOG(ERROR) << "Error with noise model:\n" << e.what();
          continue;
        }
      }  // end for match
    }    // end for channel
  }      // end for rig

  // Add pose variables
  problem_->addStateVariable(map_pose_);
  problem_->addStateVariable(query_pose_);

  // Add pose prior if requested
  if (keyframe_config_->pose_prior_enable == true) {
    addPosePrior(qdata);
  }

  // Add landmark variables
  for (auto &landmark : landmarks_ic) {
    problem_->addStateVariable(landmark);
  }

  // Add cost terms
  problem_->addCostTerm(cost_terms_);
  problem_->addCostTerm(depth_cost_terms_);

  // Add the trajectory stuff.
  if (config_->trajectory_smoothing) {
    computeTrajectory(qdata, mdata, graph);
  }

  // Go through each rig
  return problem_;
}

void KeyframeOptimizationModule::addPosePrior(QueryCache &qdata) {
  // TODO: Replace with T_leaf_branch from graph?
  EdgeTransform &pose_prior = *qdata.T_r_m_prior;

  steam::BaseNoiseModel<6>::Ptr priorUncertainty;

  /// @brief the loss function associated with observation cost.
  steam::LossFunctionBase::Ptr priorLossFunc;
  priorLossFunc.reset(new steam::L2LossFunc());
  try {
    auto pose_cov = pose_prior.cov();
    priorUncertainty.reset(new steam::StaticNoiseModel<6>(pose_cov));
  } catch (std::invalid_argument &e) {
    priorUncertainty.reset(new steam::StaticNoiseModel<6>(
        Eigen::Matrix<double, 6, 6>::Identity()));
    LOG(ERROR) << "Error on adding pose prior: " << e.what();
  }
  steam::TransformErrorEval::Ptr prior_error_func(
      new steam::TransformErrorEval(pose_prior, tf_query_));
  // Create cost term and add to problem
  steam::WeightedLeastSqCostTerm<6, 6>::Ptr prior_cost(
      new steam::WeightedLeastSqCostTerm<6, 6>(
          prior_error_func, priorUncertainty, priorLossFunc));
  cost_terms_->add(prior_cost);
}

bool KeyframeOptimizationModule::verifyInputData(QueryCache &qdata,
                                                 MapCache &) {
  // sanity check
  if ((qdata.success.is_valid() &&
       *qdata.success == false) /* || *qdata.map_status == MAP_NEW */) {
    return false;
  }

  // if there are no inliers, then abort.
  if (qdata.ransac_matches.is_valid() == false) {
    LOG(WARNING)
        << "KeyframeOptimizationModule::verifyInputData(): Matches is "
           "not set in the map data (no inliers?), this is ok if first frame.";
    // *qdata.T_r_m = lgmath::se3::Transformation();
    return false;
  }

  int inlier_count = 0;

  for (auto &rig : *qdata.ransac_matches) {
    for (auto &channel : rig.channels) {
      inlier_count += channel.matches.size();
    }
  }

  // If there are not enough inliers then abort.
  if (inlier_count < 6 && keyframe_config_->pose_prior_enable == false) {
    LOG(ERROR) << "KeyframeOptimizationModule::verifyInputData(): Insufficient "
                  "number of inliers, Bailing on steam problem!";
    return false;
  }

  // If we dont have an initial condition, then just set identity
  if (qdata.T_r_m.is_valid() == false) {
    *qdata.T_r_m = lgmath::se3::Transformation();
  }
  return true;
}

bool KeyframeOptimizationModule::verifyOutputData(QueryCache &, MapCache &) {
  return true;
}

void KeyframeOptimizationModule::resetProblem(EdgeTransform &T_q_m) {
  // set up the transforms
  map_pose_.reset(
      new steam::se3::TransformStateVar(lgmath::se3::Transformation()));
  map_pose_->setLock(true);  // lock the 'origin' pose
  query_pose_.reset(new steam::se3::TransformStateVar(T_q_m));
  tf_map_.reset(new steam::se3::TransformStateEvaluator(map_pose_));
  tf_query_.reset(new steam::se3::TransformStateEvaluator(query_pose_));

  // Initialize the cost term container
  cost_terms_.reset(new steam::ParallelizedCostTermCollection());

  // Initialize the depth cost term container
  depth_cost_terms_.reset(new steam::ParallelizedCostTermCollection());

  // make the loss functions, TODO: make this configurable.
  sharedDepthLossFunc_.reset(new steam::DcsLossFunc(2.0));
  sharedLossFunc_.reset(new steam::DcsLossFunc(2.0));

  // Initialize the problem.;
  problem_.reset(new steam::OptimizationProblem());
}

void KeyframeOptimizationModule::addDepthCost(
    steam::se3::LandmarkStateVar::Ptr landmark) {
  vtr::steam_extensions::RangeConditioningEval::Ptr errorfunc_range(
      new vtr::steam_extensions::RangeConditioningEval(landmark));
  double depth = landmark->getValue().hnormalized()[2];
  double weight = keyframe_config_->depth_prior_weight / depth;
  steam::BaseNoiseModel<1>::Ptr rangeNoiseModel(new steam::StaticNoiseModel<1>(
      Eigen::Matrix<double, 1, 1>::Identity() * weight));
  steam::WeightedLeastSqCostTerm<1, 3>::Ptr depth_cost;
  depth_cost.reset(new steam::WeightedLeastSqCostTerm<1, 3>(
      errorfunc_range, rangeNoiseModel, sharedDepthLossFunc_));
  depth_cost_terms_->add(depth_cost);
}

void KeyframeOptimizationModule::computeTrajectory(
    QueryCache &qdata, MapCache &, const std::shared_ptr<const Graph> &graph) {
  velocity_map_.clear();

  // reset the trajectory
  trajectory_.reset(
      new steam::se3::SteamTrajInterface(smoothing_factor_information_, true));

  // get the map vertex
  auto map_vertex = graph->at(*qdata.live_id);

  // get the stamps
  auto &query_stamp = *qdata.stamp;
  auto &map_stamp = map_vertex->keyFrameTime();

  // set up a search for the previous keyframes in the graph
  TemporalEvaluator::Ptr tempeval(new TemporalEvaluator());
  tempeval->setGraph((void *)graph.get());

  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  typedef pose_graph::eval::Mask::DirectionFromVertexDirect<Graph>
      DirectionEvaluator;
  auto direval = std::make_shared<DirectionEvaluator>(*qdata.live_id, true);
  direval->setGraph((void *)graph.get());

  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);
  evaluator->setGraph((void *)graph.get());

  // look back five vertices
  int temporal_depth = 5;

  // perform the search and automatically step back one
  auto itr = graph->beginDfs(*qdata.live_id, temporal_depth, evaluator);
  itr++;

  // initialize the compunded transform
  EdgeTransform T_p_m;

  // initialize the timestamp that will be used as
  auto next_stamp = map_vertex->keyFrameTime();

  // Trajectory is of the following form, where the origin = 0 is at m
  // which is the most recent keyframe in the graph.
  //                      local
  //       T_1_0     T_2_1     T_m_2      T_q_m
  //    0---------1---------2---------m----------q
  //  T_0_m     T_1_m     T_2_m       0        T_q_1
  //                      global
  // loop through all the found vertices
  for (; itr != graph->end(); ++itr) {
    // get the stamp of the vertex we're looking at
    auto prev_vertex = graph->at(itr->to());
    auto &prev_stamp = prev_vertex->keyFrameTime();

    // get the transform and compund it
    const auto &T_pp1_p = itr->e()->T();
    T_p_m = T_pp1_p.inverse() * T_p_m;

    // set up a locked global pose for this vertex, with an tf evaluator
    // Note: normally steam would have states T_a_0, T_b_0, ..., where 'a' and
    // 'b' are always sequential in time. So in our case, since our locked '0'
    // frame is in the future, 'a' is actually further from '0' than 'b'.
    auto prev_pose = boost::make_shared<steam::se3::TransformStateVar>(T_p_m);
    prev_pose->setLock(true);
    auto tf_prev =
        boost::make_shared<steam::se3::TransformStateEvaluator>(prev_pose);

    // time difference between next and previous
    int64_t next_prev_dt =
        next_stamp.nanoseconds_since_epoch - prev_stamp.nanoseconds_since_epoch;

    // generate a velocity estimate
    // The velocity is in the body frame, helping you get from 'a' to 'b'.
    // This part can ignore the fact that the transforms above are weird
    // (new to old instead of old to new), and keep doing vel_b_a.
    // we use pp1_p instead of p_pm1 for convenience
    Eigen::Matrix<double, 6, 1> prev_velocity =
        T_pp1_p.vec() / (next_prev_dt / 1e9);

    // TODO nice to have once the code is thread safe
    //    auto proto_velocity =
    //    prev_vertex->retrieveKeyframeData<robochunk::kinematic_msgs::Velocity>("/velocities");
    //    Eigen::Matrix<double,6,1> velocity;
    //    prev_velocity(0,0) = proto_velocity->translational().x();
    //    prev_velocity(1,0) = proto_velocity->translational().y();
    //    prev_velocity(2,0) = proto_velocity->translational().z();
    //    prev_velocity(3,0) = proto_velocity->rotational().x();
    //    prev_velocity(4,0) = proto_velocity->rotational().y();
    //    prev_velocity(5,0) = proto_velocity->rotational().z();
    auto prev_frame_velocity =
        boost::make_shared<steam::VectorSpaceStateVar>(prev_velocity);

    velocity_map_.insert({prev_vertex->id(), prev_frame_velocity});

    // add the velocity to the state variable.
    problem_->addStateVariable(prev_frame_velocity);

    // make a steam time from the timstamp
    steam::Time prev_time(
        static_cast<int64_t>(prev_stamp.nanoseconds_since_epoch));

    // Add the poses to the trajectory
    trajectory_->add(prev_time, tf_prev, prev_frame_velocity);
    next_stamp = prev_stamp;
  }

  // lock the velocity at the begining of the trajectory
  if (velocity_map_.empty() == false) {
    velocity_map_.begin()->second->setLock(true);
  }

  // time difference between query and map
  int64_t query_map_dt =
      query_stamp.nanoseconds_since_epoch - map_stamp.nanoseconds_since_epoch;
  Eigen::Matrix<double, 6, 1> query_velocity =
      query_pose_->getValue().vec() / (query_map_dt / 1e9);
  steam::VectorSpaceStateVar::Ptr map_frame_velocity(
      new steam::VectorSpaceStateVar(query_velocity));
  steam::VectorSpaceStateVar::Ptr query_frame_velocity(
      new steam::VectorSpaceStateVar(query_velocity));

  velocity_map_.insert({*qdata.live_id, map_frame_velocity});
  velocity_map_.insert({VertexId::Invalid(), query_frame_velocity});

  // add the velocities to the state variable.
  problem_->addStateVariable(map_frame_velocity);
  problem_->addStateVariable(query_frame_velocity);

  steam::Time map_time(static_cast<int64_t>(map_stamp.nanoseconds_since_epoch));
  steam::Time query_time(
      static_cast<int64_t>(query_stamp.nanoseconds_since_epoch));

  // Add the poses to the trajectory
  trajectory_->add(map_time, tf_map_, map_frame_velocity);
  trajectory_->add(query_time, tf_query_, query_frame_velocity);

  // Trajectory prior smoothing terms
  trajectory_->appendPriorCostTerms(cost_terms_);
  trajectory_->appendPriorCostTerms(depth_cost_terms_);

  if (config_->velocity_prior) {
    trajectory_->addVelocityPrior(query_time, velocity_prior_,
                                  velocity_prior_cov_);
  }

#if false
  // Clear the trajectory status message and save off the pre-optimized
  // velocities.
  trajectory_status_.Clear();
  for (auto velocity_itr : velocity_map_) {
    auto *ba_pose = trajectory_status_.add_pre_optimization_window();
    auto velocity = velocity_itr.second->getValue();
    auto *proto_velocity = ba_pose->mutable_velocity();
    for (auto idx = 0; idx < 6; ++idx) {
      proto_velocity->add_entries(velocity(idx, 0));
    }
    ba_pose->set_id(velocity_itr.first);
  }
#endif
}

void KeyframeOptimizationModule::updateCaches(QueryCache &qdata, MapCache &) {
  // update our estimate for the transform
  *qdata.T_r_m = query_pose_->getValue();

  // give the query cache a copy of the trajectory estimate
  qdata.trajectory = trajectory_;

  // look up covariance on pose
  if (!config_->disable_solver) {
    auto gn_solver =
        std::dynamic_pointer_cast<steam::GaussNewtonSolverBase>(solver_);
    if (gn_solver != nullptr) {
      if (config_->solver_type == "LevenburgMarquardt" ||
          backup_lm_solver_used_) {
        auto lm_solver =
            std::dynamic_pointer_cast<steam::LevMarqGaussNewtonSolver>(
                gn_solver);
        std::lock_guard<std::mutex> iteration_lock(*qdata.steam_mutex->get());
        try {
          lm_solver->solveCovariances();
        } catch (std::runtime_error &e) {
          LOG(ERROR) << "KeyframeOptimizationModule: Couldn't solve for "
                        "covariance in LM solver!"
                     << std::endl
                     << e.what();
        }
      }
      // now we can safely query the covariance
      auto cov = gn_solver->queryCovariance(query_pose_->getKey());
      (*qdata.T_r_m).setCovariance(cov);
    } else {
      LOG(INFO)
          << "This solver does not derive from The GaussNewtonSolverBase!";
    }

  } else {
    // default to zero covariance, because we have no other information
    qdata.T_r_m->setZeroCovariance();
  }
}
#if false
void KeyframeOptimizationModule::saveTrajectory(
    QueryCache &qdata, MapCache &mdata, const std::shared_ptr<Graph> &graph,
    VertexId id) {
  // if the trajectory is no good, then return early.
  if (trajectory_ == nullptr) return;

  // if we used the backup LM solver, cast that instead of the main one.
  auto gn_solver =
      backup_lm_solver_used_
          ? std::dynamic_pointer_cast<steam::GaussNewtonSolverBase>(
                backup_lm_solver_)
          : std::dynamic_pointer_cast<steam::GaussNewtonSolverBase>(solver_);

  if (gn_solver == nullptr) {
    LOG(ERROR) << "This solver does not derive from The GaussNewtonSolverBase!";
    return;
  }

  // look back five vertices
  int temporal_depth = 5;
  int64_t live_stamp = (*qdata.stamp).nanoseconds_since_epoch;

  // exrapolate 10 evenly spaced points one second into the future.
  int64_t future_stamp = live_stamp + 1e9;
  int64_t interval = 1e9 / 10;
  // save off the data
  for (auto idx = 0; idx < 10; ++idx) {
    auto *ba_pose = trajectory_status_.mutable_optimization_window()->Add();
    ba_pose->set_id(-1);
    ba_pose->set_locked(false);
    ba_pose->set_interpolated(true);
    ba_pose->set_stamp(future_stamp);
    auto curr_eval = trajectory_->getInterpPoseEval(steam::Time(future_stamp));
    EdgeTransform T_q_0 = curr_eval->evaluate();
    //  TODO: reenable when the steam library is updated.
    //    auto cov =
    //    trajectory_->getCovariance(*gn_solver.get(),steam::Time(future_stamp));
    //    T_q_0.setCovariance(cov);
    *ba_pose->mutable_t_q_0() << T_q_0;
    future_stamp -= interval;
  }

  // Compile data for the live pose and save it off.
  auto *ba_pose = trajectory_status_.mutable_optimization_window()->Add();
  auto curr_eval = trajectory_->getInterpPoseEval(steam::Time(live_stamp));
  auto covariance = gn_solver->queryCovariance(query_pose_->getKey());

  ba_pose->set_id(-1);
  ba_pose->set_locked(false);
  ba_pose->set_interpolated(false);
  ba_pose->set_stamp(live_stamp);
  EdgeTransform T_q_0 = curr_eval->evaluate();
  T_q_0.setCovariance(covariance);

  *ba_pose->mutable_t_q_0() << T_q_0;

  // grab the optimized velocity and save it off.
  auto velocity = velocity_map_[VertexId::Invalid()]->getValue();
  auto *proto_velocity = ba_pose->mutable_velocity();
  for (auto idx = 0; idx < 6; ++idx) {
    proto_velocity->add_entries(velocity(idx, 0));
  }

  // get the most recent vertex.
  auto vertex = graph->at(VertexId(*qdata.live_id));
  auto vertex_stamp = vertex->keyFrameTime().nanoseconds_since_epoch;

  // interpolate between the live pose and the vertex pose
  // (query and map in the keyframe opt. problem)
  int64_t time_delta = live_stamp - vertex_stamp;

  for (auto idx = 5; idx >= 0; --idx) {
    int64_t query_time = vertex_stamp + ((time_delta / 5) * idx);
    auto *ba_pose = trajectory_status_.mutable_optimization_window()->Add();
    ba_pose->set_interpolated(true);
    ba_pose->set_stamp(query_time);
    auto curr_eval = trajectory_->getInterpPoseEval(steam::Time(query_time));
    *ba_pose->mutable_t_q_0() << curr_eval->evaluate();
  }

  // set up a search for the previous keyframes in the graph
  TemporalEvaluatorPtr tempeval(new TemporalEvaluator());
  tempeval->setGraph((void *)graph.get());

  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  // TODO: It might be easier to just iterate over the trajectory??
  typedef pose_graph::eval::Mask::DirectionFromVertexDirect<Graph>
      DirectionEvaluator;
  auto direval = std::make_shared<DirectionEvaluator>(*qdata.live_id, true);
  direval->setGraph((void *)graph.get());

  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);
  evaluator->setGraph((void *)graph.get());

  // Iterate through the locked poses in the trajectory, interpolate inbetween
  // them and save off all of the data points.
  VertexId vertexa_id(VertexId::Invalid());
  auto itr = graph->beginDfs(*qdata.live_id, temporal_depth, evaluator);
  itr++;
  for (; itr != graph->end(); ++itr) {
    // Grab the two time stamps associated with the poses.
    vertexa_id = itr->to();
    auto vertexa = graph->at(itr->to());
    auto vertexb = graph->at(itr->from());
    int64_t stamp_a = vertexa->keyFrameTime().nanoseconds_since_epoch;
    int64_t stamp_b = vertexb->keyFrameTime().nanoseconds_since_epoch;

    // save vertex b
    auto *ba_pose = trajectory_status_.mutable_optimization_window()->Add();
    ba_pose->set_id(vertexb->id());
    ba_pose->set_locked(true);
    ba_pose->set_interpolated(false);
    ba_pose->set_stamp(stamp_b);
    auto curr_eval = trajectory_->getInterpPoseEval(steam::Time(stamp_b));
    *ba_pose->mutable_t_q_0() << curr_eval->evaluate();

    // grab the optimized velocity
    auto velocity = velocity_map_[vertexb->id()]->getValue();
    auto *proto_velocity = ba_pose->mutable_velocity();
    for (auto idx = 0; idx < 6; ++idx) {
      proto_velocity->add_entries(velocity(idx, 0));
    }

    int64_t time_delta = stamp_b - stamp_a;
    // interpolate between pose a and pose b
    for (auto idx = 5; idx >= 0; --idx) {
      int64_t query_time = stamp_a + ((time_delta / 5) * idx);
      ba_pose = trajectory_status_.mutable_optimization_window()->Add();
      ba_pose->set_interpolated(true);
      ba_pose->set_stamp(query_time);
      curr_eval = trajectory_->getInterpPoseEval(steam::Time(query_time));
      *ba_pose->mutable_t_q_0() << curr_eval->evaluate();
    }
  }

  // Get the last vertex in the trajectory and save it off.
  if (vertexa_id.isValid()) {
    auto vertexa = graph->at(vertexa_id);
    int64_t stamp_a = vertexa->keyFrameTime().nanoseconds_since_epoch;
    // save vertex a
    ba_pose = trajectory_status_.mutable_optimization_window()->Add();
    ba_pose->set_id(vertexa->id());
    ba_pose->set_locked(true);
    ba_pose->set_interpolated(false);
    ba_pose->set_stamp(stamp_a);
    curr_eval = trajectory_->getInterpPoseEval(steam::Time(stamp_a));
    *ba_pose->mutable_t_q_0() << curr_eval->evaluate();

    // grab the optimized velocity
    auto velocity = velocity_map_[vertexa->id()]->getValue();
    auto *proto_velocity = ba_pose->mutable_velocity();
    for (auto idx = 0; idx < 6; ++idx) {
      proto_velocity->add_entries(velocity(idx, 0));
    }
  }

  // insert into the vertex.
  auto run = graph->run((*qdata.live_id).majorId());
  std::string stream_name("/results/quick_vo_trajectory");
  run->registerVertexStream(stream_name);
  vertex->insert<decltype(trajectory_status_)>(stream_name, trajectory_status_,
                                               *qdata.stamp);
}
#endif
void KeyframeOptimizationModule::updateGraphImpl(QueryCache &qdata, MapCache &,
                                                 const Graph::Ptr &graph,
                                                 VertexId id) {
  if (config_->save_trajectory) {
    throw std::runtime_error{"Trajectory saving not ported yet."};
#if false
    saveTrajectory(qdata, mdata, graph, id);
#else
    (void)qdata;
    (void)graph;
    (void)id;
#endif
  }
}

}  // namespace stereo
}  // namespace tactic
}  // namespace vtr