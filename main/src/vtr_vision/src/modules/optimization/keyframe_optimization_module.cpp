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
 * \file keyframe_optimization_module.cpp
 * \brief KeyframeOptimizationModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/modules/optimization/keyframe_optimization_module.hpp>




namespace vtr {

namespace vision {

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

KeyframeOptimizationModule::Config::ConstPtr KeyframeOptimizationModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) {
  auto keyframe_config = std::make_shared<KeyframeOptimizationModule::Config>();
  auto casted_config =
      std::static_pointer_cast<SteamModule::Config>(keyframe_config);
  *casted_config = *SteamModule::Config::fromROS(node, param_prefix);  // copy over base config
  // clang-format off
  keyframe_config->pose_prior_enable = node->declare_parameter<bool>(param_prefix + ".pose_prior_enable", keyframe_config->pose_prior_enable);
  keyframe_config->depth_prior_enable = node->declare_parameter<bool>(param_prefix + ".depth_prior_enable", keyframe_config->depth_prior_enable);
  keyframe_config->depth_prior_weight = node->declare_parameter<double>(param_prefix + ".depth_prior_weight", keyframe_config->depth_prior_weight);
  keyframe_config->use_migrated_points = node->declare_parameter<bool>(param_prefix + ".use_migrated_points", keyframe_config->use_migrated_points);
  keyframe_config->min_inliers = node->declare_parameter<int>(param_prefix + ".min_inliers", keyframe_config->min_inliers);
  
  // clang-format on

  return keyframe_config;
}

bool KeyframeOptimizationModule::isLandmarkValid(const Eigen::Vector3d &point) {
  // check depth
  return point(2) > 0.0 && point(2) < keyframe_config_->max_point_depth;
}

steam::OptimizationProblem KeyframeOptimizationModule::generateOptimizationProblem(
    CameraQueryCache &qdata, const std::shared_ptr<const Graph> &graph) {
  // initialize the steam problem.
  resetProblem(*qdata.T_r_m);

  auto problem = OptimizationProblem();

  // Initialize the landmark container
  std::vector<steam::stereo::HomoPointStateVar::Ptr> landmarks_ic;

  // get the cache data
  auto &map_landmarks = *qdata.map_landmarks;
  auto &calibrations = *qdata.rig_calibrations;
  auto calibration_itr = calibrations.begin();

  // get the ransac matches
  auto ransac_matches = *qdata.ransac_matches;

  // iterate through the inliers of every rig
  for (uint32_t rig_idx = 0; rig_idx < ransac_matches.size();
       ++rig_idx, ++calibration_itr) {

    // get the inliers and calibration for this rig.
    auto &rig_ransac_matches = (*qdata.ransac_matches)[rig_idx];
    auto &calibration = *calibration_itr;

    // Setup camera intrinsics
    StereoCalibPtr sharedStereoIntrinsics = toStereoSteamCalibration(calibration);
    

    // Create a transform evaluator for T_q_m, in the camera sensor frame.
    steam::Evaluable<lgmath::se3::Transformation>::ConstPtr tf_qs_mv = nullptr;
    steam::Evaluable<lgmath::se3::Transformation>::ConstPtr tf_qs_ms = nullptr;

    if (config_->use_T_q_m_prior) {
      tf_qs_mv = steam::se3::compose(tf_sensor_vehicle_, query_pose_);
      tf_qs_ms = steam::se3::compose_rinv(tf_qs_mv, tf_sensor_vehicle_);
    } else {
      tf_qs_mv = steam::se3::compose(tf_sensor_vehicle_, query_pose_);
      tf_qs_ms = steam::se3::compose_rinv(tf_qs_mv, tf_sensor_vehicle_);
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

          if (keyframe_config_->use_migrated_points) {
            // set the map points from the migrated points
            map_points = &(*qdata.migrated_points_3d);

            // check the validity
            bool map_point_valid = qdata.migrated_validity->at(match.first);
            if (!map_point_valid) {
              continue;  // if this landmark isn't valid, skip it
            }

            // // convenience accessor
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
          landmarks_ic.push_back(stereo::HomoPointStateVar::MakeShared(map_point));

          

          // Get landmark reference
          auto &landVar = landmarks_ic.back();

          // lock the landmark
          landVar->locked() = true;

          steam::BaseNoiseModel<4>::Ptr noise_stereo;

          try {
            // If this is with migrated points, then use the dynamic model.
            if (keyframe_config_->use_migrated_points) {
              // // TODO: Calculate directly instead of in landmark migration.
              auto &migrated_cov = *qdata.migrated_covariance;
              const Eigen::Matrix3d &cov = Eigen::Map<Eigen::Matrix3d>(
                  migrated_cov.col(match.first).data());

              // set up the noise for the stereo
              using NoiseEval = steam::stereo::LandmarkNoiseEvaluator ;
              auto noise_eval = std::make_shared<NoiseEval>(
                  landVar->value(), cov, meas_covariance,
                  sharedStereoIntrinsics, tf_qs_ms);
              noise_stereo = steam::DynamicNoiseModel<4>::MakeShared(noise_eval, steam::NoiseType::COVARIANCE);
              CLOG(DEBUG, "stereo.keyframe_optimization") << "Noise \n" << noise_eval->value() << "\nMeas Noise:\n" << meas_covariance
                                                          << "\nLand Noise:\n" << cov;

            } else {
              noise_stereo = std::make_shared<steam::StaticNoiseModel<4>>(meas_covariance);
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

          if (!tf_qs_ms || !landVar) {
            CLOG(ERROR, "stereo.keyframe_optimization") << tf_qs_ms << " " << landVar;
            continue;
          }

          if (!noise_stereo || !sharedLossFunc_) {
            CLOG(ERROR, "stereo.keyframe_optimization") << noise_stereo << " " << sharedLossFunc_;
            continue;
          }

          // Construct error function for observation to the fixed landmark.
          auto errorfunc = stereo::StereoErrorEvaluator::MakeShared(data, sharedStereoIntrinsics, tf_qs_ms, landVar);
          auto cost = WeightedLeastSqCostTerm<4>::MakeShared(errorfunc, noise_stereo, sharedLossFunc_);
          // add the cost term
          // CLOG(DEBUG, "stereo.keyframe_optimization") << "Cost " << cost->cost();
          problem.addCostTerm(cost);


          // steam throws?
        } catch (std::exception &e) {
          LOG(ERROR) << "Error with noise model:\n" << e.what();
          continue;
        }
      }  // end for match
    }    // end for channel
  }      // end for rig

  // Add pose variables
  problem.addStateVariable(map_pose_);
  problem.addStateVariable(query_pose_);

  // Add pose prior if requested
  if (keyframe_config_->pose_prior_enable) {
    addPosePrior(qdata, problem);
  }

  // Add landmark variables
  for (auto &landmark : landmarks_ic) {
    problem.addStateVariable(landmark);
  }

  // Add the trajectory stuff.
  if (config_->trajectory_smoothing) {
    computeTrajectory(qdata, graph, problem);
  }

  // Go through each rig
  return problem;
}

void KeyframeOptimizationModule::addPosePrior(CameraQueryCache &qdata, OptimizationProblem &problem) {
  // TODO: Replace with T_leaf_branch from graph?
  EdgeTransform &pose_prior = *qdata.T_r_m_prior;

  steam::BaseNoiseModel<6>::Ptr priorUncertainty;

  /// @brief the loss function associated with observation cost.
  steam::BaseLossFunc::Ptr priorLossFunc = std::make_shared<L2LossFunc>();

  try {
    auto pose_cov = pose_prior.cov();
    priorUncertainty = std::make_shared<steam::StaticNoiseModel<6>>(pose_cov);
  } catch (std::invalid_argument &e) {
    priorUncertainty = std::make_shared<steam::StaticNoiseModel<6>>(
        Eigen::Matrix<double, 6, 6>::Identity());
    LOG(ERROR) << "Error on adding pose prior: " << e.what();
  }
  auto prior_error_func = std::make_shared<se3::SE3ErrorEvaluator>(query_pose_, pose_prior);
  // Create cost term and add to problem
  steam::WeightedLeastSqCostTerm<6>::Ptr prior_cost(
      new steam::WeightedLeastSqCostTerm<6>(
          prior_error_func, priorUncertainty, priorLossFunc));

  problem.addCostTerm(prior_cost);

}

bool KeyframeOptimizationModule::verifyInputData(CameraQueryCache &qdata) {
  // sanity check
  if ((config_->is_odometry && *qdata.odo_success == false) /* || *qdata.map_status == MAP_NEW */) {
    return false;
  }

  if ((!config_->is_odometry) && qdata.loc_success.valid() && *qdata.loc_success == false /* || *qdata.map_status == MAP_NEW */) {
    return false;
  }

  // if there are no inliers, then abort.
  if (qdata.ransac_matches.valid() == false) {
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
  if (inlier_count < keyframe_config_->min_inliers && keyframe_config_->pose_prior_enable == false) {
    LOG(ERROR) << "KeyframeOptimizationModule::verifyInputData(): Insufficient "
                  "number of inliers, Bailing on steam problem!";
    return false;
  }

  // If we dont have an initial condition, then just set identity
  if (!qdata.T_r_m.valid()) {
    *qdata.T_r_m = lgmath::se3::Transformation();
  }
  return true;
}

bool KeyframeOptimizationModule::verifyOutputData(CameraQueryCache &) {
  return true;
}


void KeyframeOptimizationModule::resetProblem(EdgeTransform &T_q_m) {
  // set up the transforms
  map_pose_ = std::make_shared<se3::SE3StateVar>(lgmath::se3::Transformation());
  map_pose_->locked() = true; // lock the 'origin' pose
  query_pose_ = std::make_shared<se3::SE3StateVar>(T_q_m);

  // make the loss functions, TODO: make this configurable.
  sharedDepthLossFunc_ = steam::DcsLossFunc::MakeShared(2.0);
  sharedLossFunc_ = steam::DcsLossFunc::MakeShared(2.0);

}


void KeyframeOptimizationModule::computeTrajectory(
    CameraQueryCache &qdata, const std::shared_ptr<const Graph> &graph, OptimizationProblem &problem) {
  velocity_map_.clear();

  // reset the trajectory
  trajectory_ = std::make_shared<traj::const_vel::Interface>(smoothing_factor_information_);

  // get the map vertex
  auto map_vertex = graph->at(*qdata.vid_odo);

  // get the stamps
  auto &query_stamp = *qdata.stamp;
  const auto &map_stamp = map_vertex->vertexTime();

  // set up a search for the previous keyframes in the graph
  auto tempeval = std::make_shared<pose_graph::eval::mask::temporal::Eval<Graph>>(*graph);

  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  using DirectionEvaluator = pose_graph::eval::mask::direction_from_vertex::Eval;
  auto direval = std::make_shared<DirectionEvaluator>(*qdata.vid_odo, true);

  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);

  // look back five vertices
  int temporal_depth = 5;

  // perform the search and automatically step back one
  auto itr = graph->beginDfs(*qdata.vid_odo, temporal_depth, evaluator);
  itr++;

  // initialize the compunded transform
  EdgeTransform T_p_m;

  // initialize the timestamp that will be used as
  auto next_stamp = map_vertex->vertexTime();

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
    const auto &prev_stamp = prev_vertex->vertexTime();

    // get the transform and compund it
    const auto &T_pp1_p = itr->e()->T();
    T_p_m = T_pp1_p.inverse() * T_p_m;

    // set up a locked global pose for this vertex, with an tf evaluator
    // Note: normally steam would have states T_a_0, T_b_0, ..., where 'a' and
    // 'b' are always sequential in time. So in our case, since our locked '0'
    // frame is in the future, 'a' is actually further from '0' than 'b'.
    auto prev_pose = std::make_shared<steam::se3::SE3StateVar>(T_p_m);
    prev_pose->locked() = true;
    // auto tf_prev =
    //     std::make_shared<steam::se3::TransformStateEvaluator>(prev_pose);

    // time difference between next and previous
    int64_t next_prev_dt =  next_stamp - prev_stamp;

    // generate a velocity estimate
    // The velocity is in the body frame, helping you get from 'a' to 'b'.
    // This part can ignore the fact that the transforms above are weird
    // (new to old instead of old to new), and keep doing vel_b_a.
    // we use pp1_p instead of p_pm1 for convenience
    Eigen::Matrix<double, 6, 1> prev_velocity =
        T_pp1_p.vec() / (next_prev_dt / 1e9);

    auto prev_frame_velocity =  std::make_shared<VSpaceStateVar<6>>(prev_velocity);

    velocity_map_.insert({prev_vertex->id(), prev_frame_velocity});

    // add the velocity to the state variable.
    problem.addStateVariable(prev_frame_velocity);

    // make a steam time from the timstamp
    Time prev_time(
        static_cast<int64_t>(prev_stamp));

    // Add the poses to the trajectory
    trajectory_->add(prev_time, prev_pose, prev_frame_velocity);
    next_stamp = prev_stamp;
  }

  // lock the velocity at the begining of the trajectory
  if (velocity_map_.empty() == false) {
    velocity_map_.begin()->second->locked() = true;
  }

  // time difference between query and map
  int64_t query_map_dt =
      query_stamp - map_stamp;
  Eigen::Matrix<double, 6, 1> query_velocity =
      query_pose_->value().vec() / (query_map_dt / 1e9);
  VSpaceStateVar<6>::Ptr map_frame_velocity(
      new VSpaceStateVar<6>(query_velocity));

  //TODO should this be locked?

  VSpaceStateVar<6>::Ptr query_frame_velocity(
      new VSpaceStateVar<6>(query_velocity));

  velocity_map_.insert({*qdata.vid_odo, map_frame_velocity});
  velocity_map_.insert({VertexId::Invalid(), query_frame_velocity});

  // add the velocities to the state variable.
  problem.addStateVariable(map_frame_velocity);
  problem.addStateVariable(query_frame_velocity);

  Time map_time(static_cast<int64_t>(map_stamp));
  Time query_time(
      static_cast<int64_t>(query_stamp));

  // Add the poses to the trajectory
  trajectory_->add(map_time, map_pose_, map_frame_velocity);
  trajectory_->add(query_time, query_pose_, query_frame_velocity);

  // Trajectory prior smoothing terms
  trajectory_->addPriorCostTerms(problem);
  // trajectory_->addPriorCostTerms(depth_cost_terms_);

  if (config_->velocity_prior) {
    trajectory_->addVelocityPrior(query_time, velocity_prior_,
                                  velocity_prior_cov_);
  }

}

void KeyframeOptimizationModule::updateCaches(CameraQueryCache &qdata) {
  // update our estimate for the transform
  *qdata.T_r_m = query_pose_->value();

  if (config_->is_odometry)
    *qdata.w_v_r_in_r_odo = trajectory_->getVelocityInterpolator(*qdata.timestamp_odo)->value();

  // give the query cache a copy of the trajectory estimate
  // qdata.trajectory = trajectory_;

  // look up covariance on pose
  if (!config_->disable_solver) {
    // auto gn_solver =
    //     std::dynamic_pointer_cast<steam::GaussNewtonSolver>(solver_);
    // if (gn_solver != nullptr) {
    //   if (config_->solver_type == "LevenburgMarquardt" ||
    //       backup_lm_solver_used_) {
    //     auto lm_solver =
    //         std::dynamic_pointer_cast<steam::LevMarqGaussNewtonSolver>(
    //             gn_solver);
    //     std::lock_guard<std::mutex> iteration_lock(*qdata.steam_mutex);
    //     try {
    //       lm_solver->solveCovariances();
    //     } catch (std::runtime_error &e) {
    //       LOG(ERROR) << "KeyframeOptimizationModule: Couldn't solve for "
    //                     "covariance in LM solver!"
    //                  << std::endl
    //                  << e.what();
    //     }
    //   }
    //   // now we can safely query the covariance
    //   auto cov = gn_solver->queryCovariance(query_pose_->getKey());
    //   (*qdata.T_r_m).setCovariance(cov);
    // } else {
    //   LOG(INFO)
    //       << "This solver does not derive from The GaussNewtonSolverBase!";
    // }

  } else {
    // default to zero covariance, because we have no other information
    qdata.T_r_m->setZeroCovariance();
  }
}



}  // namespace vision
}  // namespace vtr