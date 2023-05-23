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
 * \file stereo_window_optimization_module.cpp
 * \brief StereoWindowOptimizationModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_common/timing/stopwatch.hpp"
#include <vtr_vision/steam_extensions/landmark_range_prior.hpp>
#include <vtr_vision/geometry/geometry_tools.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/modules/optimization/stereo_window_optimization_module.hpp>
#include <vtr_vision/types.hpp>


namespace vtr {
namespace vision {

using namespace tactic;
using namespace steam;

StereoWindowOptimizationModule::Config::ConstPtr StereoWindowOptimizationModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) {

  auto window_config = std::make_shared<StereoWindowOptimizationModule::Config>();
  auto casted_config =
      std::static_pointer_cast<SteamModule::Config>(window_config);
  *casted_config = *SteamModule::Config::fromROS(node, param_prefix);  // copy over base config

  // clang-format off
  window_config->depth_prior_enable = node->declare_parameter<bool>(param_prefix + ".depth_prior_enable", window_config->depth_prior_enable);
  window_config->depth_prior_weight = node->declare_parameter<double>(param_prefix + ".depth_prior_weight", window_config->depth_prior_weight);
  // clang-format on
  return window_config;
}

steam::OptimizationProblem StereoWindowOptimizationModule::generateOptimizationProblem(
    CameraQueryCache &qdata, const std::shared_ptr<const Graph> &graph) {
  // get references to the relevent data.
  LandmarkMap &lm_map = *qdata.landmark_map;
  auto &poses = *qdata.pose_map;
  auto &tsv_transforms = *qdata.T_sensor_vehicle_map;
  const auto &calibrations = *qdata.rig_calibrations;
  auto calibration_itr = calibrations.begin();

  // reset to remove any old data from the problem setup
  resetProblem();

  auto problem = OptimizationProblem();

  // get calibration for this rig.
  auto &calibration = *calibration_itr;

  // Setup camera intrinsics, TODO: This should eventually be different for each
  // rig.
  StereoCalibPtr sharedStereoIntrinsics = toStereoSteamCalibration(calibration);
  

  // Go through all of the landmarks
  for (auto &landmark : lm_map) {
    // 1. get the pose associated with this map
    auto vertex = landmark.first.vid;
    auto &lm_pose = poses[vertex];

    // Extract the point associated with this landmark
    Eigen::Vector3d lm_point(landmark.second.point.x, landmark.second.point.y,
                             landmark.second.point.z);

    // Extract the validity of the landmark
    bool map_point_valid = landmark.second.valid;

    // If the point and its depth is valid, then add it as a landmark.
    if (map_point_valid && isLandmarkValid(lm_point) &&
        landmark.second.observations.size() > 1) {
      landmark.second.steam_lm = stereo::HomoPointStateVar::MakeShared(lm_point);
      auto &steam_lm = landmark.second.steam_lm;

      // set the lock only if the map is initialized
      steam_lm->locked() =  lm_pose.isLocked();

      // add the depth prior
      if (window_config_->depth_prior_enable) {
        addDepthCost(problem, steam_lm);
      }
    } else {
      landmark.second.steam_lm.reset();
      continue;
    }

    // get a reference to our new landmark.
    auto &steam_lm = landmark.second.steam_lm;
    // 3. go through the observations, and add cost terms for each.
    for (const LandmarkObs &obs : landmark.second.observations) {
      try {  // steam throws?

        // get the keypoints for this observation.
        auto obs_vertex = obs.origin_ref.from_id.vid;
        auto &obs_pose = poses[obs_vertex];

        // Set up the transform evaluator (i.e. The transform that takes the
        // landmark into the observation frame)
        steam::Evaluable<lgmath::se3::Transformation>::ConstPtr pose_obs_lm;

        // if we are observing the landmark in the parent frame, then use
        // identity.
        if (obs_vertex == vertex) {
          pose_obs_lm = tf_identity_;
        } else {
          // // otherwise compose the transform
          // steam::se3::TransformEvaluator::Ptr pose_lm_0 =
          //     steam::se3::TransformStateEvaluator::MakeShared(
          //         lm_pose.tf_state_var);
          // steam::se3::TransformEvaluator::Ptr pose_obs_0 =
          //     steam::se3::TransformStateEvaluator::MakeShared(
          //         obs_pose.tf_state_var);
          pose_obs_lm = steam::se3::compose_rinv(lm_pose.tf_state_var, obs_pose.tf_state_var);
        }

        // make a pointer to the landmark->observation transform
        steam::Evaluable<lgmath::se3::Transformation>::ConstPtr T_obs_lm;

        // Compose with non-fixed camera to vehicle transform
        auto T_s_v_obs_ptr = tsv_transforms.find(obs_vertex);
        auto T_s_v_ptr = tsv_transforms.find(vertex);

        // check if we have pre-loaded the transforms
        if (T_s_v_ptr == tsv_transforms.end() ||
            T_s_v_obs_ptr == tsv_transforms.end()) {
          // no, compose with fixed camera to vehicle transform
          LOG(WARNING) << "Couldn't find transform! for either " << obs_vertex
                       << " or " << vertex;
          T_obs_lm = steam::se3::compose_rinv(
              steam::se3::compose(tf_sensor_vehicle_, pose_obs_lm),
              tf_sensor_vehicle_);
        } else {
          // yes, create the non-static (but fixed for this optimisation)
          // sensor->vehicle transform have we created this specific transform
          // for *vertex* before?
          auto composed_T_s_v_fixed_ptr = tf_sensor_vehicle_map_.find(vertex);
          if (composed_T_s_v_fixed_ptr == tf_sensor_vehicle_map_.end()) {
            // we haven't, make it
            tf_sensor_vehicle_map_[vertex] = steam::se3::SE3StateVar::MakeShared(T_s_v_ptr->second);            // this should now get the correct reference
            tf_sensor_vehicle_map_[vertex]->locked() = true;

            composed_T_s_v_fixed_ptr = tf_sensor_vehicle_map_.find(vertex);
          }
          // have we created this specific transform for *obs_vertex* before?
          auto composed_T_s_v_fixed_obs_ptr =
              tf_sensor_vehicle_map_.find(obs_vertex);
          if (composed_T_s_v_fixed_obs_ptr == tf_sensor_vehicle_map_.end()) {
            // we haven't, make it
            // tf_sensor_vehicle_map_[obs_vertex] = T_s_v_obs_ptr->second;
            tf_sensor_vehicle_map_[obs_vertex] = steam::se3::SE3StateVar::MakeShared(T_s_v_obs_ptr->second);            // this should now get the correct reference
            tf_sensor_vehicle_map_[obs_vertex]->locked() = true;
            // this should now get the correct reference
            composed_T_s_v_fixed_obs_ptr =
                tf_sensor_vehicle_map_.find(obs_vertex);
          }
          // ok...., now we can compose the transform evaluator. Compose with
          // non-fixed camera to vehicle transform
          T_obs_lm = steam::se3::compose_rinv(
              steam::se3::compose(composed_T_s_v_fixed_obs_ptr->second,
                                  pose_obs_lm),
              composed_T_s_v_fixed_ptr->second);
        }

        // set up the stereo noise for each potential type
        steam::BaseNoiseModel<4>::Ptr noise_stereo;

        // set up the measurement covariance vector
        unsigned m_sz = 4;
        Eigen::MatrixXd meas_cov(m_sz, m_sz);
        meas_cov.setZero();

        // add the measurement covariances from the stored memory
        unsigned idx = 0;
        for (auto &cov : obs.covariances) {
          meas_cov(2 * idx, 2 * idx) = cov[0];
          meas_cov(2 * idx + 1, 2 * idx + 1) = cov[3];
          idx++;
        }

        // add to the noise models
        noise_stereo = steam::StaticNoiseModel<4>::MakeShared(meas_cov);
        

        // Construct the measurement vector for the current camera
        Eigen::MatrixXd data(obs.keypoints.size() * 2, 1);
        for (uint32_t idx = 0; idx < obs.keypoints.size(); idx++) {
          data(idx * 2) = obs.keypoints.at(idx).position.x;
          data(idx * 2 + 1) = obs.keypoints.at(idx).position.y;
        }


        // Construct error function for the current camera
        steam::stereo::StereoErrorEvaluator::Ptr errorfunc(
            new steam::stereo::StereoErrorEvaluator(data, sharedStereoIntrinsics,
                                              T_obs_lm, steam_lm));
        // Construct cost term for the current camera
        steam::WeightedLeastSqCostTerm<4>::Ptr cost(
            new steam::WeightedLeastSqCostTerm<4>(errorfunc, noise_stereo,
                                                      sharedLossFunc_));

        // finally, add the cost.
        // cost_terms_->add(cost);
        problem.addCostTerm(cost);


        // steam throws?
      } catch (std::exception &e) {
        LOG(ERROR) << "Error with noise model:\n" << e.what();
        continue;
      }
    }
  }

  // add pose variables
  int jj = 0;
  for (auto &pose : poses) {
    auto &steam_pose = pose.second;
    problem.addStateVariable(steam_pose.tf_state_var);
    jj++;
  }

  // Add landmark variables
  for (auto &landmark : lm_map) {
    if (landmark.second.steam_lm != nullptr &&
        (landmark.second.valid) == true) {
      problem.addStateVariable(landmark.second.steam_lm);
    }
  }

  // if (window_config_->depth_prior_enable) {
  //   problem.addCostTerm(depth_cost_terms_);
  // }

  // add trajectory stuff
  if (window_config_->trajectory_smoothing == true) {
    // reset the trajectory
    trajectory_ = std::make_shared<traj::const_vel::Interface>(smoothing_factor_information_);

    bool prior_added = false;
    for (auto &pose : poses) {
      auto &steam_pose = pose.second;
      if (steam_pose.velocity == nullptr) {
        LOG(ERROR) << "Trajectory velocity was null!";
        continue;
      }
      trajectory_->add(steam_pose.time, steam_pose.tf_state_var,
                       steam_pose.velocity);
      problem.addStateVariable(steam_pose.velocity);
      if (window_config_->velocity_prior == true &&
          steam_pose.isLocked() == false && prior_added == false) {
        trajectory_->addVelocityPrior(steam_pose.time, velocity_prior_,
                                      velocity_prior_cov_);
      }
    }

    // Add smoothing terms
    trajectory_->addPriorCostTerms(problem);
  }

  return problem;
}

void StereoWindowOptimizationModule::resetProblem() {
  // make the depth loss function
  sharedDepthLossFunc_ = steam::DcsLossFunc::MakeShared(2.0);

  // make the loss function, TODO: make this configurable, move to member var.
  sharedLossFunc_ = steam::DcsLossFunc::MakeShared(2.0);
}

void StereoWindowOptimizationModule::addDepthCost(
  OptimizationProblem& problem, stereo::HomoPointStateVar::Ptr landmark) {
  auto range_error_func = stereo::LandmarkRangePrior::MakeShared(landmark);

  double depth = landmark->value().hnormalized()[2];
  double weight = window_config_->depth_prior_weight / depth;
  auto rangeNoiseModel = steam::StaticNoiseModel<1>::MakeShared(
      Eigen::Matrix<double, 1, 1>::Identity() * weight);
  auto depth_cost = WeightedLeastSqCostTerm<1>::MakeShared(range_error_func, rangeNoiseModel,
                                               sharedDepthLossFunc_);
  problem.addCostTerm(depth_cost);
}

bool StereoWindowOptimizationModule::verifyInputData(CameraQueryCache &qdata) {
  // make sure we have a landmark and pose map, and calibration.
  if (qdata.landmark_map.valid() == false ||
      qdata.pose_map.valid() == false ||
      qdata.rig_calibrations.valid() == false) {
    LOG(ERROR)
        << "StereoWindowOptimizationModule::verifyInputData(): Input data for "
           "windowed BA problem is not set! (Is the Windowed Recall "
           "Module Running?)";
    return false;
  }

  // If there is nothing to optimize, then quit.
  if ((*qdata.pose_map).empty() || (*qdata.landmark_map).empty()) {
    LOG(ERROR)
        << "StereoWindowOptimizationModule::verifyInputData(): No poses or "
           "landmarks found. (Is the Windowed Recall Module Running?)";
    return false;
  } else
    return true;
}

bool StereoWindowOptimizationModule::isLandmarkValid(const Eigen::Vector3d &point) {
  // check depth
  if (point(2) > window_config_->max_point_depth ||
      point(2) < window_config_->min_point_depth) {
    return false;
  }

  // check the distance from the plane
  if (window_config_->perform_planarity_check) {
    throw std::runtime_error{
        "planarity check not ported and tested - window opt"};
#if false
    if (qdata.plane_coefficients.valid() == true) {
      // estimate the distance of the point from the plane
      double dist =
          vision::estimatePlaneDepth(point, *qdata.plane_coefficients);

      // if it is beyond the maximum depth, it's invalid
      if (dist > window_config_->plane_distance) {
        return false;
      }
    }
#endif
  }

  // if we got here, the landmark is valid
  return true;
}

bool StereoWindowOptimizationModule::verifyOutputData(CameraQueryCache &qdata) {
  // attempt to fit a plane to the point cloud
  if (window_config_->perform_planarity_check) {
    throw std::runtime_error{
        "planarity check not ported and tested - window opt"};
#if false
    // point cloud containers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(2000);
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;

    if (qdata.landmark_map.valid()) {
      LandmarkMap &lm_map = *qdata.landmark_map;

      // push the points into a PCL point cloud
      for (auto &landmark : lm_map) {
        if (landmark.second.steam_lm != nullptr &&
            (*landmark.second.valid) == true) {
          Eigen::Vector3d steam_point =
              landmark.second.steam_lm->value().hnormalized();
          cloud->points.push_back(
              pcl::PointXYZ(steam_point[0], steam_point[1], steam_point[2]));
        }
      }

      // attempt to fit a plane to the data
      if (vision::estimatePlane(cloud, window_config_->plane_distance, coefficients,
                                inliers) &&
          std::count_if(inliers.indices.begin(), inliers.indices.end(),
                        [](int i) { return i; }) > 100) {
        qdata.plane_coefficients =
            Eigen::Vector4f(coefficients.values[0], coefficients.values[1],
                            coefficients.values[2], coefficients.values[3]);
      } else {
        LOG(WARNING) << "SteamModule: Couldn't estimate map landmarks plane! "
                        "Inliers/Points: "
                     << std::count_if(inliers.indices.begin(),
                                      inliers.indices.end(),
                                      [](int i) { return i; })
                     << "/" << cloud->points.size();
      }
    }
#endif
  }

  // if the landmark map is valid, check for outliers from the plane or min/max
  // point depths
  if (qdata.landmark_map.valid()) {
    LandmarkMap &lm_map = *qdata.landmark_map;

    // do a sanity check on the points to ensure there are no gross outliers
    for (auto &landmark : lm_map) {
      if (landmark.second.steam_lm != nullptr &&
          landmark.second.valid == true) {
        // get the steam point
        Eigen::Vector3d steam_point =
            landmark.second.steam_lm->value().hnormalized();

        // perform the validity check
        landmark.second.valid = isLandmarkValid(steam_point);
      }
    }
  }

  // always return true for now
  return true;
}

void StereoWindowOptimizationModule::updateCaches(CameraQueryCache &qdata) {
#if false
  /// \note we should not assign to trajectory cache since qdata.trajectory is
  /// also used by path tracker for extrapolation - choose another name
  qdata.trajectory = trajectory_;
#else
  (void)qdata;
#endif
}

#if false
void StereoWindowOptimizationModule::saveTrajectory(
    CameraQueryCache &qdata, const std::shared_ptr<Graph> &graph) {
  auto &poses = *qdata.pose_map;
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

  // Set up the status message
  asrl::status_msgs::TrajectoryStatus status_msg;

  // record the optimization window, starting from the origin.
  auto pose_a_itr = poses.begin();
  auto pose_b_itr = poses.begin();
  pose_b_itr++;

  for (; pose_a_itr != poses.end() && pose_b_itr != poses.end();
       ++pose_a_itr, ++pose_b_itr) {
    // save off pose A
    auto vertex = graph->at(VertexId(pose_a_itr->first));
    auto *ba_pose = status_msg.mutable_optimization_window()->Add();
    ba_pose->set_id(vertex->id());
    ba_pose->set_locked(pose_a_itr->second.locked());
    ba_pose->set_interpolated(false);
    ba_pose->set_stamp(vertex->vertexTime().nanoseconds_since_epoch());
    EdgeTransform T_1_0 = pose_a_itr->second.tf_state_var->value();

    // extract the covariance
    if (pose_a_itr->second.locked() == false) {
      auto covariance =
          gn_solver->queryCovariance(pose_a_itr->second.tf_state_var->key());
      T_1_0.setCovariance(covariance);
    }
    *ba_pose->mutable_t_q_0() << T_1_0;

    // compute the delta in time between pose a and b
    auto time_a = graph->at(VertexId(pose_a_itr->first))
                      ->vertexTime()
                      .nanoseconds_since_epoch();
    auto time_b = graph->at(VertexId(pose_b_itr->first))
                      ->vertexTime()
                      .nanoseconds_since_epoch();
    auto time_delta = time_b - time_a;

    // sample the trajectory between a and b.
    for (auto idx = 0; idx < 5; ++idx) {
      int64_t query_time = time_a + ((time_delta / 5) * idx);
      auto *ba_pose = status_msg.mutable_optimization_window()->Add();
      ba_pose->set_interpolated(true);
      ba_pose->set_stamp(query_time);
      auto curr_eval =
          qdata.trajectory->getInterpPoseEval(steam::Time(query_time));
      EdgeTransform T_q_0 = curr_eval->evaluate();

      // if pose a and b are both unlocked, then interpolate the covariance
      // TODO: Re-enable once the steam library supports it.
      //      if(pose_a_itr->second.locked() == false &&
      //      pose_b_itr->second.locked() == false) {
      //        auto cov =
      //        qdata.trajectory->getCovariance(*gn_solver.get(),steam::Time(query_time));
      //        T_q_0.setCovariance(cov);
      //      }

      *ba_pose->mutable_t_q_0() << T_q_0;
    }
  }

  // save off the final pose...
  auto vertex = graph->at(VertexId(pose_a_itr->first));
  auto *ba_pose = status_msg.mutable_optimization_window()->Add();
  ba_pose->set_id(vertex->id());
  ba_pose->set_locked(pose_a_itr->second.locked());
  ba_pose->set_interpolated(false);
  ba_pose->set_stamp(vertex->vertexTime().nanoseconds_since_epoch());
  EdgeTransform T_1_0 = pose_a_itr->second.tf_state_var->value();
  if (pose_a_itr->second.locked() == false) {
    // extract the covariance
    auto covariance =
        gn_solver->queryCovariance(pose_a_itr->second.tf_state_var->key());
    T_1_0.setCovariance(covariance);
  }
  *ba_pose->mutable_t_q_0() << T_1_0;
  *ba_pose->mutable_t_q_0() << T_1_0;

  // Extrapolate into the future.
  int64_t future_stamp = vertex->vertexTime().nanoseconds_since_epoch();
  int64_t interval = 1e9 / 10;
  for (auto idx = 0; idx < 10; ++idx) {
    auto *ba_pose = status_msg.mutable_optimization_window()->Add();
    ba_pose->set_id(-1);
    ba_pose->set_locked(false);
    ba_pose->set_interpolated(true);
    ba_pose->set_stamp(future_stamp);
    auto curr_eval = trajectory_->getInterpPoseEval(steam::Time(future_stamp));
    EdgeTransform T_q_0 = curr_eval->evaluate();

    // TODO re-enable once the steam library supports it.
    // if pose a and b are both unlocked, then interpolate the covariance
    //    auto cov =
    //    qdata.trajectory->getCovariance(*gn_solver.get(),steam::Time(future_stamp));
    //    T_q_0.setCovariance(cov);

    *ba_pose->mutable_t_q_0() << T_q_0;
    future_stamp += interval;
  }

  // insert into the vertex.
  auto live_vertex = graph->at(*qdata.vid_odo);
  auto run = graph->run((*qdata.vid_odo).majorId());
  std::string stream_name("/results/refined_vo");
  run->registerVertexStream(stream_name);
  live_vertex->insert<decltype(status_msg)>(stream_name, status_msg,
                                            *qdata.stamp);
}
#endif

void StereoWindowOptimizationModule::updateGraphImpl(QueryCache &qdata0,
                                                     const Graph::Ptr &graph,
                                                     VertexId) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);
  if (qdata.landmark_map.valid() == false ||
      qdata.pose_map.valid() == false || qdata.odo_success.valid() == false ||
      *qdata.odo_success == false) {
    return;
  }

  if (window_config_->save_trajectory) {
    throw std::runtime_error{
        "trajectory saving untested - windowed optimization"};
#if false
    saveTrajectory(qdata, graph);
#endif
  }

  auto &lm_map = *qdata.landmark_map;
  auto &poses = *qdata.pose_map;

  // if we used the backup LM solver, cast that instead of the main one.
  auto gn_solver =
      backup_lm_solver_used_
          ? std::dynamic_pointer_cast<steam::GaussNewtonSolver>(
                backup_lm_solver_)
          : std::dynamic_pointer_cast<steam::GaussNewtonSolver>(solver_);

  if (gn_solver == nullptr) {
    LOG(ERROR) << "This solver does not derive from The GaussNewtonSolverBase!";
    return;
  } else if (window_config_->solver_type == "LevenburgMarquardt" ||
             backup_lm_solver_used_) {
    // we need to explicitly ask the LM solver to update the covariance, but
    // this may throw
    auto lm_solver =
        std::dynamic_pointer_cast<steam::LevMarqGaussNewtonSolver>(gn_solver);
    // try {
    //   lm_solver->solveCovariances();
    // } catch (std::runtime_error &e) {
    //   LOG(ERROR)
    //       << "StereoWindowOptimizationModule: Couldn't solve for covariance "
    //          "in LM solver!"
    //       << std::endl
    //       << e.what();
    // }
  }

  // update the edges in the graph.
  Eigen::Matrix<double, 6, 6> zero_matrix;
  zero_matrix.setZero();
  auto pose_a_itr = poses.begin();
  auto pose_b_itr = poses.begin();
  pose_b_itr++;
  for (; pose_b_itr != poses.end() && pose_a_itr != poses.end();
       pose_b_itr++, pose_a_itr++) {
    lgmath::se3::Transformation T_a_0 =
        pose_a_itr->second.tf_state_var->value();
    lgmath::se3::Transformation T_b_0 =
        pose_b_itr->second.tf_state_var->value();
    if (pose_b_itr->second.isLocked() == false) {
      if (pose_a_itr->first.majorId() != qdata.vid_odo->majorId()) {
        continue;
      }
      auto e_id =
          EdgeId(pose_a_itr->first, pose_b_itr->first);
      if (graph->contains(e_id)) {
        auto e = graph->at(e_id);
        lgmath::se3::TransformationWithCovariance T_b_a = T_b_0 / T_a_0;
        // if (pose_a_itr->second.isLocked() == false) {
        //   std::vector<steam::StateKey> pose_keys{
        //       pose_a_itr->second.tf_state_var->key(),
        //       pose_b_itr->second.tf_state_var->key()};
        //   auto Cov_a0a0_b0b0 = gn_solver->queryCovarianceBlock(pose_keys);
        //   auto Tadj_b_a = T_b_a.adjoint();
        //   auto correlation = Tadj_b_a * Cov_a0a0_b0b0.at(0, 1);
        //   auto Cov_ba_ba =
        //       Cov_a0a0_b0b0.at(1, 1) - correlation - correlation.transpose() +
        //       Tadj_b_a * Cov_a0a0_b0b0.at(0, 0) * Tadj_b_a.transpose();
        //   T_b_a.setCovariance(Cov_ba_ba);
        // } else {
        //   T_b_a.setCovariance(gn_solver->queryCovariance(
        //       pose_b_itr->second.tf_state_var->key()));
        // }
        e->setTransform(T_b_a);
      } else {
        LOG(WARNING) << "Trying to update covariance of edge " << e_id
                     << ", which doesnt exist!";
        LOG(WARNING) << "++++++++++++++++++ Current Optimization problem "
                        "++++++++++++++++++++++++++++";
        for (auto itr = poses.begin(); itr != poses.end(); ++itr) {
          LOG(WARNING) << itr->first << " " << itr->second.isLocked();
        }
        LOG(WARNING) << "++++++++++++++++++ Current Optimization problem "
                        "++++++++++++++++++++++++++++";
      }
    }
  }

  VertexId first_unlocked;
  bool found_first_unlocked = false;
  // update the velocities in the graph
  for (auto &pose : poses) {
    if (pose.second.isLocked() == false) {
      auto v = graph->at(pose.first);
      auto locked_vel_msg = v->retrieve<VelocityMsg>(
            "velocities", "vtr_common_msgs/msg/Velocity");
      if (locked_vel_msg == nullptr) {
        std::stringstream err;
        err << "Couldn't retrieve velocity from vertex data!";
        CLOG(ERROR, "stereo.windowed_recall") << err.str();
        throw std::runtime_error{err.str()};
      }
      auto locked_msg = locked_vel_msg->sharedLocked();
      auto v_vel = locked_msg.get().getDataPtr();

      auto new_velocity = pose.second.velocity->value();
      v_vel->translational.x = new_velocity(0, 0);
      v_vel->translational.y = new_velocity(1, 0);
      v_vel->translational.z = new_velocity(2, 0);
      v_vel->rotational.x = new_velocity(3, 0);
      v_vel->rotational.y = new_velocity(4, 0);
      v_vel->rotational.z = new_velocity(5, 0);


    // fill the velocities
      std::string vel_str = "velocities";
      using Vel_Msg = storage::LockableMessage<VelocityMsg>;
      auto vel_msg =
              std::make_shared<Vel_Msg>(v_vel, v->vertexTime());
      v->insert<VelocityMsg>(vel_str, "vtr_common_msgs/msg/Velocity", vel_msg);


      if (!found_first_unlocked) {
        first_unlocked = pose.first;
        found_first_unlocked = true;
      }
    }
  }

  // Update the landmarks in the graph
  std::map<VertexId, std::shared_ptr<vtr_messages::msg::RigLandmarks>>
      landmark_msgs;

  for (auto &landmark : lm_map) {
      auto vid = landmark.first.vid;
    // VertexId vid = graph->fromPersistent(
    //     messages::copyPersistentId(landmark.first.persistent));

    if (!landmark_msgs.count(vid)) {
      auto v = graph->at(vid);

      std::string lm_stream_name = "stereo_landmarks"; //#TODO: make rig_name configurable
      auto locked_landmark_msg = v->retrieve<vtr_messages::msg::RigLandmarks>(
              lm_stream_name, "vtr_messages/msg/RigLandmarks");
      auto locked_msg = locked_landmark_msg->sharedLocked();
      auto v_lms = locked_msg.get().getDataPtr();
      if (v_lms == nullptr) {
        std::stringstream err;
        err << "Landmarks at " << v->id() << " for " << "stereo"
            << " could not be loaded!";
        LOG(ERROR) << err.str();
        throw std::runtime_error{err.str()};
      }


      // auto v_lms = v->retrieve<vtr_messages::msg::RigLandmarks>(
      //     "front_xb3_landmarks");
      landmark_msgs.emplace(std::make_pair(vid, v_lms));
    }

    if (landmark.second.observations.size() >
        landmark.second.num_vo_observations) {
      landmark_msgs.at(vid)
          ->channels[landmark.first.channel]
          .num_vo_observations[landmark.first.index] =
          landmark.second.observations.size();
    }
    // if this is a valid, unlocked landmark, then update its point/cov in the
    // graph.
    if (landmark.second.steam_lm != nullptr &&
        !landmark.second.steam_lm->locked() &&
        landmark.second.observations.size() > 1) {
      Eigen::Vector3d steam_point =
          landmark.second.steam_lm->value().hnormalized();

      landmark_msgs.at(vid)
          ->channels[landmark.first.channel]
          .points[landmark.first.index]
          .x = steam_point[0];
      landmark_msgs.at(vid)
          ->channels[landmark.first.channel]
          .points[landmark.first.index]
          .y = steam_point[1];
      landmark_msgs.at(vid)
          ->channels[landmark.first.channel]
          .points[landmark.first.index]
          .z = steam_point[2];

      // check validity on the landmark, but only if the point was valid in the
      // first place
      if (landmark_msgs.at(vid)
              ->channels[landmark.first.channel]
              .valid[landmark.first.index]) {
        landmark_msgs.at(vid)
            ->channels[landmark.first.channel]
            .valid[landmark.first.index] = isLandmarkValid(steam_point);
      }
#if 0
      /*
            // TODO: This is sooper dooper slow bud.
            // if this is the first unlocked pose.
            if(landmark.first.vertex == first_unlocked) {
              auto cov =
         gn_solver->queryCovariance(landmark.second.steam_lm->key()); auto
         *robochunk_cov = landmark.second.covariance; auto landmark_offset =
         landmark.first.index * 9;
              // Todo Eigen map for readability?
              //Eigen::Map<Eigen::Matrix<double,3,3> > ...

              for(int row = 0; row < 3; ++row) {
                for(int col = 0; col < 3; ++col) {
                  robochunk_cov->Set(landmark_offset + row*3 + col,
         cov(row,col));
                }
              }
            }
      */
#endif
    }
  }

  for (auto &msg : landmark_msgs) {
    auto v = graph->at(msg.first);
    // v->replace("front_xb3_landmarks", *(msg.second), v->vertexTime());


    std::string lm_str = "stereo_landmarks";
    using LM_Msg = storage::LockableMessage<RigLandmarksMsg>;
    auto lm_msg =
        std::make_shared<LM_Msg>(msg.second, v->vertexTime());
    v->insert<RigLandmarksMsg>(lm_str, "vtr_messages/msg/RigLandmarks", lm_msg);
      



  }
  // reset to remove any old data from the problem setup
  resetProblem();
}

}  // namespace vision
}  // namespace vtr
