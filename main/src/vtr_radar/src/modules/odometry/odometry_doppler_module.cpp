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
 * \file odometry_doppler_module.cpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_doppler_module.hpp"

#include "vtr_radar/utils/nanoflann_utils.hpp"
#include "vtr_common/conversions/se2_to_se3.hpp"

namespace vtr {
namespace radar {

namespace {

template <class PointT>
void cart2pol(pcl::PointCloud<PointT> &point_cloud) {
  for (auto &p : point_cloud) {
    p.rho = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = std::atan2(std::sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = std::atan2(p.y, p.x);
  }
}

}  // namespace

using namespace tactic;
using namespace steam;
using namespace steam::se2;
using namespace steam::traj;
using namespace steam::vspace;
using namespace common::conversions;

auto OdometryDopplerModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();

  // Undistortion trajectory parameters
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 3) {
    std::string err{"Qc diagonal malformed. Must be 3 elements!"};
    CLOG(ERROR, "radar.odometry_icp") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2];
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  config->zero_velocity_threshold = node->declare_parameter<double>(param_prefix + ".zero_velocity_threshold", 0.1);

  return config;
}

void OdometryDopplerModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any radar data (was populated by gyro)
  if(!qdata.radar_data)
  {
    return;
  }

  if (!qdata.doppler_scan || !qdata.vel_meas) {
    CLOG(ERROR, "radar.odometry_doppler") << "Doppler information not provided!";
    throw std::runtime_error("Doppler information not provided!");
  }

  if (!qdata.gyro_msgs) {
    CLOG(ERROR, "radar.odometry_doppler") << "Gyro measurements not provided!";
    throw std::runtime_error("Gyro measurements not provided!");
  }

  if (!qdata.sliding_map_odo) {
    // Initialize all variables
    CLOG(INFO, "radar.odometry_doppler") << "First frame, simply return.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.T_r_m_odo_radar.emplace(EdgeTransform(true));

    // Initialize prior values
    qdata.T_r_m_odo_prior.emplace(lgmath::se3::Transformation());
    qdata.w_m_r_in_r_odo_prior.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    qdata.timestamp_prior.emplace(*qdata.stamp);
    scan_stamp_prev_ = *qdata.stamp;

    // Initialize gt variables for passing to next frame
    qdata.T_s_world_gt_prev.emplace(lgmath::se3::Transformation());

    *qdata.odo_success = true;
    // clang-format on

    // This is the first odometry frame
    if(qdata.first_frame)
      *qdata.first_frame = true;
    else
      qdata.first_frame.emplace(true); // reset first frame - this is the first frame! Gyro could have run before though

    return;
  }

  CLOG(DEBUG, "radar.odometry_doppler")
      << "Retrieve input data and setup evaluators.";

  // Inputs (these are all 3D to be consistent with other pipelines)
  const auto &scan_stamp = *qdata.stamp;
  const auto &pointcloud = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &beta = *qdata.beta;
  const auto &T_r_m_prev = *qdata.T_r_m_odo_prior;
  // Load prior that contain hand-over info
  const auto &preint_prev_leftover = *qdata.T_s_world_gt_prev;
  auto &w_m_r_in_r_odo_prior = *qdata.w_m_r_in_r_odo_prior;
  auto &timestamp_prev = *qdata.timestamp_prior; // This is the timestamp of the last gyro measurement from the previous frame

  // Extract timestamps in seconds
  long double scan_stamp_s = static_cast<long double>(scan_stamp) / 1e9;
  long double timestamp_prev_s = static_cast<long double>(timestamp_prev) / 1e9;
  long double scan_stamp_prev_s = static_cast<long double>(scan_stamp_prev_) / 1e9;

  // Preintegrate yaw gyro
  // Initialize rotation that has been built up since last scan stamp
  Eigen::Matrix2d delta_C = lgmath::so2::vec2rot(preint_yaw_);
  double yaw_dt = 0.0;
  bool past_scan_time = false;
  double yaw_rate_avg = 0.0;
  for (const auto &gyro_msg : *qdata.gyro_msgs) {
    // Load in gyro yaw rate
    const double yaw_rate_curr = gyro_msg.angular_velocity.z;
    const double yaw_rate_use = (yaw_rate_curr + yaw_rate_prev_) / 2.0;
    // Load timestamp
    const rclcpp::Time gyro_stamp(gyro_msg.header.stamp);
    const auto gyro_stamp_time = gyro_stamp.seconds();

    // Check if we're just before the scan stamp
    if (gyro_stamp_time > scan_stamp_s && !past_scan_time) {
      const auto temp_dt = scan_stamp_s - timestamp_prev_s;
      const auto temp_delta_C = delta_C * lgmath::so2::vec2rot(-yaw_rate_use * temp_dt);
      yaw_dt = -lgmath::so2::rot2vec(temp_delta_C);

      // Reset delta_C
      delta_C = Eigen::Matrix2d::Identity();
      timestamp_prev_s = scan_stamp_s;
      past_scan_time = true;
    }

    // Integrate
    const double dt  = gyro_stamp_time - timestamp_prev_s;
    delta_C = delta_C * lgmath::so2::vec2rot(-yaw_rate_use * dt);

    // Update previous timestamp
    timestamp_prev_s = gyro_stamp_time;
    yaw_rate_prev_ = yaw_rate_curr;
    yaw_rate_avg += yaw_rate_curr;
  }
  if (!qdata.gyro_msgs->empty()) {
    yaw_rate_avg /= qdata.gyro_msgs->size();
    yaw_rate_avg_prev_ = yaw_rate_avg;
  }

  // Handle case where we ran out of IMU measurements before reaching scan time
  if (!past_scan_time) {
    // Integrate to scan_time from previous timestamp only
    const auto temp_dt = scan_stamp_s - timestamp_prev_s;
    const auto temp_delta_C = delta_C * lgmath::so2::vec2rot(-yaw_rate_prev_ * temp_dt);
    yaw_dt = -lgmath::so2::rot2vec(temp_delta_C);

    // Now set up for next frame, with the previous timestamp rooted at scan_stamp
    delta_C = Eigen::Matrix2d::Identity();
    timestamp_prev_s = scan_stamp_s;
  }
  // Save latest state for future frame
  timestamp_prev = static_cast<int64_t>(timestamp_prev_s * 1e9);
  preint_yaw_ = lgmath::so2::rot2vec(delta_C);

  // Load in current velocities to be used in next frame
  const auto Ad_T_r_s = lgmath::se2::tranAd(T_s_r.inverse().toSE2().matrix());
  const Eigen::Vector2d linear_vel_s = *qdata.vel_meas;
  Eigen::Vector2d v_r_v_in_r = Ad_T_r_s.block<2,2>(0,0) * linear_vel_s;

  // Compute total translation change from last timestamp to current scan timestamp due to previous velocity
  Eigen::Vector2d r_dt = (v_r_v_in_r_prev_ + v_r_v_in_r) / 2 * (scan_stamp_s - scan_stamp_prev_s);
  Eigen::Matrix<double,3,1> varpi_dt(r_dt(0), r_dt(1), yaw_dt);

  if (varpi_dt.head<2>().norm() < config_->zero_velocity_threshold) {
    // If the velocity is very low, we assume no movement
    varpi_dt = Eigen::Matrix<double,3,1>::Zero();
  }

  // Compute total change in state
  Eigen::Matrix4d T_r_delta = lgmath::se3::vec2tran(vec2Dto3D(-varpi_dt));

  // Propgate state
  *qdata.T_r_v_odo = T_r_delta * *qdata.T_r_v_odo;
  *qdata.T_r_m_odo = T_r_delta * T_r_m_prev;
  *qdata.T_r_m_odo_radar = T_r_delta * T_r_m_prev;
  // Save T_r_m_new since we need both it and prev for undistortion
  const auto T_r_m_new = T_r_delta * T_r_m_prev;

  // Save new velocity
  // For our yaw rate, just use the middle gyro measurement
  // It doesn't affect pose so purely for velocity "estimation"
  const double yaw_r_v_in_r = qdata.gyro_msgs->at(qdata.gyro_msgs->size() / 2).angular_velocity.z;
  *qdata.w_v_r_in_r_odo = -Eigen::Matrix<double, 6, 1>(v_r_v_in_r(0), v_r_v_in_r(1), 0.0, 0.0, 0.0, yaw_r_v_in_r);
  w_m_r_in_r_odo_prior(0) = -v_r_v_in_r(0);
  w_m_r_in_r_odo_prior(1) = -v_r_v_in_r(1);
  *qdata.w_m_r_in_r_odo_prior = w_m_r_in_r_odo_prior;

  // Undistort pointcloud
  // Create trajectory for undistortion
  const_vel_se2::Interface::Ptr trajectory = const_vel_se2::Interface::MakeShared(config_->traj_qc_diag);
  // Add poses and velocities to trajectory
  const Eigen::Vector3d v_m_r_in_r_prev(-v_r_v_in_r_prev_(0), -v_r_v_in_r_prev_(1), 0.0);
  const Eigen::Vector3d v_m_r_in_r(-v_r_v_in_r(0), -v_r_v_in_r(1), 0.0);
  trajectory->add(Time(scan_stamp_prev_), SE2StateVar::MakeShared(T_r_m_prev.toSE2()),
                  VSpaceStateVar<3>::MakeShared(v_m_r_in_r_prev));
  trajectory->add(Time(scan_stamp), SE2StateVar::MakeShared(T_r_m_new.toSE2()),
                  VSpaceStateVar<3>::MakeShared(v_m_r_in_r));
  // Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE2StateVar::MakeShared(T_s_r.toSE2());
  T_s_r_var->locked() = true;
  // Compound transform for alignment (sensor to point map transform)
  Evaluable<lgmath::se2::Transformation>::ConstPtr T_r_m_eval = trajectory->getPoseInterpolator(scan_stamp);
  const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval));

  // Initialize deep copy of pointcloud for undistortion
  pcl::PointCloud<PointWithInfo> udist_pc(pointcloud);

  // Eigen matrix of original data (only shallow copy of ref clouds)
  auto points_mat = udist_pc.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
  auto norms_mat = udist_pc.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

  // Remove Doppler effect
  if (beta != 0) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < udist_pc.size(); ++i) {
      const auto &pt_time = udist_pc[i].timestamp;
      const auto &up_chirp = udist_pc[i].up_chirp;
      const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(pt_time));
      const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);
      const auto w_m_s_in_s = w_m_s_in_s_intp_eval->evaluate().matrix().cast<float>();
      // Still create 3D v_m_s_in_s but just with a 0 z component
      const Eigen::Vector3f v_m_s_in_s = Eigen::Vector3f(w_m_s_in_s(0), w_m_s_in_s(1), 0.0f);
      Eigen::Vector3f abar = points_mat.block<3, 1>(0, i);
      abar.normalize();
      // If up chirp azimuth, subtract Doppler shift
      if (up_chirp) {
        points_mat.block<3, 1>(0, i) -= beta * abar * abar.transpose() * v_m_s_in_s;
      } else {
        points_mat.block<3, 1>(0, i) += beta * abar * abar.transpose() * v_m_s_in_s;
      }
    }
  }
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
  for (unsigned i = 0; i < udist_pc.size(); i++) {
    const auto &pt_time = udist_pc[i].timestamp;
    const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(pt_time));
    const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
    // Transform to 3D for point manipulation
    const auto T_m_s = T_m_s_intp_eval->evaluate().toSE3().matrix().cast<float>();
    points_mat.block<4, 1>(0, i) = T_m_s * points_mat.block<4, 1>(0, i);
    norms_mat.block<4, 1>(0, i) = T_m_s * norms_mat.block<4, 1>(0, i);
  }

  // undistort the preprocessed pointcloud to eval state (at query timestamp)
  const auto T_s_m = T_m_s_eval->evaluate().toSE3().matrix().inverse().cast<float>();
  points_mat = T_s_m * points_mat;
  norms_mat = T_s_m * norms_mat;
  auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(udist_pc);
  cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
  qdata.undistorted_point_cloud = undistorted_point_cloud;

  // Save final quantities
  *qdata.timestamp_prior = timestamp_prev;
  scan_stamp_prev_ = scan_stamp;
  v_r_v_in_r_prev_ = v_r_v_in_r;
  *qdata.T_r_m_odo_prior = T_r_m_new;
  *qdata.odo_success = true;
}

}  // namespace radar
}  // namespace vtr