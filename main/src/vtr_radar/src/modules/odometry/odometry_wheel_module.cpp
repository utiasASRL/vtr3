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
 * \file odometry_wheel_module.cpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_wheel_module.hpp"

#include "vtr_radar/utils/nanoflann_utils.hpp"

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
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryWheelModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  //
  config->potentially_slipping = node->declare_parameter<bool>(param_prefix + ".potentially_slipping", true);
  config->estimate_bias = node->declare_parameter<bool>(param_prefix + ".estimate_bias", true);
  //
  config->bias_alpha = node->declare_parameter<double>(param_prefix + ".bias_alpha", 0.005);
  config->min_time_bias_count = node->declare_parameter<double>(param_prefix + ".min_time_bias_count", 200.0);
  config->wheel_parameter = node->declare_parameter<double>(param_prefix + ".wheel_parameter", 5e-04);
  //
  config->max_trans_vel_diff = node->declare_parameter<float>(param_prefix + ".max_trans_vel_diff", config->max_trans_vel_diff);
  config->max_rot_vel_diff = node->declare_parameter<float>(param_prefix + ".max_rot_vel_diff", config->max_rot_vel_diff);
  config->max_trans_diff = node->declare_parameter<float>(param_prefix + ".max_trans_diff", config->max_trans_diff);
  config->max_rot_diff = node->declare_parameter<float>(param_prefix + ".max_rot_diff", config->max_rot_diff);
  //
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OdometryWheelModule::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if (!qdata.gyro_msgs || !qdata.wheel_meas) {
    CLOG_IF(!qdata.gyro_msgs, WARNING, "radar.odometry_wheel") << "No gyro messages found, cannot run odometry.";
    CLOG_IF(!qdata.wheel_meas, WARNING, "radar.odometry_wheel") << "No wheel measurements found, cannot run odometry.";
    if (qdata.preprocessed_point_cloud) {
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
      cart2pol(*undistorted_point_cloud);
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    }
    *qdata.odo_success = false;
    return;
  }

  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "radar.odometry_wheel") << "First frame.";
    // clang-format off
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.T_r_m_odo_radar.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    qdata.w_m_r_in_r_odo_radar.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    qdata.timestamp_odo_radar.emplace(*qdata.stamp);
    //
    qdata.T_r_m_odo_prior.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo_prior.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    // Initialize timestamp equal to the end of the first frame
    const auto &query_points = *qdata.raw_point_cloud;
    const auto compare_time = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
    qdata.timestamp_prior.emplace(std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp);
    // clang-format on
  }

  CLOG(DEBUG, "radar.odometry_wheel") << "Retrieve input data and setup evaluators.";

  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_s_r_gyro = *qdata.T_s_r_gyro;
  const auto &T_s_r_wheel = *qdata.T_s_r_wheel;
  const auto &beta = *qdata.beta;

  // Load in prior parameters
  const auto &T_r_m_odo_prior = *qdata.T_r_m_odo_prior;
  const auto &w_m_r_in_r_odo_prior = *qdata.w_m_r_in_r_odo_prior;
  const auto &timestamp_prior = *qdata.timestamp_prior;

  const double bias_alpha = config_->estimate_bias ? config_->bias_alpha : 0.0;
  const double min_bias_init_count = config_->estimate_bias ? config_->min_time_bias_count : 0.0;

  // Load in measurements
  auto &gyro_msgs = *qdata.gyro_msgs;
  auto &wheel_meas = *qdata.wheel_meas;

  // clang-format off
  // Create robot to sensor transform variables, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;
  const auto T_s_r_wh_var = SE3StateVar::MakeShared(T_s_r_wheel);
  T_s_r_wh_var->locked() = true;
  const auto T_s_r_gyro_var = SE3StateVar::MakeShared(T_s_r_gyro);
  T_s_r_gyro_var->locked() = true;

  int ptr_ang = 0;
  int ptr_pulse = 0;
  if (qdata.sliding_map_odo) {
    // add last gyro and wheel measurement to the beginning of the current measurement list
    gyro_msgs.insert(gyro_msgs.begin(), last_gyro_msg);
    wheel_meas.insert(wheel_meas.begin(), last_wheel_meas);

    CLOG(DEBUG, "radar.odometry_wheel") << "Added last gyro and wheel measurement to the beginning of the current measurement list.";

    // Advance ptr_ang until gyro_msgs[ptr_ang].header.stamp matches last_gyro_stamp
    while (ptr_ang + 1 < (int)gyro_msgs.size() &&
      rclcpp::Time(gyro_msgs[ptr_ang].header.stamp).nanoseconds() < rclcpp::Time(last_gyro_stamp).nanoseconds()) {
      ptr_ang++;
      CLOG(DEBUG, "radar.odometry_wheel") << "Advancing gyro pointer to index " << ptr_ang;
    }
    // Advance ptr_pulse until wheel_meas[ptr_pulse].first matches last_wheel_stamp
    while (ptr_pulse + 1 < (int)wheel_meas.size() &&
      wheel_meas[ptr_pulse].first.nanoseconds() < rclcpp::Time(last_wheel_stamp).nanoseconds()) {
      ptr_pulse++;
      CLOG(DEBUG, "radar.odometry_wheel") << "Advancing wheel pointer to index " << ptr_pulse;
    }
  }

  // Initialize trajectory
  const_vel::Interface::Ptr trajectory = nullptr;
  Eigen::Matrix<double, 6, 1> traj_qc_diag = Eigen::Matrix<double, 6, 1>::Ones();
  traj_qc_diag << 1.0, 0.001, 0.001, 0.001, 0.001, 1.0; // to do: move to config
  trajectory = const_vel::Interface::MakeShared(traj_qc_diag);

  if (qdata.sliding_map_odo) {
    auto prev_T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo_prior);
    prev_T_r_m_var->locked() = true;
    auto prev_w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo_prior);
    trajectory->add(Time(timestamp_prior), prev_T_r_m_var, prev_w_m_r_in_r_var);
  }

  // Initialize variables for odometry estimation
  double dist = 0.0;
  Eigen::Vector2d delta_pos = Eigen::Vector2d::Zero();

  // Initialize pose we add to the trajectory
  EdgeTransform T_last;

  // Save unique measurement times
  std::vector<int64_t> timestamps;
  for (const auto &gyro_msg : gyro_msgs) {
    timestamps.push_back(static_cast<int64_t>(rclcpp::Time(gyro_msg.header.stamp).nanoseconds()));
  }
  for (const auto &wheel_set : wheel_meas) {
    timestamps.push_back(static_cast<int64_t>(wheel_set.first.nanoseconds()));
  }
  std::sort(timestamps.begin(), timestamps.end());
  timestamps.erase(std::unique(timestamps.begin(), timestamps.end()), timestamps.end());

  if (timestamps.size() < 2 ||
      ptr_ang + 1 >= (int)gyro_msgs.size() ||
      ptr_pulse + 1 >= (int)wheel_meas.size()) {
    CLOG(WARNING, "radar.odometry_wheel") << "Insufficient measurements to run odometry (timestamps=" << timestamps.size()
      << ", gyro=" << gyro_msgs.size() << ", wheel=" << wheel_meas.size() << ").";
    *qdata.odo_success = false;
    return;
  }

  // Initialize current and next timestamps
  rclcpp::Time curr_time(timestamps[0]);
  rclcpp::Time next_time(timestamps[1]);
  //
  rclcpp::Time curr_gyro_time(gyro_msgs[ptr_ang].header.stamp);
  rclcpp::Time curr_wheel_time(wheel_meas[ptr_pulse].first);
  rclcpp::Time next_gyro_time(gyro_msgs[ptr_ang + 1].header.stamp);
  rclcpp::Time next_wheel_time(wheel_meas[ptr_pulse + 1].first);

  auto query_time = static_cast<int64_t>(query_stamp);

  Eigen::Matrix<double, 6, 1> w_last = Eigen::Matrix<double, 6, 1>::Zero();

  // estimation loop
  Eigen::Matrix<double, 6, 1> w_robot = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < (int)timestamps.size() - 1; ++i) {
    curr_time = rclcpp::Time(timestamps[i]);
    next_time = rclcpp::Time(timestamps[i + 1]);
    //
    curr_gyro_time = rclcpp::Time(gyro_msgs[ptr_ang].header.stamp);
    curr_wheel_time = wheel_meas[ptr_pulse].first;
    next_gyro_time = rclcpp::Time(gyro_msgs[ptr_ang + 1].header.stamp);
    next_wheel_time = wheel_meas[ptr_pulse + 1].first;

    if (timestamps[i] < next_est_stamp) {
      if (next_time.seconds() >= next_gyro_time.seconds() && ptr_ang + 2 < (int)gyro_msgs.size()) ptr_ang++;
      if (next_time.seconds() >= next_wheel_time.seconds() && ptr_pulse + 2 < (int)wheel_meas.size()) ptr_pulse++;
      continue;
    }

    // Scalar z-axis gyro (radar operates in 2D plane)
    const double curr_gyro_z = gyro_msgs[ptr_ang].angular_velocity.z;
    const double next_gyro_z = gyro_msgs[ptr_ang + 1].angular_velocity.z;
    const double diff_ang_vel = next_gyro_z - curr_gyro_z;
    const double diff_ang_vel_time = next_gyro_time.seconds() - curr_gyro_time.seconds();
    const double t0_ang = curr_time.seconds() - curr_gyro_time.seconds();
    const double t1_ang = next_time.seconds() - curr_gyro_time.seconds();
    const double t_mid = (t0_ang + t1_ang) / 2;

    double mean_ang_vel = 0.0;
    if (diff_ang_vel_time == 0) {
      mean_ang_vel = (curr_gyro_z + next_gyro_z) / 2;
    } else {
      mean_ang_vel = curr_gyro_z + diff_ang_vel * t_mid / diff_ang_vel_time;
    }

    if (bias_init) mean_ang_vel -= gyr_bias;

    const double dt = next_time.seconds() - curr_time.seconds();
    double ang = -mean_ang_vel * dt;  // negative sign: radar convention

    const int diff_pulse_count = wheel_meas[ptr_pulse + 1].second - wheel_meas[ptr_pulse].second;
    double diff_pulse_time = next_wheel_time.seconds() - curr_wheel_time.seconds();

    if (diff_pulse_count != 0) {
      const double t0_pulse = curr_time.seconds() - curr_wheel_time.seconds();
      const double t1_pulse = next_time.seconds() - curr_wheel_time.seconds();
      if (diff_pulse_time <= 1e-6) diff_pulse_time = 1e-6;
      dist = (t1_pulse - t0_pulse) * diff_pulse_count / diff_pulse_time;
      dist *= config_->wheel_parameter;
      total_dist += dist;

      // 2D arc-length motion model
      Eigen::Vector2d delta_pos_local;
      if (ang != 0) {
        const double radius_arc = dist / ang;
        delta_pos_local[1] = radius_arc * std::sin(ang);
        delta_pos_local[0] = -radius_arc * (1 - std::cos(ang));
      } else {
        delta_pos_local[1] = dist;
        delta_pos_local[0] = 0.0;
      }
      Eigen::Matrix2d R;
      R << std::cos(current_theta), -std::sin(current_theta),
           std::sin(current_theta),  std::cos(current_theta);
      delta_pos = R * delta_pos_local;
    } else {
      delta_pos = Eigen::Vector2d::Zero();

      if (!config_->potentially_slipping) ang = 0.0;

      const int64_t gyro_time_ns = static_cast<int64_t>(curr_gyro_time.nanoseconds());
      if (config_->estimate_bias && last_bias_time < gyro_time_ns) {
        if (bias_init) {
          const double gyro_data = (curr_gyro_z + next_gyro_z) / 2;
          if (std::abs(gyro_data - gyr_bias) < 2 * std::abs(gyr_bias)) {
            gyr_bias = gyr_bias * (1 - bias_alpha) + gyro_data * bias_alpha;
            max_bias = std::max(max_bias, std::abs(gyr_bias));
          }
        } else {
          gyr_bias += curr_gyro_z;
          bias_counter++;
          if (bias_counter >= min_bias_init_count) {
            bias_init = true;
            gyr_bias /= bias_counter;
          }
        }
        last_bias_time = gyro_time_ns;
      }
    }

    current_p += delta_pos;
    current_theta += ang;

    // Instantaneous velocity at curr_time via linear interpolation
    const double t_interp = curr_time.seconds() - curr_gyro_time.seconds();
    const double ratio = (diff_ang_vel_time > 0.0) ? std::clamp(t_interp / diff_ang_vel_time, 0.0, 1.0) : 0.0;
    const double instant_gyro_z = curr_gyro_z + diff_ang_vel * ratio;

    const double alpha = 0.2;
    const double raw_pulse_rate = (diff_pulse_time > 1e-6) ? (diff_pulse_count / diff_pulse_time) : 0.0;
    if (diff_pulse_time > 1e-6) {
      filtered_pulse_rate_ = (alpha * raw_pulse_rate) + ((1.0 - alpha) * filtered_pulse_rate_);
    }

    const double v_fwd = filtered_pulse_rate_ * config_->wheel_parameter;
    const Eigen::Vector3d v_wheel(0.0, v_fwd, 0.0);
    const Eigen::Vector3d omega_wheel(0.0, 0.0, -instant_gyro_z);  // negative sign: radar convention
    const Eigen::Matrix3d R_r_wheel = T_s_r_wheel.matrix().inverse().block<3, 3>(0, 0).cast<double>();
    w_robot.block<3, 1>(0, 0) = R_r_wheel * v_wheel;
    w_robot.block<3, 1>(3, 0) = R_r_wheel * omega_wheel;

    if (next_time.seconds() >= next_wheel_time.seconds() && ptr_pulse + 2 < (int)wheel_meas.size()) ptr_pulse++;
    if (next_time.seconds() >= next_gyro_time.seconds() && ptr_ang + 2 < (int)gyro_msgs.size()) ptr_ang++;

  } // end estimation loop

  // Build last estimated pose from 2D state
  int64_t last_est_time = curr_time.nanoseconds();
  Eigen::Matrix4d last_trans = Eigen::Matrix4d::Identity();
  last_trans.block<2, 2>(0, 0) << std::cos(current_theta), -std::sin(current_theta),
                                   std::sin(current_theta),  std::cos(current_theta);
  last_trans.block<2, 1>(0, 3) << current_p.x(), current_p.y();
  T_last = EdgeTransform(last_trans, Eigen::Matrix<double, 6, 6>::Identity() * 1e-3);
  w_last = w_robot;

  next_est_stamp = next_time.nanoseconds();
  last_gyro_stamp = curr_gyro_time.nanoseconds();
  last_wheel_stamp = curr_wheel_time.nanoseconds();

  last_gyro_msg = gyro_msgs.back();
  last_wheel_meas = wheel_meas.back();

  // Transform estimated pose to map frame at last time
  const auto T_m_s_wheel_last = SE3StateVar::MakeShared(T_last);
  auto T_r_m_eval_last = inverse(compose(T_m_s_wheel_last, T_s_r_wh_var));
  auto vel_last = VSpaceStateVar<6>::MakeShared(-w_last);

  bool estimate_reasonable = true;
  // Check if change between initial and final velocity is reasonable
  const auto &w_m_r_in_r_prev = *qdata.w_m_r_in_r_odo_prior;
  const auto &w_m_r_in_r_new = vel_last->value();
  const auto vel_diff = w_m_r_in_r_new - w_m_r_in_r_prev;
  const auto trans_vel_diff_norm = vel_diff.head<3>().norm();
  const auto rot_vel_diff_norm = vel_diff.tail<3>().norm();

  const auto T_r_m_new = T_r_m_eval_last->value();
  const auto diff_T = (T_r_m_odo_prior * T_r_m_new.inverse()).vec();
  const auto diff_T_trans = diff_T.head<3>().norm();
  const auto diff_T_rot = diff_T.tail<3>().norm();

  CLOG(DEBUG, "radar.odometry_wheel") << "Current transformation difference: " << diff_T.transpose();
  CLOG(DEBUG, "radar.odometry_wheel") << "Diff_T_trans: " << diff_T_trans << " , Diff_T_rot: " << diff_T_rot;
  CLOG(DEBUG, "radar.odometry_wheel") << "Current velocity difference: " << vel_diff.transpose();
  CLOG(DEBUG, "radar.odometry_wheel") << "Translational velocity diff: " << trans_vel_diff_norm << " , Rotational velocity diff: " << rot_vel_diff_norm;

  if (trans_vel_diff_norm > config_->max_trans_vel_diff || rot_vel_diff_norm > config_->max_rot_vel_diff) {
    CLOG(WARNING, "radar.odometry_wheel") << "Velocity difference between initial and final is too large."
      << " Translational: " << trans_vel_diff_norm << " Rotational: " << rot_vel_diff_norm;
    estimate_reasonable = false;
  }
  if (diff_T_trans > config_->max_trans_diff) {
    CLOG(WARNING, "radar.odometry_wheel") << "Translation difference too large. Transform difference vector: " << diff_T.transpose();
    estimate_reasonable = false;
  }
  if (diff_T_rot > config_->max_rot_diff) {
    CLOG(WARNING, "radar.odometry_wheel") << "Rotation difference too large. Transform difference vector: " << diff_T.transpose();
    estimate_reasonable = false;
  }

  if (estimate_reasonable) {
    // Add estimated state at end of frame as knot
    trajectory->add(Time(last_est_time), T_r_m_eval_last, VSpaceStateVar<6>::MakeShared(vel_last->value()));

    // Interpolate state at query time
    const auto T_r_m_query = trajectory->getPoseInterpolator(Time(query_time));
    const auto w_m_r_in_r_query = trajectory->getVelocityInterpolator(Time(query_time));
    const auto T_s_m_query = compose(T_s_r_var, T_r_m_query);

    *qdata.T_r_m_odo = EdgeTransform(T_r_m_query->value(), Eigen::Matrix<double, 6, 6>::Identity() * 1e-3);
    *qdata.T_r_m_odo_radar = EdgeTransform(T_r_m_query->value(), Eigen::Matrix<double, 6, 6>::Identity() * 1e-3);
    *qdata.w_m_r_in_r_odo = w_m_r_in_r_query->value();
    *qdata.w_m_r_in_r_odo_radar = w_m_r_in_r_query->value();
    *qdata.timestamp_odo = query_stamp;
    *qdata.timestamp_odo_radar = query_stamp;

    if (qdata.preprocessed_point_cloud) {
      CLOG(DEBUG, "radar.odometry_wheel") << "localizing: processing point cloud with estimated pose at query time";
      const auto &query_points = *qdata.preprocessed_point_cloud;

      pcl::PointCloud<PointWithInfo> aligned_points(query_points);
      const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());
      auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

      // Copy points before any correction
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (unsigned i = 0; i < query_points.size(); ++i) {
        aligned_mat.block<4, 1>(0, i) = query_mat.block<4, 1>(0, i);
      }

      // Radar-specific: Doppler frequency correction
      if (beta != 0) {
        const auto T_r_m_var = SE3StateVar::MakeShared(*qdata.T_r_m_odo);
        const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(*qdata.w_m_r_in_r_odo);
        Time knot_time(static_cast<int64_t>(query_stamp));
        trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);

#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
        for (unsigned i = 0; i < query_points.size(); ++i) {
          const auto &qry_time = query_points[i].timestamp;
          const auto &up_chirp = query_points[i].up_chirp;
          const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(qry_time));
          const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);
          const auto w_m_s_in_s = w_m_s_in_s_intp_eval->evaluate().matrix().cast<float>();
          const Eigen::Vector3f v_m_s_in_s = w_m_s_in_s.block<3, 1>(0, 0);
          Eigen::Vector3f abar = aligned_mat.block<3, 1>(0, i);
          abar.normalize();
          if (up_chirp) {
            aligned_mat.block<3, 1>(0, i) -= beta * abar * abar.transpose() * v_m_s_in_s;
          } else {
            aligned_mat.block<3, 1>(0, i) += beta * abar * abar.transpose() * v_m_s_in_s;
          }
        }
      }

      // Per-point trajectory undistortion
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (unsigned i = 0; i < query_points.size(); ++i) {
        const auto &qry_time = query_points[i].timestamp;
        const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
        const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
        const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
        aligned_mat.block<4, 1>(0, i) = T_m_s * aligned_mat.block<4, 1>(0, i);
        aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
      }

      // Bring back to sensor frame
      const auto T_s_m = T_s_m_query->evaluate().matrix().inverse().cast<float>();
      aligned_mat = T_s_m * aligned_mat;
      aligned_norms_mat = T_s_m * aligned_norms_mat;

      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
      cart2pol(*undistorted_point_cloud);
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    } else {
      CLOG(DEBUG, "radar.odometry_wheel") << "not localizing: skip point cloud processing, saving estimated pose and velocity";
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>();
      undistorted_point_cloud->resize(1);
      cart2pol(*undistorted_point_cloud);
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    }

    if (!first_frame_) {
      EdgeTransform T_r_m(*qdata.T_r_m_odo, Eigen::Matrix<double, 6, 6>::Identity() * 1e-3);
      auto &sliding_map_odo = *qdata.sliding_map_odo;
      *qdata.T_r_v_odo = T_r_m * sliding_map_odo.T_vertex_this().inverse();
      *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
    }

    *qdata.T_r_m_odo_prior = T_r_m_eval_last->value();
    *qdata.w_m_r_in_r_odo_prior = vel_last->value();
    *qdata.timestamp_prior = last_est_time;

    *qdata.odo_success = true;

  } else {
    CLOG(WARNING, "radar.odometry_wheel") << "Wheel-Odometer Odometry failed";
    if (qdata.preprocessed_point_cloud) {
      CLOG(DEBUG, "radar.odometry_wheel") << "using preprocessed point cloud";
      const auto &query_points = *qdata.preprocessed_point_cloud;
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(query_points);
      cart2pol(*undistorted_point_cloud);
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    } else {
      CLOG(DEBUG, "radar.odometry_wheel") << "no point cloud saved";
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>();
      undistorted_point_cloud->resize(1);
      cart2pol(*undistorted_point_cloud);
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    }
    *qdata.odo_success = false;
  }

  first_frame_ = false;
  // clang-format on
}

}  // namespace radar
}  // namespace vtr