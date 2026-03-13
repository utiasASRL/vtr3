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
  const auto cov = node->declare_parameter<std::vector<double>>(param_prefix + ".wheel_cov", {1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
  if (cov.size() != 6) {
    throw std::runtime_error("wheel_cov must be a vector of size 6");
  }
  config->wheel_cov << cov[0], cov[1], cov[2], cov[3], cov[4], cov[5];
  //
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OdometryWheelModule::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "radar.odometry_wheel") << "First frame.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //    
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.T_r_m_odo_radar.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    // clang-format on
    frame_count = 0;
  }

  CLOG(DEBUG, "radar.odometry_wheel") << "Retrieve input data and setup evaluators.";
 
  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_s_r_gyro = *qdata.T_s_r_gyro;
  const auto &T_s_r_wheel = *qdata.T_s_r_wheel;
  const auto &timestamp_odo = *qdata.timestamp_odo;
  const auto &T_r_m_odo = *qdata.T_r_m_odo;
  const auto &w_m_r_in_r_odo = *qdata.w_m_r_in_r_odo;
  const auto &beta = *qdata.beta;

  double bias_alpha;
  double min_bias_init_count;
  if (config_->estimate_bias) {
    bias_alpha = config_->bias_alpha;
    min_bias_init_count = config_->min_time_bias_count;
  }

  auto &gyro_msgs = *qdata.gyro_msgs;
  auto &wheel_meas = *qdata.wheel_meas;
  if (frame_count > 0) {
    gyro_msgs.insert(gyro_msgs.begin(), last_gyro_msg);
    wheel_meas.insert(wheel_meas.begin(), last_wheel_meas);
  }

  // clang-format off
  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;
  const auto T_s_r_wh_var = SE3StateVar::MakeShared(T_s_r_wheel);
  T_s_r_wh_var->locked() = true;
  const auto T_s_r_gyro_var = SE3StateVar::MakeShared(T_s_r_gyro);
  T_s_r_gyro_var->locked() = true;

  // initialize trajectory
  const_vel::Interface::Ptr trajectory = nullptr;
  trajectory = const_vel::Interface::MakeShared();

  // add previous state to trajectory
  Time t_prev_time(static_cast<int64_t>(timestamp_odo));
  auto prev_T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo);
  auto prev_w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo);
  trajectory->add(t_prev_time, prev_T_r_m_var, prev_w_m_r_in_r_var); 

  const rclcpp::Time query_time(query_stamp);
  const rclcpp::Time gyro_time(gyro_msgs.front().header.stamp);
  const rclcpp::Time wheel_time(wheel_meas.front().first);
  double curr_time = std::min({query_time.seconds(), gyro_time.seconds(), wheel_time.seconds()});

  int ptr_ang = 0;
  int ptr_pulse = 0;
  Eigen::Vector2d delta_pos;

  EdgeTransform T_est; // estimated pose at stamp time
  EdgeTransform T_begin; // first estimated pose
  EdgeTransform T_end; // last estimated pose
  Time first_time;
  Time last_time;

  // save unique meaurement times
  std::vector<int64_t> timestamps;
  for (const auto &gyro_msg : gyro_msgs) {
    timestamps.push_back(static_cast<int64_t>(rclcpp::Time(gyro_msg.header.stamp).nanoseconds()));
  }
  for (const auto &wheel_set : wheel_meas) {
    timestamps.push_back(static_cast<int64_t>(wheel_set.first.nanoseconds()));
  }
  timestamps.push_back(static_cast<int64_t>(query_time.nanoseconds()));
  std::sort(timestamps.begin(), timestamps.end());
  timestamps.erase(std::unique(timestamps.begin(), timestamps.end()), timestamps.end());

  // estimation loop
  for (int i = 0; i < timestamps.size()-1; ++i) {
    const rclcpp::Time curr_gyro_time(gyro_msgs[ptr_ang].header.stamp);
    const rclcpp::Time curr_wheel_time(wheel_meas[ptr_pulse].first);
    const rclcpp::Time next_gyro_time(gyro_msgs[ptr_ang+1].header.stamp);
    const rclcpp::Time next_wheel_time(wheel_meas[ptr_pulse+1].first);

    const auto next_time = rclcpp::Time(timestamps[i+1]).seconds();

    const auto diff_pulse_count = wheel_meas[ptr_pulse+1].second - wheel_meas[ptr_pulse].second;
    double t0_ang = curr_time - curr_gyro_time.seconds();
    double t1_ang = next_time - curr_gyro_time.seconds();
    double diff_ang_vel = gyro_msgs[ptr_ang + 1].angular_velocity.z - gyro_msgs[ptr_ang].angular_velocity.z;
    double diff_ang_vel_time = next_gyro_time.seconds() - curr_gyro_time.seconds();
    double t_mid = (t0_ang + t1_ang) / 2;
    
    double mean_ang_vel = 0.0;
    if (diff_ang_vel_time == 0) {
      mean_ang_vel = (gyro_msgs[ptr_ang + 1].angular_velocity.z + gyro_msgs[ptr_ang].angular_velocity.z) / 2;
    } else {
      mean_ang_vel = gyro_msgs[ptr_ang].angular_velocity.z + diff_ang_vel * t_mid / diff_ang_vel_time;
    }

    if (bias_init) mean_ang_vel -= gyr_bias;

    auto ang = -mean_ang_vel * (next_time - curr_time);

    if (diff_pulse_count != 0) {
      auto diff_pulse_time = next_wheel_time.seconds() - curr_wheel_time.seconds();
      auto t0_pulse = curr_time - curr_wheel_time.seconds();
      auto t1_pulse = next_time - curr_wheel_time.seconds();

      auto dist = (t1_pulse - t0_pulse) * diff_pulse_count / diff_pulse_time;
      dist = dist * config_->wheel_parameter;
      total_dist += dist;

      long double delta_x, delta_y;
      if (ang != 0) {
        auto radius_arc = dist / ang;
        delta_y = radius_arc * std::sin(ang);
        delta_x = -radius_arc * (1 - std::cos(ang));
      } else {
        delta_y = dist;
        delta_x = 0;
      }

      Eigen::Matrix2d R;
      R << std::cos(current_theta), -std::sin(current_theta),
           std::sin(current_theta),  std::cos(current_theta);

      delta_pos << delta_x, delta_y;
      delta_pos = R * delta_pos;
    } else {
      delta_pos << 0, 0;

      if (!config_->potentially_slipping) ang = 0;

      if (config_->estimate_bias) {
        if (bias_init) {
          auto gyro_data = (gyro_msgs[ptr_ang].angular_velocity.z + gyro_msgs[ptr_ang + 1].angular_velocity.z) / 2;
          if (std::abs(gyro_data - gyr_bias) < 2 * std::abs(gyr_bias)) {
            gyr_bias = gyr_bias * (1 - bias_alpha) + gyro_data * bias_alpha;
            max_bias = std::max(max_bias, std::abs(gyr_bias));
          }
        } else {
          gyr_bias += gyro_msgs[ptr_ang].angular_velocity.z;
          bias_counter++;
          if (bias_counter >= min_bias_init_count) {
            bias_init = true;
            gyr_bias /= bias_counter;
          }
        }
      }
    }

    current_p += delta_pos;
    current_theta += ang;

    // save first estimated pose
    if (i == 0) {
      first_time = Time(static_cast<int64_t>(timestamps[i+1]));
      Eigen::Matrix4d first_trans = Eigen::Matrix4d::Identity();
      first_trans.block<2, 2>(0, 0) << std::cos(current_theta), -std::sin(current_theta),
                                       std::sin(current_theta),  std::cos(current_theta);
      first_trans.block<2, 1>(0, 3) << current_p.x(), current_p.y(); // T_m_wheel
      T_begin = EdgeTransform(first_trans, Eigen::Matrix<double, 6, 6>::Identity());
    }

    // save last estimated pose
    if (i == timestamps.size() - 2) {
      last_time = Time(static_cast<int64_t>(timestamps[i + 1]));
      Eigen::Matrix4d last_trans = Eigen::Matrix4d::Identity();
      last_trans.block<2, 2>(0, 0) << std::cos(current_theta), -std::sin(current_theta),
                                      std::sin(current_theta),  std::cos(current_theta);
      last_trans.block<2, 1>(0, 3) << current_p.x(), current_p.y(); // T_m_wheel
      T_end = EdgeTransform(last_trans, Eigen::Matrix<double, 6, 6>::Identity());
    }

    // save estimated pose at scan time
    if (next_time == query_time.seconds()) {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
      trans.block<2, 2>(0, 0) << std::cos(current_theta), -std::sin(current_theta),
                                 std::sin(current_theta),  std::cos(current_theta);
      trans.block<2, 1>(0, 3) << current_p.x(), current_p.y(); // T_m_wheel
      T_est = EdgeTransform(trans, Eigen::Matrix<double, 6, 6>::Identity());
      CLOG(DEBUG, "radar.odometry_wheel") << "Scan: " << frame_count + 1 << " / "
                                          << " Dist: " << std::round(total_dist)
                                          << " Bias: " << std::scientific << gyr_bias
                                          << " Max bias (abs): " << std::scientific << max_bias;
    } 

    curr_time = next_time;
    if (curr_time >= next_wheel_time.seconds()) ptr_pulse++;
    if (curr_time >= next_gyro_time.seconds()) ptr_ang++;
  } // end estimation loop

  // save the last gyro message and wheel measurement
  last_gyro_msg = gyro_msgs[ptr_ang];
  last_wheel_meas = wheel_meas[ptr_pulse];

  // transform estimated pose to map frame
  // at scan time
  const auto T_m_s_wheel = SE3StateVar::MakeShared(T_est); // T_map_wheel
  auto T_r_m_eval = inverse(compose(T_m_s_wheel, T_s_r_wh_var)); // T_map_wheel * T_wheel_robot
  // at first time
  const auto T_m_s_wheel_first = SE3StateVar::MakeShared(T_begin); // T_map_wheel
  auto T_r_m_eval_first = inverse(compose(T_m_s_wheel_first, T_s_r_wh_var)); // T_map_wheel * T_wheel_robot
  // at last time
  const auto T_m_s_wheel_end = SE3StateVar::MakeShared(T_end); // T_map_wheel
  auto T_r_m_eval_end = inverse(compose(T_m_s_wheel_end, T_s_r_wh_var)); // T_map_wheel * T_wheel_robot
  
  *qdata.T_r_m_odo = T_r_m_eval->value();
  *qdata.T_r_m_odo_radar = T_r_m_eval->value();
  *qdata.timestamp_odo = query_stamp;

  if (frame_count > 0) {
    // finite diff approximation for velocity
    auto vel = (T_r_m_eval_end->value() * T_r_m_eval_first->value().inverse()).vec() / (last_time - first_time).seconds();
    *qdata.w_m_r_in_r_odo = vel;

    auto &sliding_map_odo = *qdata.sliding_map_odo;
    EdgeTransform T_r_m(T_r_m_eval->value(), config_->wheel_cov.asDiagonal() * Eigen::Matrix<double, 6, 6>::Identity());
    *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
    *qdata.T_r_v_odo = T_r_m * sliding_map_odo.T_vertex_this().inverse();
    CLOG(DEBUG, "radar.odometry_wheel") << "T_r_v_odo: " << std::endl << *qdata.T_r_v_odo;

    // compound transform for alignment (sensor to point map transform)
    const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval)); // radar

    // outputs - create shallow copy
    pcl::PointCloud<PointWithInfo> aligned_points(query_points);
    const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());  
    auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

    Time knot_time(static_cast<int64_t>(query_stamp));
    const auto T_r_m_var = SE3StateVar::MakeShared(*qdata.T_r_m_odo);
    const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(*qdata.w_m_r_in_r_odo);
    trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);

#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < query_points.size(); ++i) {
      aligned_mat.block<4, 1>(0, i) = query_mat.block<4, 1>(0, i);
    }

    if (beta != 0) {
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

#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < query_points.size(); i++) {
      const auto &qry_time = query_points[i].timestamp;
      const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
      const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
      const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
      aligned_mat.block<4, 1>(0, i) = T_m_s * aligned_mat.block<4, 1>(0, i);
      aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
    }

    // undistort the preprocessed pointcloud
    const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse().cast<float>();
    aligned_mat = T_s_m * aligned_mat;
    aligned_norms_mat = T_s_m * aligned_norms_mat;
    
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
    cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
    qdata.undistorted_point_cloud = undistorted_point_cloud;

  }
  
  *qdata.odo_success = true;
  frame_count++;
  // clang-format on
}

}  // namespace radar
}  // namespace vtr