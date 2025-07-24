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
#include "vtr_lidar/modules/odometry/odometry_wheel_module.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

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
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

void OdometryWheelModule::run_(QueryCache &qdata0, OutputCache &,
                               const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "lidar.odometry_wheel") << "First frame.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.preprocessed_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //    
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    // clang-format on
    frame_count = 0;
  }

  if (!qdata.gyro_msgs) {
    CLOG(WARNING, "lidar.odometry_wheel") << "No gyro messages found, cannot run odometry.";
    frame_count++;
    return;
  }

  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &query_points = *qdata.preprocessed_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_s_r_gyro = *qdata.T_s_r_gyro;
  const auto &T_s_r_wheel = *qdata.T_s_r_wheel;
  const auto &timestamp_odo = *qdata.timestamp_odo;
  const auto &T_r_m_odo = *qdata.T_r_m_odo;
  const auto &w_m_r_in_r_odo = *qdata.w_m_r_in_r_odo;

  double bias_alpha;
  double min_bias_init_count;
  if (config_->estimate_bias) {
    bias_alpha = config_->bias_alpha;
    min_bias_init_count = config_->min_time_bias_count;
  }

  const auto T_wheel_imu = T_s_r_wheel * T_s_r_gyro.inverse();

  // clang-format off
  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;
  const auto T_s_r_wh_var = SE3StateVar::MakeShared(T_s_r_wheel);
  T_s_r_wh_var->locked() = true;
  const auto T_s_r_gyro_var = SE3StateVar::MakeShared(T_s_r_gyro);
  T_s_r_gyro_var->locked() = true;

  auto &gyro_msgs = *qdata.gyro_msgs;
  auto &wheel_meas = *qdata.wheel_meas;

  int ptr_ang = 0;
  int ptr_pulse = 0;
  if (frame_count > 0) {
    // Advance ptr_ang until gyro_msgs[ptr_ang].header.stamp matches last_gyro_stamp
    while (ptr_ang + 1 < gyro_msgs.size() &&
      rclcpp::Time(gyro_msgs[ptr_ang].header.stamp).nanoseconds() < rclcpp::Time(last_gyro_stamp).nanoseconds()) {
      ptr_ang++;
    }
    // Advance ptr_pulse until wheel_meas[ptr_pulse].first matches last_wheel_stamp
    while (ptr_pulse + 1 < wheel_meas.size() &&
      wheel_meas[ptr_pulse].first.nanoseconds() < rclcpp::Time(last_wheel_stamp).nanoseconds()) {
      ptr_pulse++;
    }
  }

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

  Eigen::Vector3d delta_pos;

  EdgeTransform T_est; // estimated pose at stamp time
  EdgeTransform T_begin; // first estimated pose
  EdgeTransform T_end; // last estimated pose

  // save unique measurement times
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

  rclcpp::Time curr_time(timestamps[0]);
  rclcpp::Time next_time(timestamps[1]);

  rclcpp::Time curr_gyro_time(gyro_msgs[ptr_ang].header.stamp);
  rclcpp::Time curr_wheel_time(wheel_meas[ptr_pulse].first);
  rclcpp::Time next_gyro_time(gyro_msgs[ptr_ang + 1].header.stamp);
  rclcpp::Time next_wheel_time(wheel_meas[ptr_pulse + 1].first);

  const auto compare_time = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
  int64_t first_pt_time = std::min_element(query_points.begin(), query_points.end(), compare_time)->timestamp;
  int64_t last_pt_time = std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp;

  // Find the largest timestamp less than last_pt_time
  auto it = std::lower_bound(timestamps.begin(), timestamps.end(), last_pt_time);
  int64_t last_timestamp = (it == timestamps.begin()) ? *it : *(it - 1);

  Time first_time(static_cast<int64_t>(next_est_stamp));
  Time last_time(static_cast<int64_t>(last_timestamp));


  CLOG(DEBUG, "lidar.odometry_wheel") << std::endl
      << "first_pt_time : " << first_pt_time << std::endl
      << "next_est_stamp: " << next_est_stamp << std::endl
      << "last_pt_time  : " << last_pt_time << std::endl
      << "last_timestamp: " << last_timestamp << std::endl;

  // estimation loop
  for (int i = 0; i < timestamps.size()-1; ++i) {
    if (timestamps[i] > last_pt_time) continue;

    // get the current and next timestamps
    curr_time = rclcpp::Time(timestamps[i]);
    next_time = rclcpp::Time(timestamps[i+1]);
    //
    curr_gyro_time = rclcpp::Time(gyro_msgs[ptr_ang].header.stamp);
    curr_wheel_time = wheel_meas[ptr_pulse].first;
    next_gyro_time = rclcpp::Time(gyro_msgs[ptr_ang + 1].header.stamp);
    next_wheel_time = wheel_meas[ptr_pulse + 1].first;

    if (timestamps[i] < next_est_stamp){
      if (next_time.seconds() >= next_gyro_time.seconds()) ptr_ang++;
      if (next_time.seconds() >= next_wheel_time.seconds()) ptr_pulse++;
      continue;
    }

    Eigen::Vector3d curr_gyro_meas = Eigen::Vector3d(
        gyro_msgs[ptr_ang].angular_velocity.x,
        gyro_msgs[ptr_ang].angular_velocity.y,
        gyro_msgs[ptr_ang].angular_velocity.z);

    Eigen::Vector3d next_gyro_meas = Eigen::Vector3d(
        gyro_msgs[ptr_ang + 1].angular_velocity.x,
        gyro_msgs[ptr_ang + 1].angular_velocity.y,
        gyro_msgs[ptr_ang + 1].angular_velocity.z);

    Eigen::Vector3d curr_gyro_wheel = T_wheel_imu.matrix().block<3, 3>(0, 0).cast<double>() * curr_gyro_meas;
    Eigen::Vector3d next_gyro_wheel = T_wheel_imu.matrix().block<3, 3>(0, 0).cast<double>() * next_gyro_meas;

    const int diff_pulse_count = wheel_meas[ptr_pulse+1].second - wheel_meas[ptr_pulse].second;

    double t0_ang = curr_time.seconds() - curr_gyro_time.seconds();
    double t1_ang = next_time.seconds() - curr_gyro_time.seconds();
    auto diff_ang_vel = next_gyro_wheel - curr_gyro_wheel;
    double diff_ang_vel_time = next_gyro_time.seconds() - curr_gyro_time.seconds();
    double t_mid = (t0_ang + t1_ang) / 2;    
    
    Eigen::Vector3d mean_ang_vel = Eigen::Vector3d::Zero();
    if (diff_ang_vel_time == 0) {
      mean_ang_vel = (next_gyro_wheel + curr_gyro_wheel) / 2;
    } else {
      mean_ang_vel = curr_gyro_wheel + diff_ang_vel * t_mid / diff_ang_vel_time;
    }

    if (bias_init) mean_ang_vel -= gyro_bias;

    Eigen::Vector3d ang = mean_ang_vel * (next_time.seconds() - curr_time.seconds());

    if (diff_pulse_count != 0) {
      double diff_pulse_time = next_wheel_time.seconds() - curr_wheel_time.seconds();

      double t0_pulse = curr_time.seconds() - curr_wheel_time.seconds();
      double t1_pulse = next_time.seconds() - curr_wheel_time.seconds();
      auto dist = (t1_pulse - t0_pulse) * diff_pulse_count / diff_pulse_time;
      dist = dist * config_->wheel_parameter;
      
      total_dist += dist;

      delta_pos = current_theta * Eigen::Vector3d(dist, 0, 0); // check this
      
    } else {
      delta_pos = Eigen::Vector3d::Zero();

      if (!config_->potentially_slipping) ang = Eigen::Vector3d::Zero();
      
      int64_t gyro_time_ns = static_cast<int64_t>(curr_gyro_time.nanoseconds());
      if (config_->estimate_bias && last_bias_time < gyro_time_ns) {
        if (bias_init) {
          auto gyro_data = (curr_gyro_wheel + next_gyro_wheel) / 2;
          if (gyro_data.norm() < 2 * gyro_bias.norm()) {
            gyro_bias = gyro_bias * (1 - bias_alpha) + gyro_data * bias_alpha;
            max_bias = std::max(max_bias, gyro_bias.norm());
          }
        } else {
          gyro_bias += curr_gyro_wheel;
          bias_counter++;
          if (bias_counter >= min_bias_init_count) {
            bias_init = true;
            gyro_bias /= bias_counter;
          }
        }
        last_bias_time = gyro_time_ns;
      }
    }

    current_p += delta_pos;
    current_theta = current_theta * lgmath::so3::vec2rot(ang); // update orientation

    // save first estimated pose
    if (i == next_est_stamp) {
      Eigen::Matrix4d first_trans = Eigen::Matrix4d::Identity();
      first_trans.block<3, 3>(0, 0) = current_theta;
      first_trans.block<3, 1>(0, 3) << current_p.x(), current_p.y(), current_p.z(); // T_m_wheel
      T_begin = EdgeTransform(first_trans, Eigen::Matrix<double, 6, 6>::Identity());
    }

    // save last estimated pose
    if (i == last_timestamp) {
      Eigen::Matrix4d last_trans = Eigen::Matrix4d::Identity();
      last_trans.block<3, 3>(0, 0) = current_theta;
      last_trans.block<3, 1>(0, 3) << current_p.x(), current_p.y(), current_p.z(); // T_m_wheel
      T_end = EdgeTransform(last_trans, Eigen::Matrix<double, 6, 6>::Identity());
    }

    // save estimated pose at scan time
    if (next_time.seconds() == query_time.seconds()) {
      Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
      trans.block<3, 3>(0, 0) = current_theta;
      trans.block<3, 1>(0, 3) << current_p.x(), current_p.y(), current_p.z(); // T_m_wheel
      T_est = EdgeTransform(trans, Eigen::Matrix<double, 6, 6>::Identity());
    }

    if (next_time.seconds() >= next_wheel_time.seconds())
      ptr_pulse++;

    if (next_time.seconds() >= next_gyro_time.seconds())
      ptr_ang++;

  } // end estimation loop 

  next_est_stamp = next_time.nanoseconds();
  last_gyro_stamp = curr_gyro_time.nanoseconds();
  last_wheel_stamp = curr_wheel_time.nanoseconds();

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
  *qdata.timestamp_odo = query_stamp;

  if (frame_count > 0) {
    // finite diff approximation for velocity
    *qdata.w_m_r_in_r_odo = (T_r_m_eval_end->value() * T_r_m_eval_first->value().inverse()).vec() / ((last_pt_time - first_pt_time) / 1e9);

    CLOG(DEBUG, "lidar.odometry_wheel")
        << "last_timestamp: " << last_pt_time
        << " next_est_stamp: " << first_pt_time
        << " last-first time: " << (last_pt_time - first_pt_time) / 1e9;

    auto &sliding_map_odo = *qdata.sliding_map_odo;
    EdgeTransform T_r_m(T_r_m_eval->value(), Eigen::Matrix<double, 6, 6>::Identity());
    *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
    *qdata.T_r_v_odo = T_r_m * sliding_map_odo.T_vertex_this().inverse();

    // compound transform for alignment (sensor to point map transform)
    const auto T_m_s_eval = inverse(compose(T_s_r_var, T_r_m_eval)); // lidar

    // outputs - create shallow copy
    pcl::PointCloud<PointWithInfo> aligned_points(query_points);
    const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());  
    auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

    const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(*qdata.w_m_r_in_r_odo);
    trajectory->add(last_time, T_r_m_eval_end, w_m_r_in_r_var);

#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (unsigned i = 0; i < query_points.size(); i++) {
      const auto &qry_time = query_points[i].timestamp;
      const auto T_r_m_intp_eval = trajectory->getPoseInterpolator(Time(qry_time));
      const auto T_m_s_intp_eval = inverse(compose(T_s_r_var, T_r_m_intp_eval));
      const auto T_m_s = T_m_s_intp_eval->evaluate().matrix().cast<float>();
      aligned_mat.block<4, 1>(0, i) = T_m_s * query_mat.block<4, 1>(0, i);
      aligned_norms_mat.block<4, 1>(0, i) = T_m_s * query_norms_mat.block<4, 1>(0, i);
    }

    // undistort the preprocessed pointcloud
    const auto T_s_m = T_m_s_eval->evaluate().matrix().inverse().cast<float>();
    aligned_mat = T_s_m * aligned_mat;
    aligned_norms_mat = T_s_m * aligned_norms_mat;
    
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
    cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
    qdata.undistorted_point_cloud = undistorted_point_cloud;

    CLOG(DEBUG, "lidar.odometry_wheel") << "Undistorted point cloud size: " << qdata.undistorted_point_cloud->size();

  }
  
  *qdata.odo_success = true;
  frame_count++;
  // clang-format on
}

}  // namespace lidar
}  // namespace vtr