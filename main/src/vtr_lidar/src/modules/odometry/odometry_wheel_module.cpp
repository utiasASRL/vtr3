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
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // Check if the required data is initialized
  if (!qdata.gyro_msgs || !qdata.wheel_meas) {
    CLOG_IF(!qdata.gyro_msgs, WARNING, "lidar.odometry_wheel") << "No gyro messages found, cannot run odometry.";
    CLOG_IF(!qdata.wheel_meas, WARNING, "lidar.odometry_wheel") << "No wheel messages found, cannot run odometry.";
    
    if (qdata.preprocessed_point_cloud) {
      const auto &points = *qdata.preprocessed_point_cloud;
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(points);
      cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    }
    
    *qdata.odo_success = false;
    return;
  }

  if (!qdata.sliding_map_odo) {
    CLOG(INFO, "lidar.odometry_wheel") << "First frame.";
    // clang-format off
    // undistorted preprocessed point cloud
    auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(*qdata.raw_point_cloud);
    cart2pol(*undistorted_point_cloud);
    qdata.undistorted_point_cloud = undistorted_point_cloud;
    //    
    qdata.timestamp_odo.emplace(*qdata.stamp);
    qdata.T_r_m_odo.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    //
    qdata.T_r_m_odo_prior.emplace(EdgeTransform(true));
    qdata.w_m_r_in_r_odo_prior.emplace(Eigen::Matrix<double, 6, 1>::Zero());
    // Initialize timestamp equal to the end of the first frame
    const auto &query_points = *qdata.raw_point_cloud;
    const auto compare_time = [](const auto &a, const auto &b) { return a.timestamp < b.timestamp; };
    qdata.timestamp_prior.emplace(std::max_element(query_points.begin(), query_points.end(), compare_time)->timestamp);
    // clang-format on
  }

  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r; // T_lidar_robot
  const auto &T_s_r_gyro = *qdata.T_s_r_gyro;
  const auto &T_s_r_wheel = *qdata.T_s_r_wheel; // T_wheel_robot
  const auto &timestamp_odo = *qdata.timestamp_odo;

  // Load in prior parameters
  const auto &T_r_m_odo_prior = *qdata.T_r_m_odo_prior;
  const auto &w_m_r_in_r_odo_prior = *qdata.w_m_r_in_r_odo_prior;
  const auto &timestamp_prior = *qdata.timestamp_prior;

  // Load in measurements
  auto &gyro_msgs = *qdata.gyro_msgs;
  auto &wheel_meas = *qdata.wheel_meas;

  // Load in localization flag
  bool loc_flag = *qdata.loc_flag;

  double bias_alpha;
  double min_bias_init_count;
  if (config_->estimate_bias) {
    bias_alpha = config_->bias_alpha;
    min_bias_init_count = config_->min_time_bias_count;
  }

  // clang-format off
  // Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;
  const auto T_s_r_wh_var = SE3StateVar::MakeShared(T_s_r_wheel);
  T_s_r_wh_var->locked() = true;
  const auto T_s_r_gyro_var = SE3StateVar::MakeShared(T_s_r_gyro);
  T_s_r_gyro_var->locked() = true;

  const auto T_wheel_imu = T_s_r_wheel * T_s_r_gyro.inverse(); // T_wheel_imu = T_wheel_robot * T_robot_gyro

  int ptr_ang = 0;
  int ptr_pulse = 0;
  if (qdata.sliding_map_odo) {
    // add last gyro and wheel measurement to the beginning of the current measurement list
    gyro_msgs.insert(gyro_msgs.begin(), last_gyro_msg);
    wheel_meas.insert(wheel_meas.begin(), last_wheel_meas);

    CLOG(DEBUG, "lidar.odometry_wheel") << "Added last gyro and wheel measurement to the beginning of the current measurement list.";

    // Advance ptr_ang until gyro_msgs[ptr_ang].header.stamp matches last_gyro_stamp
    while (ptr_ang + 1 < gyro_msgs.size() &&
      rclcpp::Time(gyro_msgs[ptr_ang].header.stamp).nanoseconds() < rclcpp::Time(last_gyro_stamp).nanoseconds()) {
      ptr_ang++;
      CLOG(DEBUG, "lidar.odometry_wheel") << "Advancing gyro pointer to index " << ptr_ang;
    }
    // Advance ptr_pulse until wheel_meas[ptr_pulse].first matches last_wheel_stamp
    while (ptr_pulse + 1 < wheel_meas.size() &&
      wheel_meas[ptr_pulse].first.nanoseconds() < rclcpp::Time(last_wheel_stamp).nanoseconds()) {
      ptr_pulse++;
      CLOG(DEBUG, "lidar.odometry_wheel") << "Advancing wheel pointer to index " << ptr_pulse;
    }
  }

  // Initialize trajectory
  const_vel::Interface::Ptr trajectory = nullptr;
  Eigen::Matrix<double, 6, 1> traj_qc_diag = Eigen::Matrix<double, 6, 1>::Ones();
  traj_qc_diag << 1.0, 0.001, 0.001, 0.001, 0.001, 1.0; // to do: move to config
  trajectory = const_vel::Interface::MakeShared(traj_qc_diag);

  if (qdata.sliding_map_odo) {
    // add previous state to trajectory
    auto prev_T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo_prior);
    prev_T_r_m_var->locked() = true;
    auto prev_w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo_prior);
    trajectory->add(Time(timestamp_prior), prev_T_r_m_var, prev_w_m_r_in_r_var); 
  }

  // Initialize variables for odometry estimation
  double dist = 0.0;
  Eigen::Vector3d delta_pos;

  // Initialize poses we add to the trajectory
  EdgeTransform T_est; // estimated pose at stamp time
  EdgeTransform T_first; // first estimated pose
  EdgeTransform T_last; // last estimated pose

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

  // Initialize current and next timestamps
  rclcpp::Time curr_time(timestamps[0]);
  rclcpp::Time next_time(timestamps[1]);
  //
  rclcpp::Time curr_gyro_time(gyro_msgs[ptr_ang].header.stamp);
  rclcpp::Time curr_wheel_time(wheel_meas[ptr_pulse].first);
  rclcpp::Time next_gyro_time(gyro_msgs[ptr_ang + 1].header.stamp);
  rclcpp::Time next_wheel_time(wheel_meas[ptr_pulse + 1].first);

  // Find the largest timestamp less than last_pt_time
  int64_t last_timestamp = timestamps[-1];
  auto query_time = static_cast<int64_t>(query_stamp);

  // Initialize variables for velocity estimation
  double delta_time = 0.0;
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  //
  Eigen::Matrix<double, 6, 1> w_first = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> w_query = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> w_last = Eigen::Matrix<double, 6, 1>::Zero();

  // estimation loop
  Eigen::Matrix<double, 6, 1> w_robot = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < timestamps.size()-1; ++i) {
    // get the current and next timestamps
    curr_time = rclcpp::Time(timestamps[i]);
    next_time = rclcpp::Time(timestamps[i+1]);
    //
    curr_gyro_time = rclcpp::Time(gyro_msgs[ptr_ang].header.stamp);
    curr_wheel_time = wheel_meas[ptr_pulse].first;
    next_gyro_time = rclcpp::Time(gyro_msgs[ptr_ang + 1].header.stamp);
    next_wheel_time = wheel_meas[ptr_pulse + 1].first;

    if (timestamps[i] < next_est_stamp){
      if (next_time.seconds() >= next_gyro_time.seconds() && ptr_ang + 2 < (int)gyro_msgs.size()) ptr_ang++;
      if (next_time.seconds() >= next_wheel_time.seconds() && ptr_pulse + 2 < (int)wheel_meas.size()) ptr_pulse++;
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
    double diff_pulse_time = next_wheel_time.seconds() - curr_wheel_time.seconds();

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

      double t0_pulse = curr_time.seconds() - curr_wheel_time.seconds();
      double t1_pulse = next_time.seconds() - curr_wheel_time.seconds();
      dist = (t1_pulse - t0_pulse) * diff_pulse_count / diff_pulse_time;
      dist = dist * config_->wheel_parameter;
      
      total_dist += dist;

      delta_pos = current_theta * Eigen::Vector3d(0, dist, 0); // check this
      
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

    // linear interpolation at current time
    double t_interp = curr_time.seconds() - curr_gyro_time.seconds();
    double ratio = std::clamp(t_interp / diff_ang_vel_time, 0.0, 1.0);
    Eigen::Vector3d instant_gyro_wheel = curr_gyro_wheel + diff_ang_vel * ratio;

    const double alpha = 0.2;
    double raw_pulse_rate = (diff_pulse_time > 1e-6) ? (diff_pulse_count / diff_pulse_time) : 0.0;

    // only filter if we have a valid time step
    if (diff_pulse_time > 1e-6) {
        filtered_pulse_rate_ = (alpha * raw_pulse_rate) + ((1.0 - alpha) * filtered_pulse_rate_);
    }

    double v_fwd = filtered_pulse_rate_ * config_->wheel_parameter;
    Eigen::Vector3d v_wheel(0.0, v_fwd, 0.0);
    Eigen::Vector3d omega_wheel = instant_gyro_wheel;

    Eigen::Matrix3d R_r_wheel = T_s_r_wheel.matrix().inverse().block<3, 3>(0, 0).cast<double>();
    w_robot.block<3, 1>(0, 0) = R_r_wheel * v_wheel;
    w_robot.block<3, 1>(3, 0) = R_r_wheel * omega_wheel;
    
    if (next_time.seconds() >= next_wheel_time.seconds() && ptr_pulse + 2 < (int)wheel_meas.size()) ptr_pulse++;
    if (next_time.seconds() >= next_gyro_time.seconds() && ptr_ang + 2 < (int)gyro_msgs.size()) ptr_ang++;

  } // end estimation loop 

  // save last estimated pose timestamp and measurement for next iteration
  int64_t last_est_time = curr_time.nanoseconds();
  Eigen::Matrix4d last_trans = Eigen::Matrix4d::Identity();
  last_trans.block<3, 3>(0, 0) = current_theta;
  last_trans.block<3, 1>(0, 3) = current_p; // T_m_wheel
  T_last = EdgeTransform(last_trans, Eigen::Matrix<double, 6, 6>::Identity() * 1e-3);
  w_last = w_robot;

  next_est_stamp = next_time.nanoseconds();
  last_gyro_stamp = curr_gyro_time.nanoseconds();
  last_wheel_stamp = curr_wheel_time.nanoseconds();

  last_gyro_msg = gyro_msgs.back();
  last_wheel_meas = wheel_meas.back();

  // transform estimated pose to map frame at last time
  const auto T_m_s_wheel_last = SE3StateVar::MakeShared(T_last); // T_map_wheel
  auto T_r_m_eval_last = inverse(compose(T_m_s_wheel_last, T_s_r_wh_var)); // T_map_wheel * T_wheel_robot
  auto vel_last = VSpaceStateVar<6>::MakeShared(-w_last);

  bool estimate_reasonable = true;
  // Check if change between initial and final velocity is reasonable
  const auto &w_m_r_in_r_prev = *qdata.w_m_r_in_r_odo_prior;
  const auto &w_m_r_in_r_new = vel_last->value();
  const auto vel_diff = w_m_r_in_r_new - w_m_r_in_r_prev;
  const auto vel_diff_norm = vel_diff.norm();
  const auto trans_vel_diff_norm = vel_diff.head<3>().norm();
  const auto rot_vel_diff_norm = vel_diff.tail<3>().norm();

  const auto T_r_m_new = T_r_m_eval_last->value();
  const auto diff_T = (T_r_m_odo_prior * T_r_m_new.inverse()).vec();
  const auto diff_T_trans = diff_T.head<3>().norm();
  const auto diff_T_rot = diff_T.tail<3>().norm();
  
  CLOG(DEBUG, "lidar.odometry_wheel") << "Current transformation difference: " << diff_T.transpose();
  CLOG(DEBUG, "lidar.odometry_wheel") << "Diff_T_trans: " << diff_T_trans << " , Diff_T_rot: " << diff_T_rot;
  CLOG(DEBUG, "lidar.odometry_wheel") << "Current velocity difference: " << vel_diff.transpose();
  CLOG(DEBUG, "lidar.odometry_wheel") << "Translational velocity diff: " << trans_vel_diff_norm << " , Rotational velocity diff: " << rot_vel_diff_norm;

  if (trans_vel_diff_norm > config_->max_trans_vel_diff || rot_vel_diff_norm > config_->max_rot_vel_diff) {
    CLOG(WARNING, "lidar.odometry_wheel") << "Velocity difference between initial and final is too large: " << vel_diff_norm << ". Translational velocity difference: " << trans_vel_diff_norm << ". Rotational velocity difference: " << rot_vel_diff_norm;
    estimate_reasonable = false;
  }

  if (diff_T_trans > config_->max_trans_diff) {
    CLOG(WARNING, "lidar.odometry_wheel") << "Transformation difference between initial and final translation is too large. Transform difference vector: " << diff_T.transpose();
    estimate_reasonable = false;
  }
  if (diff_T_rot > config_->max_rot_diff) {
    CLOG(WARNING, "lidar.odometry_wheel") << "Transformation difference between initial and final rotation is too large. Transform difference vector: " << diff_T.transpose();
    estimate_reasonable = false;
  }


  if (estimate_reasonable) {

    // add estimated state at the end of the frame as a knot to the trajectory
    trajectory->add(Time(last_est_time), T_r_m_eval_last, VSpaceStateVar<6>::MakeShared(vel_last->value()));

    // interpolate state at query time
    Evaluable<lgmath::se3::Transformation>::ConstPtr T_r_m_query = nullptr;
    Evaluable<Eigen::Matrix<double, 6, 1>>::ConstPtr w_m_r_in_r_query = nullptr;
    T_r_m_query = trajectory->getPoseInterpolator(Time(query_time));
    w_m_r_in_r_query = trajectory->getVelocityInterpolator(Time(query_time));

    // compound transform for alignment (sensor to point map transform)
    const auto T_s_m_query = compose(T_s_r_var, T_r_m_query);

    *qdata.T_r_m_odo = EdgeTransform(T_r_m_query->value(), Eigen::Matrix<double, 6, 6>::Identity() * 1e-3);
    *qdata.w_m_r_in_r_odo = w_m_r_in_r_query->value();
    *qdata.timestamp_odo = query_stamp;


    if (loc_flag && qdata.preprocessed_point_cloud) {
      CLOG(DEBUG, "lidar.odometry_wheel") << "localizing: processing point cloud with estimated pose at query time";
      // only if we're localizing this frame
      const auto &query_points = *qdata.preprocessed_point_cloud;

      // outputs - create shallow copy
      pcl::PointCloud<PointWithInfo> aligned_points(query_points);
      const auto query_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      const auto query_norms_mat = query_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());  
      auto aligned_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
      auto aligned_norms_mat = aligned_points.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::normal_offset());

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
      const auto T_s_m = T_s_m_query->evaluate().matrix().cast<float>();
      aligned_mat = T_s_m * aligned_mat;
      aligned_norms_mat = T_s_m * aligned_norms_mat;
      
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(aligned_points);
      cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    } else {
      // if not localizing, just save the estimated pose and velocity at query time without transforming the point cloud
      CLOG(DEBUG, "lidar.odometry_wheel") << "not localizing: skip point cloud processing, saving estimated pose and velocity";
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>();
      undistorted_point_cloud->resize(1); // create a dummy point cloud with one point since we won't be using it for localization
      cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
      qdata.undistorted_point_cloud = undistorted_point_cloud;
    }

    if (!first_frame_) {
      EdgeTransform T_r_m(*qdata.T_r_m_odo, Eigen::Matrix<double, 6, 6>::Identity() * 1e-3);
      auto &sliding_map_odo = *qdata.sliding_map_odo;
      *qdata.T_r_v_odo = T_r_m * sliding_map_odo.T_vertex_this().inverse(); // T_r_m * T_m_v
      *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;
    }

    *qdata.T_r_m_odo_prior = T_r_m_eval_last->value();
    *qdata.w_m_r_in_r_odo_prior = vel_last->value();
    *qdata.timestamp_prior = last_est_time;

    *qdata.odo_success = true;
  } else {
    CLOG(WARNING, "lidar.odometry_wheel") << "Wheel-Odometer Odometry failed";
    if (loc_flag && qdata.preprocessed_point_cloud) {
      CLOG(DEBUG, "lidar.odometry_wheel") << "using VTR preprocessed point cloud";
      const auto &query_points = *qdata.preprocessed_point_cloud;
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>(query_points);
      cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
      qdata.undistorted_point_cloud = undistorted_point_cloud;
      CLOG(DEBUG, "lidar.odometry_wheel") << "undistorted_point_cloud size: " << undistorted_point_cloud->size();
    } else {
      CLOG(DEBUG, "lidar.odometry_wheel") << "no point cloud saved";
      auto undistorted_point_cloud = std::make_shared<pcl::PointCloud<PointWithInfo>>();
      undistorted_point_cloud->resize(1); // create a dummy point cloud with one point since we won't be using it for localization
      cart2pol(*undistorted_point_cloud);  // correct polar coordinates.
      qdata.undistorted_point_cloud = undistorted_point_cloud;
      CLOG(DEBUG, "lidar.odometry_wheel") << "undistorted_point_cloud size: " << undistorted_point_cloud->size();
    }
    *qdata.odo_success = false;  
  }

  first_frame_ = false;

  // clang-format on
}

}  // namespace lidar
}  // namespace vtr