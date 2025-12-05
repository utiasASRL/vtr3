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

  // High-level parameters
  config->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config->num_threads);
  config->zero_velocity_threshold = node->declare_parameter<double>(param_prefix + ".zero_velocity_threshold", 0.1);
  config->optimize = node->declare_parameter<bool>(param_prefix + ".optimize", true);

  // CT trajectory optimization parameters
  config->max_iter = node->declare_parameter<int>(param_prefix + ".max_iter", 20);
  config->traj_num_extra_states = node->declare_parameter<int>(param_prefix + ".traj_num_extra_states", config->traj_num_extra_states);
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", false);
  config->integration_steps = node->declare_parameter<int>(param_prefix + ".integration_steps", 100);

  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 3) {
    std::string err{"Qc diagonal malformed. Must be 3 elements!"};
    CLOG(ERROR, "radar.odometry_icp") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2];

  const auto doppler_bias = node->declare_parameter<std::vector<double>>(param_prefix + ".doppler_bias", {1.0, 1.0});
  if (doppler_bias.size() != 2) {
    std::string err{"doppler_bias malformed. Must be 2 elements!"};
    CLOG(ERROR, "radar.odometry_icp") << err;
    throw std::invalid_argument{err};
  }

  // Noise and measurement model parameters
  config->doppler_bias << doppler_bias[0], doppler_bias[1];
  config->dopp_cauchy_k = node->declare_parameter<double>(param_prefix + ".dopp_cauchy_k", config->dopp_cauchy_k);
  config->dopp_meas_std = node->declare_parameter<double>(param_prefix + ".dopp_meas_std", config->dopp_meas_std);
  config->gyro_cov = node->declare_parameter<double>(param_prefix + ".gyro_cov", config->gyro_cov);

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
    qdata.cov_prior.emplace(1e-5 * Eigen::Matrix<double, 12, 12>::Identity());
    qdata.timestamp_prior.emplace(*qdata.stamp);
    scan_stamp_prev_ = *qdata.stamp;

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
  auto &w_m_r_in_r_odo_prior = *qdata.w_m_r_in_r_odo_prior;
  // This is the timestamp of the last gyro measurement from the previous frame for the discrete case
  // It is the timestamp of the last gyro or Doppler measurement from the previous frame for the optimization case
  auto &timestamp_prev = *qdata.timestamp_prior; 

  // Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE2StateVar::MakeShared(T_s_r.toSE2());
  T_s_r_var->locked() = true;

  // Extract timestamps in seconds
  long double scan_stamp_s = static_cast<long double>(scan_stamp) / 1e9;
  long double timestamp_prev_s = static_cast<long double>(timestamp_prev) / 1e9;
  long double scan_stamp_prev_s = static_cast<long double>(scan_stamp_prev_) / 1e9;

  // Create trajectory for undistortion
  const_vel_se2::Interface::Ptr udist_trajectory = const_vel_se2::Interface::MakeShared(config_->traj_qc_diag);

  // Output variables populated by either pipeline
  Eigen::Matrix4d T_r_delta = Eigen::Matrix4d::Identity();
  // 6D velocity vector in robot frame, we only estimate 3 but pipeline needs to pass around 6D vectors
  Eigen::VectorXd w_v_r_in_r = Eigen::VectorXd::Zero(6);

  if (config_->optimize) {
    CLOG(DEBUG, "radar.odometry_doppler") << "Setting up STEAM optimization problem.";

    // Load in prior covariance
    const auto &cov_prior = *qdata.cov_prior;
    Eigen::Matrix<double, 6, 6> cov_prior_2d = cov3Dto2D(cov_prior);

    // Compute end of trajectory (max timestamp of gyro or Doppler measurements)
    int64_t traj_end_time;
    const int64_t dopp_end_time = static_cast<int64_t>((*qdata.doppler_scan).back().timestamp);
    if (qdata.gyro_msgs) {
      const rclcpp::Time gyro_end_stamp((*qdata.gyro_msgs).back().header.stamp);
      const int64_t gyro_end_time = static_cast<int64_t>(gyro_end_stamp.nanoseconds());
      traj_end_time = std::max(dopp_end_time, gyro_end_time);
    } else {
      traj_end_time = dopp_end_time;
    }
    long double traj_end_time_s = static_cast<long double>(traj_end_time) / 1e9;

    // Initialize problem
    // Create secondary trajectory to actually optimize over
    // TODO: This should ideally be an SE(2) velocity-only trajectory since the pose is recovered after optimizing
    const_vel_se2::Interface::Ptr trajectory = const_vel_se2::Interface::MakeShared(config_->traj_qc_diag);
    SlidingWindowFilter problem(config_->num_threads);
    std::vector<StateVarBase::Ptr> state_vars;

    // Add variables
    // Create dummy pose variable
    const auto T_r_m_dummy_state = lgmath::se2::Transformation();
    const Eigen::Matrix<double, 3, 1> w_m_r_in_r_odo_prior_2d = vec3Dto2D(w_m_r_in_r_odo_prior);

    // Set up states between timestamp_prev and traj_end_time according to integration steps
    const int64_t num_states = config_->traj_num_extra_states + 2;
    const int64_t time_diff = (traj_end_time - timestamp_prev) / (num_states - 1);
    for (int i = 0; i < num_states; ++i) {
      // Load in explicit end_time in case there is small rounding issues
      const int64_t knot_time_stamp = (i == num_states - 1) ? traj_end_time : timestamp_prev + i * time_diff;
      Time knot_time(static_cast<int64_t>(knot_time_stamp));
      const std::string T_r_m_var_name = "T_r_m_" + std::to_string(i);
      const auto T_r_m_var = SE2StateVar::MakeShared(T_r_m_dummy_state, T_r_m_var_name);
      const std::string w_m_r_in_r_var_name = "w_m_r_in_r_" + std::to_string(i);
      const auto w_m_r_in_r_var = VSpaceStateVar<3>::MakeShared(w_m_r_in_r_odo_prior_2d, w_m_r_in_r_var_name);
      trajectory->add(knot_time, T_r_m_var, w_m_r_in_r_var);
      state_vars.emplace_back(T_r_m_var);
      state_vars.emplace_back(w_m_r_in_r_var);
      problem.addStateVariable(T_r_m_var);
      problem.addStateVariable(w_m_r_in_r_var);
    }

    // Add prior cost term
    // trajectory->addStatePrior(Time(timestamp_prev), T_r_m_dummy_state, w_m_r_in_r_odo_prior_2d, 100*cov_prior_2d);
    trajectory->addPosePrior(Time(timestamp_prev), T_r_m_dummy_state, Eigen::Matrix<double, 3, 3>::Identity() * 1e-6);
    trajectory->addVelocityPrior(Time(timestamp_prev), w_m_r_in_r_odo_prior_2d, cov_prior_2d.block<3,3>(3,3));
    trajectory->addPriorCostTerms(problem);

    // Add Doppler velocity measurements
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
    for (const auto &azimuth_vel : *qdata.doppler_scan) {
      // Load in Doppler measurement and timestamp
      const auto radial_velocity = azimuth_vel.radial_velocity;
      const auto dopp_time = azimuth_vel.timestamp;
      const auto azimuth = azimuth_vel.azimuth;
      const auto azimuth_idx = azimuth_vel.azimuth_idx;
      const Eigen::Matrix<double, 1, 1> radial_velocity_vec = radial_velocity * Eigen::Matrix<double, 1, 1>::Ones();

      // Get velocity in radar frame
      const auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(dopp_time));
      const auto w_m_s_in_s_intp_eval = compose_velocity(T_s_r_var, w_m_r_in_r_intp_eval);

      // Generate empty bias state (for now)
      Eigen::Matrix<double, 3, 1> b_zero = Eigen::Matrix<double, 3, 1>::Zero();
      b_zero.head<2>() = config_->doppler_bias;
      const auto bias = VSpaceStateVar<3>::MakeShared(b_zero);
      bias->locked() = true;

      // Form error terms
      const auto loss_func = CauchyLossFunc::MakeShared(config_->dopp_cauchy_k);
      const auto noise_model = StaticNoiseModel<1>::MakeShared(Eigen::Matrix<double, 1, 1>(config_->dopp_meas_std));
      const auto error_func = se2::RadialVelErrorEvaluator::MakeShared(w_m_s_in_s_intp_eval, bias, azimuth, radial_velocity_vec);
      const auto doppler_cost = WeightedLeastSqCostTerm<1>::MakeShared(error_func, noise_model, loss_func, "doppler_cost" + std::to_string(azimuth_idx));

#pragma omp critical(odo_dopp_add_doppler_error_cost)
{
      // Add cost term to problem
      problem.addCostTerm(doppler_cost);
}
    }

    // Add gyro measurements to problem, if they are present
    // Sometimes there may be gyro dropout for a frame, in which case we rely on
    // the prior to fix our orientation
    if (qdata.gyro_msgs) {
#pragma omp parallel for schedule(dynamic, 10) num_threads(config_->num_threads)
      for (const auto &gyro_msg : *qdata.gyro_msgs) {
        // Load in gyro measurement and timestamp
        const auto yaw_meas = gyro_msg.angular_velocity.z;
        const rclcpp::Time gyro_stamp(gyro_msg.header.stamp);
        const auto gyro_stamp_time = static_cast<int64_t>(gyro_stamp.nanoseconds());

        // Interpolate velocity measurement at gyro stamp
        auto w_m_r_in_r_intp_eval = trajectory->getVelocityInterpolator(Time(gyro_stamp_time));

        // Generate empty bias state
        Eigen::Matrix<double, 3, 1> b_zero = Eigen::Matrix<double, 3, 1>::Zero();
        const auto bias = VSpaceStateVar<3>::MakeShared(b_zero);
        bias->locked() = true;
        const auto loss_func = L2LossFunc::MakeShared();
        Eigen::Matrix<double, 1, 1> W_gyro = config_->gyro_cov * Eigen::Matrix<double, 1, 1>::Identity();
        const auto noise_model = StaticNoiseModel<1>::MakeShared(W_gyro);
        const auto error_func = imu::GyroErrorEvaluatorSE2::MakeShared(w_m_r_in_r_intp_eval, bias, yaw_meas);
        const auto gyro_cost = WeightedLeastSqCostTerm<1>::MakeShared(error_func, noise_model, loss_func, "gyro_cost" + std::to_string(gyro_stamp_time));

#pragma omp critical(odo_dopp_add_gyro_error_cost)
{
        problem.addCostTerm(gyro_cost);
}
      }
    }

    // Optimize the problem
    CLOG(DEBUG, "radar.odometry_doppler") << "Optimizing trajectory.";
    // optimize
    GaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.reuse_previous_pattern = false;
    params.max_iterations = (unsigned int)config_->max_iter;

    GaussNewtonSolver solver(problem, params);
    try {
      solver.optimize();
    } catch(const std::runtime_error& e) {
      CLOG(WARNING, "radar.odometry_icp") << "STEAM failed to solve, skipping frame. Error message: " << e.what();
      *qdata.odo_success = false;
      return;
    }

    // Integrate velocities to get pose
    const double delta_t = (traj_end_time_s - timestamp_prev_s) / config_->integration_steps;
    Eigen::Matrix4d T_r_delta_temp = T_r_delta_prev_;
    bool reached_scan_time = false;
    for (int i=0; i<config_->integration_steps; i++) {
      const double t0 = timestamp_prev_s + i * delta_t;
      const double t1 = timestamp_prev_s + (i + 1) * delta_t;

      const auto w_m_r_in_r_t0 = trajectory->getVelocityInterpolator(Time(static_cast<int64_t>(t0 * 1e9)))->value();
      const auto w_m_r_in_r_t1 = trajectory->getVelocityInterpolator(Time(static_cast<int64_t>(t1 * 1e9)))->value();
      Eigen::Vector3d w_m_r_in_r_avg = (w_m_r_in_r_t0 + w_m_r_in_r_t1) / 2.0;

      // const Eigen::Vector2d linear_vel_s = *qdata.vel_meas;
      if (-w_m_r_in_r_avg(0) < config_->zero_velocity_threshold) {
        // If the velocity is very low, we assume no movement
        w_m_r_in_r_avg = Eigen::Matrix<double, 3, 1>::Zero();
      }

      // Compute delta pose over this small interval
      const double dt = t1 - t0;
      auto T_r_dt = lgmath::se3::vec2tran(vec2Dto3D(w_m_r_in_r_avg * dt));

      if (t1 >= scan_stamp_s && !reached_scan_time) {
        // Reached end of scan time, save delta
        const double dt_temp = scan_stamp_s - t0;
        const auto T_r_dt_temp = lgmath::se3::vec2tran(vec2Dto3D(w_m_r_in_r_avg * dt_temp));
        T_r_delta_temp = T_r_dt_temp * T_r_delta_temp;
        // Save the transform at scan time
        T_r_delta = T_r_delta_temp;
        // Reset T_r_delta_temp
        T_r_delta_temp = Eigen::Matrix4d::Identity();
        T_r_dt = lgmath::se3::vec2tran(vec2Dto3D(w_m_r_in_r_avg * (t1 - scan_stamp_s)));
        reached_scan_time = true;
      }

      T_r_delta_temp = T_r_dt * T_r_delta_temp;

      // Populate udist_trajectory for undistortion
      if (i ==0) {
        // First iteration, add previous pose
        udist_trajectory->add(timestamp_prev, SE2StateVar::MakeShared(T_r_m_prev.toSE2()), VSpaceStateVar<3>::MakeShared(w_m_r_in_r_t0));
      }
      udist_trajectory->add(static_cast<int64_t>(t1 * 1e9), SE2StateVar::MakeShared((T_r_delta * T_r_m_prev).toSE2()), VSpaceStateVar<3>::MakeShared(w_m_r_in_r_t1));
    }

    // Save leftover delta
    T_r_delta_prev_ = T_r_delta_temp;
    std::cout << "T_r_delta_temp: \n" << T_r_delta_temp << std::endl;

    // Save the velocity estimate
    w_v_r_in_r = vec2Dto3D(trajectory->getVelocityInterpolator(Time(static_cast<int64_t>(scan_stamp)))->value());
    if (-w_v_r_in_r(0) < config_->zero_velocity_threshold) {
      // If the velocity is very low, we assume no movement
      w_v_r_in_r = Eigen::VectorXd::Zero(6);
    }

    // Now marginalize out all but last pose and velocity to get prior for next frame
    std::vector<StateVarBase::Ptr> state_vars_marg;
    for (int i = 0; i < num_states*2 - 2; ++i) {
      state_vars_marg.push_back(state_vars[i]);
    }
    problem.marginalizeVariable(state_vars_marg);
    params.max_iterations = 1; // Only one iteration for marginalization
    GaussNewtonSolver solver_marg(problem, params);
    solver_marg.optimize();
    Covariance covariance_marg(solver_marg);
    w_m_r_in_r_odo_prior = vec2Dto3D(trajectory->get(Time(static_cast<int64_t>(traj_end_time)))->velocity()->evaluate());
    cov_prior_2d = trajectory->getCovariance(covariance_marg, Time(static_cast<int64_t>(traj_end_time))).block<6, 6>(0, 0);
    cov_prior_2d = 0.5 * (cov_prior_2d + cov_prior_2d.transpose());

    // Save prior stuff for next frame
    *qdata.cov_prior = cov2Dto3D(cov_prior_2d);
    timestamp_prev = traj_end_time;
  } else {
    // Preintegrate yaw gyro
    // Initialize rotation that has been built up since last scan stamp
    Eigen::Matrix2d delta_C = lgmath::so2::vec2rot(preint_yaw_);
    double yaw_dt = 0.0;
    bool past_scan_time = false;
    double yaw_rate_avg = 0.0;
    if (qdata.gyro_msgs) {

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

    if (abs(v_r_v_in_r(0)) < config_->zero_velocity_threshold) {
      // If the velocity is very low, we assume no movement
      varpi_dt = Eigen::Matrix<double,3,1>::Zero();
      v_r_v_in_r = Eigen::Vector2d::Zero();
    }

    // Compute total change in state
    T_r_delta = lgmath::se3::vec2tran(vec2Dto3D(-varpi_dt));

    // Save T_r_m_new since we need both it and prev for undistortion
    const auto T_r_m_new = T_r_delta * T_r_m_prev;

    // Save prior velocity for next frame
    w_m_r_in_r_odo_prior(0) = -v_r_v_in_r(0);
    w_m_r_in_r_odo_prior(1) = -v_r_v_in_r(1);

    // Add poses and velocities to udist_trajectory for undistortion
    const Eigen::Vector3d v_m_r_in_r_prev(-v_r_v_in_r_prev_(0), -v_r_v_in_r_prev_(1), 0.0);
    const Eigen::Vector3d v_m_r_in_r(-v_r_v_in_r(0), -v_r_v_in_r(1), 0.0);
    udist_trajectory->add(Time(scan_stamp_prev_), SE2StateVar::MakeShared(T_r_m_prev.toSE2()),
                    VSpaceStateVar<3>::MakeShared(v_m_r_in_r_prev));
    udist_trajectory->add(Time(scan_stamp), SE2StateVar::MakeShared(T_r_m_new.toSE2()),
                    VSpaceStateVar<3>::MakeShared(v_m_r_in_r));

    // Save previous velocity for next frame
    v_r_v_in_r_prev_ = v_r_v_in_r;
    // Save current full velocity estimate
    w_v_r_in_r(0) = -v_r_v_in_r(0);
    w_v_r_in_r(1) = -v_r_v_in_r(1);
    // For our yaw rate, just use the middle gyro measurement
    // It doesn't affect pose so purely for velocity "estimation"
    const double yaw_r_v_in_r = qdata.gyro_msgs->at(qdata.gyro_msgs->size() / 2).angular_velocity.z;
    w_v_r_in_r(5) = -yaw_r_v_in_r;
  }

  // Propgate state
  *qdata.T_r_v_odo = T_r_delta * *qdata.T_r_v_odo;
  *qdata.T_r_m_odo = T_r_delta * T_r_m_prev;
  *qdata.T_r_m_odo_radar = T_r_delta * T_r_m_prev;

  // Save new velocity
  *qdata.w_v_r_in_r_odo = w_v_r_in_r;
  *qdata.w_m_r_in_r_odo_prior = w_m_r_in_r_odo_prior;

  // Undistort pointcloud
  // Compound transform for alignment (sensor to point map transform)
  Evaluable<lgmath::se2::Transformation>::ConstPtr T_r_m_eval = udist_trajectory->getPoseInterpolator(scan_stamp);
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
      const auto w_m_r_in_r_intp_eval = udist_trajectory->getVelocityInterpolator(Time(pt_time));
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
    const auto T_r_m_intp_eval = udist_trajectory->getPoseInterpolator(Time(pt_time));
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
  *qdata.T_r_m_odo_prior = T_r_delta * T_r_m_prev;
  *qdata.odo_success = true;
}

}  // namespace radar
}  // namespace vtr