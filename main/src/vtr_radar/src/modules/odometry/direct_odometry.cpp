// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file odometry_icp_module.cpp
 * \author Alec Krawciw Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/direct_odometry.hpp"

#include "vtr_common/conversions/se2_to_se3.hpp"

namespace vtr {
namespace radar {

using namespace tactic;
using namespace common::conversions;

auto DROModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  // 1. Estimation
  config->estimation.use_gyro = node->declare_parameter<bool>(param_prefix + ".estimation.use_gyro", config->estimation.use_gyro);
  config->estimation.estimate_gyro_bias = node->declare_parameter<bool>(param_prefix + ".estimation.estimate_gyro_bias", config->estimation.estimate_gyro_bias);
  config->estimation.estimate_vy_bias = node->declare_parameter<bool>(param_prefix + ".estimation.estimate_vy_bias", config->estimation.estimate_vy_bias);
  config->estimation.vy_bias_prior = node->declare_parameter<double>(param_prefix + ".estimation.vy_bias_prior", config->estimation.vy_bias_prior);
  config->estimation.max_acceleration = node->declare_parameter<double>(param_prefix + ".estimation.max_acceleration", config->estimation.max_acceleration);
  config->estimation.min_time_bias_init = node->declare_parameter<double>(param_prefix + ".estimation.min_time_bias_init", config->estimation.min_time_bias_init);
  config->estimation.gyro_bias_alpha = node->declare_parameter<double>(param_prefix + ".estimation.gyro_bias_alpha", config->estimation.gyro_bias_alpha);
  config->estimation.ang_vel_bias = node->declare_parameter<double>(param_prefix + ".estimation.ang_vel_bias", config->estimation.ang_vel_bias);

  auto t_axle_vec = node->declare_parameter<std::vector<double>>(param_prefix + ".estimation.T_axle_radar", config->estimation.T_axle_radar);
  if (t_axle_vec.size() == 16) {
    config->estimation.T_axle_radar = std::move(t_axle_vec);
  } else {
    CLOG(WARNING, static_name) << "'estimation.T_axle_radar' must have 16 elements. Falling back to default identity matrix.";
  }

  // 2. GP
  config->gp.lengthscale_az = node->declare_parameter<double>(param_prefix + ".gp.lengthscale_az", config->gp.lengthscale_az);
  config->gp.lengthscale_range = node->declare_parameter<double>(param_prefix + ".gp.lengthscale_range", config->gp.lengthscale_range);
  config->gp.sz = node->declare_parameter<double>(param_prefix + ".gp.sz", config->gp.sz);

  // 3. Radar
  config->radar.del_f = node->declare_parameter<double>(param_prefix + ".radar.del_f", config->radar.del_f);
  config->radar.ft = node->declare_parameter<double>(param_prefix + ".radar.ft", config->radar.ft);
  config->radar.meas_freq = node->declare_parameter<double>(param_prefix + ".radar.meas_freq", config->radar.meas_freq);
  config->radar.beta_corr_fact = node->declare_parameter<double>(param_prefix + ".radar.beta_corr_fact", config->radar.beta_corr_fact);
  config->radar.range_offset = node->declare_parameter<double>(param_prefix + ".radar.range_offset", config->radar.range_offset);
  config->radar.nb_azimuths = node->declare_parameter<int64_t>(param_prefix + ".radar.nb_azimuths", config->radar.nb_azimuths);
  config->radar.resolution = node->declare_parameter<double>(param_prefix + ".radar.resolution", config->radar.resolution);
  config->radar.doppler_enabled = node->declare_parameter<bool>(param_prefix + ".radar.doppler_enabled", config->radar.doppler_enabled);

  // 4. Direct
  config->direct.min_range = node->declare_parameter<double>(param_prefix + ".direct.min_range", config->direct.min_range);
  config->direct.max_range = node->declare_parameter<double>(param_prefix + ".direct.max_range", config->direct.max_range);
  config->direct.local_map_res = node->declare_parameter<double>(param_prefix + ".direct.local_map_res", config->direct.local_map_res);
  config->direct.max_local_map_range = node->declare_parameter<double>(param_prefix + ".direct.max_local_map_range", config->direct.max_local_map_range);
  config->direct.local_map_update_alpha = node->declare_parameter<double>(param_prefix + ".direct.local_map_update_alpha", config->direct.local_map_update_alpha);

  // 5. Doppler
  config->doppler.min_range = node->declare_parameter<double>(param_prefix + ".doppler.min_range", config->doppler.min_range);
  config->doppler.max_range = node->declare_parameter<double>(param_prefix + ".doppler.max_range", config->doppler.max_range);

  // 6. Solver
  config->solver.nb_iter = node->declare_parameter<int64_t>(param_prefix + ".solver.nb_iter", config->solver.nb_iter);
  config->solver.cost_tol = node->declare_parameter<double>(param_prefix + ".solver.cost_tol", config->solver.cost_tol);
  config->solver.step_tol = node->declare_parameter<double>(param_prefix + ".solver.step_tol", config->solver.step_tol);

  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  config->num_threads = node->declare_parameter<int64_t>(param_prefix + ".num_threads", config->num_threads);
  // clang-format on
  return config;
}

py::dict DROModule::Config::toPythonDict() const {
  py::dict opts;

  // 1. estimation
  py::dict estimation;
  estimation["use_gyro"] = this->estimation.use_gyro;
  estimation["estimate_gyro_bias"] = this->estimation.estimate_gyro_bias;
  estimation["estimate_vy_bias"] = this->estimation.estimate_vy_bias;
  estimation["vy_bias_prior"] = this->estimation.vy_bias_prior;
  estimation["max_acceleration"] = this->estimation.max_acceleration;
  estimation["min_time_bias_init"] = this->estimation.min_time_bias_init;
  estimation["gyro_bias_alpha"] = this->estimation.gyro_bias_alpha;
  estimation["ang_vel_bias"] = this->estimation.ang_vel_bias;
  estimation["T_axle_radar"] = py::array_t<double>({4, 4}, this->estimation.T_axle_radar.data());
  opts["estimation"] = estimation;

  // 2. gp
  py::dict gp;
  gp["lengthscale_az"] = this->gp.lengthscale_az;
  gp["lengthscale_range"] = this->gp.lengthscale_range;
  gp["sz"] = this->gp.sz;
  opts["gp"] = gp;

  // 3. radar
  py::dict radar;
  radar["del_f"] = this->radar.del_f;
  radar["ft"] = this->radar.ft;
  radar["meas_freq"] = this->radar.meas_freq;
  radar["beta_corr_fact"] = this->radar.beta_corr_fact;
  radar["range_offset"] = this->radar.range_offset;
  radar["nb_azimuths"] = (this->radar.nb_azimuths > 0) ? py::cast(this->radar.nb_azimuths) : py::none();
  radar["resolution"] = (this->radar.resolution > 0.0) ? py::cast(this->radar.resolution) : py::none();
  radar["doppler_enabled"] = this->radar.doppler_enabled;
  opts["radar"] = radar;

  // 4. direct
  py::dict direct;
  direct["min_range"] = this->direct.min_range;
  direct["max_range"] = this->direct.max_range;
  direct["local_map_res"] = this->direct.local_map_res;
  direct["max_local_map_range"] = this->direct.max_local_map_range;
  direct["local_map_update_alpha"] = this->direct.local_map_update_alpha;
  opts["direct"] = direct;

  // 5. doppler
  py::dict doppler;
  doppler["min_range"] = this->doppler.min_range;
  doppler["max_range"] = this->doppler.max_range;
  opts["doppler"] = doppler;

  // 6. solver
  py::dict solver;
  solver["nb_iter"] = this->solver.nb_iter;
  solver["cost_tol"] = this->solver.cost_tol;
  solver["step_tol"] = this->solver.step_tol;
  opts["solver"] = solver;

  // 7. thread cap (manually controlled; unset/<=0 leaves torch's default)
  opts["num_threads"] = (this->num_threads > 0) ? py::cast(this->num_threads) : py::none();

  return opts;
}

DROModule::DROModule(const Config::ConstPtr &config, const std::shared_ptr<tactic::ModuleFactory> &module_factory,
    const std::string &name): tactic::BaseModule(module_factory, name), config_(config)
{
  dro_ = std::make_unique<DroWrapper>(config_->toPythonDict());
  release_guard_ = std::make_unique<py::gil_scoped_release>();
}


void DROModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  if(!qdata.radar_data)
  {
    return;
  }

  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    local_map_pub_ = qdata.node->create_publisher<ImageMsg>("local_map", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  cv::Mat local_map;

  RadarData rd;
  rd.polar = qdata.radar_data->fft_scan;
  rd.azimuths = qdata.radar_data->azimuth_angles;
  rd.chirps = qdata.radar_data->up_chirps;


  rd.timestamps.reserve(qdata.radar_data->azimuth_times.size());
  for(const auto& t_i : qdata.radar_data->azimuth_times) {
    rd.timestamps.push_back(t_i / 1000);
  }

  std::vector<ImuData> relevant_imus;
  relevant_imus.reserve(qdata.gyro_msgs->size());
  int imu_state = 0;

  // Gyro measurements arrive in the gyro's own sensor frame. DRO expects
  // angular_velocity already expressed in the radar (sensor) frame
  const auto &T_s_r_gyro = *qdata.T_s_r_gyro;
  const Eigen::Matrix3d R_radar_gyro =
      qdata.T_s_r->matrix().block<3, 3>(0, 0) *
      T_s_r_gyro.matrix().block<3, 3>(0, 0).transpose();

  double middle_yaw_rate;
  bool middle_yaw_rate_set = false;
  for(const auto& msg : *qdata.gyro_msgs) {
    ImuData imu_data;
    imu_data.timestamp = static_cast<int64_t>(msg.header.stamp.sec) * 1000000LL +
                  static_cast<int64_t>(msg.header.stamp.nanosec) / 1000LL;

    // Rotate into the radar frame
    // We don't load acceleration because it's not needed
    imu_data.angular_velocity = R_radar_gyro * Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

    if (imu_data.timestamp < rd.timestamps.front()) {
      if(imu_state == 0)
        imu_state++;
      CLOG(DEBUG, static_name) << "Start message found. dt: " << static_cast<float>(imu_data.timestamp - rd.timestamps.front()) / 1e6;
    }

    if (imu_data.timestamp > *qdata.stamp / 1000 && !middle_yaw_rate_set) {
      middle_yaw_rate = imu_data.angular_velocity(2);
      middle_yaw_rate_set = true;
    }

    if (imu_data.timestamp > rd.timestamps.back()) {
      if(imu_state == 1)
        imu_state++;
      CLOG(DEBUG, static_name) << "End message found. dt: " << static_cast<float>(imu_data.timestamp - rd.timestamps.back()) / 1e6;
    }
    relevant_imus.push_back(imu_data);
  }

  if (imu_state != 2) {
    CLOG(ERROR, static_name) << "IMU data does not wrap around the scan times. DRO is considered failed";
    *qdata.odo_success = false;
    return;
  }

  py::gil_scoped_acquire acquire;
  const auto [T_delta, vel] = dro_->odometryStep(rd, relevant_imus, local_map);
  CLOG(DEBUG, static_name) << "DRO: vx " << vel(0) << ", vy " << vel(1);

  const auto& T_s_r = *qdata.T_s_r;
  const auto T_r_s = T_s_r.inverse();
  const auto Ad_T_r_s = T_r_s.adjoint();

  // Transform motion from sensor frame to robot frame
  Eigen::Vector3d vel_s;
  if (vel.size() == 3)
    vel_s << vel(0), vel(1), vel(2);
  else
    vel_s << vel(0), vel(1), middle_yaw_rate;
  *qdata.w_v_r_in_r_odo = Ad_T_r_s * vec2Dto3D(-vel_s);

  Eigen::Matrix4d T_w_s = dro_->getPose(*qdata.stamp / 1000);
  Eigen::Matrix4d T_r_rlast = (T_w_s * T_s_r.matrix()).inverse() * T_w_s_last_ * T_s_r.matrix();

  Eigen::Matrix4d T_delta_robot_frame = T_s_r.inverse().matrix() * T_delta.inverse() * T_s_r.matrix();

  CLOG(DEBUG, static_name) << "T_delta\n" << T_delta_robot_frame << "\nOld\n" << T_r_rlast;

  *qdata.T_r_v_odo = tactic::EdgeTransform(T_r_rlast) * *qdata.T_r_v_odo;
  qdata.T_r_v_odo->setZeroCovariance();
  *qdata.odo_success = true;
  T_w_s_last_ = T_w_s;

  qdata.smoothed_scan.emplace(local_map);
  
  CLOG(DEBUG, static_name) << "DRO odom\n" << qdata.T_r_v_odo->matrix();
  CLOG(DEBUG, static_name) << "DRO odom vel\n" << qdata.w_v_r_in_r_odo->transpose();


  if(config_->visualize) {
    // publish the fft scan image
    cv_bridge::CvImage local_map_image;
    local_map_image.header.frame_id = "radar";
    // local_map_image.header.stamp = qdata.scan_msg->header.stamp;
    local_map_image.encoding = "mono8";
    local_map.convertTo(local_map_image.image, CV_8UC1, 255);
    local_map_pub_->publish(*local_map_image.toImageMsg());
  }


  // clang-format on
}

}  // namespace radar
}  // namespace vtr