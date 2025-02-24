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
 * \file odometry_preintegration_module.cpp
 * \author Sven Lilge, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/odometry/odometry_preintegration_module.hpp"

#include "vtr_lidar/utils/nanoflann_utils.hpp"

namespace vtr {
namespace lidar {

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryPreintegrationModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  const auto gyro_noise_ = node->declare_parameter<std::vector<double>>(param_prefix + ".gyro_noise", std::vector<double>());
  if (gyro_noise_.size() != 3) {
    std::string err{"Gyro noise malformed. Must be 3 elements!"};
    CLOG(ERROR, "lidar.odometry_preintegration") << err;
    throw std::invalid_argument{err};
  }
  config->gyro_noise.diagonal() << gyro_noise_[0], gyro_noise_[1], gyro_noise_[2];
  // clang-format on
  return config;
}

void OdometryPreintegrationModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  // convert gyro message to Eigen matrix
  if (qdata.gyro_msg) {
    Eigen::MatrixXd gyro_data(1, 3);
    gyro_data(0, 0) = qdata.gyro_msg->angular_velocity.x;
    gyro_data(0, 1) = qdata.gyro_msg->angular_velocity.y;
    gyro_data(0, 2) = qdata.gyro_msg->angular_velocity.z;
    *qdata.gyro.emplace(gyro_data);
    *qdata.gyro_stamp.emplace(*qdata.stamp);
  } // will have only one row when live

  // Ensure that gyro and gyro_stamp have the same length
  if (qdata.gyro->rows() != qdata.gyro_stamp->size()) {
    throw std::runtime_error("Mismatch between gyro data and gyro timestamp sizes.");
  }

  // Do nothing if qdata does not contain any gyro data
  // Also do nothing, if odometry has not been initialized
  if(!qdata.gyro || !qdata.sliding_map_odo || !qdata.stamp_end_pre_integration) {
    CLOG(WARNING, "lidar.odometry_preintegration") << "No gyro data or odometry data, returning.";
    return;
  }

  if(qdata.gyro && !qdata.prev_gyro) {
    // This is the first time we are every receiving gyro, we cannot preintegrate
    // Save current gyro message and update preintegration terms
    qdata.prev_gyro.emplace(qdata.gyro->row(qdata.gyro->rows() - 1).rightCols<3>());
    *qdata.stamp_end_pre_integration = qdata.gyro_stamp->back();
    *qdata.stamp_start_pre_integration = qdata.gyro_stamp->back();
    return;
  }

  CLOG(DEBUG, "lidar.odometry_preintegration") << "Number of gyro measurements: " << qdata.gyro->rows();

  for (int i = 0; i < qdata.gyro->rows(); ++i) {
    // Inputs
    const auto &current_time_stamp = qdata.gyro_stamp->at(i);
    const auto &prev_gyro = *qdata.prev_gyro;
    const auto &prev_time_stamp = *qdata.stamp_end_pre_integration;
    const auto row = qdata.gyro->row(i).rightCols<3>();

    // Distance between current and last time
    Time prev_time(static_cast<int64_t>(prev_time_stamp)); // get previous odometry timestamp
    Time cur_time(static_cast<int64_t>(current_time_stamp));
    const auto delta_time = (cur_time - prev_time).seconds();
    CLOG(DEBUG, "lidar.odometry_preintegration") << "Previous time stamp : " << prev_time_stamp;
    CLOG(DEBUG, "lidar.odometry_preintegration") << "Current time stamp  : " << current_time_stamp;
    CLOG(DEBUG, "lidar.odometry_preintegration") << "Delta time          : " << delta_time;
    
    // Integrate last gyro measurement
    Eigen::Matrix3d delta_gyro = lgmath::so3::vec2rot(prev_gyro * delta_time); // exp(u_k-1 * dT)
    CLOG(DEBUG, "lidar.odometry_preintegration") << "Previous gyro data  : " << prev_gyro.transpose();
    CLOG(DEBUG, "lidar.odometry_preintegration") << std::endl << "Delta gyro: " << std::endl << delta_gyro;

    // Set preintegrated value
    Eigen::Matrix3d value = Eigen::Matrix3d::Identity(); // D C_ii
    Eigen::Matrix3d sigma_prev = Eigen::Matrix3d::Zero(); // Sigma_ii

    // Check if we have a pre-integrated value
    if (qdata.preintegrated_delta_gyro) {
      value = lgmath::so3::vec2rot(*qdata.preintegrated_delta_gyro);  // if the first meas, then D C_ii = 1, else it is previous D C_ik-1
      sigma_prev = *qdata.preintegrated_gyro_cov;                     // if the first meas, then Sigma_ii = 0, else it is previous Sigma_ik-1
    }

    CLOG(DEBUG, "lidar.odometry_preintegration") << std::endl << "Previous value: " << std::endl << value;
    CLOG(DEBUG, "lidar.odometry_preintegration") << std::endl << "Previous covar:" << std::endl << sigma_prev;

    value = value * delta_gyro; // want value to be _pc
    CLOG(DEBUG, "lidar.odometry_preintegration") << std::endl << "Current value: " << std::endl << value;

    Eigen::Matrix3d F = -1 * delta_gyro.transpose();
    double L = -1 * delta_time;

    // approxmate with the assumption that DT is small
    Eigen::Matrix3d sigma_curr = (F * sigma_prev * F.transpose()) + (L * config_->gyro_noise * L);
    Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
    sigma.diagonal() = sigma_curr.diagonal();
    CLOG(DEBUG, "lidar.odometry_preintegration") << std::endl << "Current covar:" << std::endl << sigma;
    CLOG(DEBUG, "lidar.odometry_preintegration") << std::endl << "first part: " << std::endl << (F * sigma_prev * F.transpose());
    
    if (!qdata.preintegrated_delta_gyro) {
      qdata.preintegrated_delta_gyro.emplace(lgmath::so3::rot2vec(value));
      qdata.preintegrated_gyro_cov.emplace(sigma);
    } else {
      *qdata.preintegrated_delta_gyro = lgmath::so3::rot2vec(value);
      *qdata.preintegrated_gyro_cov = sigma;
    }

    // Set preintegration end time to current time
    *qdata.stamp_end_pre_integration = current_time_stamp;

    // Set prev gyro msg to the current message for the next preintegration
    qdata.prev_gyro = std::make_shared<const Eigen::Vector3d>(row);  

    CLOG(DEBUG, "lidar.odometry_preintegration") << std::endl;
  }
  }
  // clang-format on
}  // namespace lidar
}  // namespace vtr