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
  if (qdata.gyro->rows() != static_cast<Eigen::Index>(qdata.gyro_stamp->size())) {
    throw std::runtime_error("Mismatch between gyro data and gyro timestamp sizes.");
  }

  // Do nothing if qdata does not contain any gyro data
  // Also do nothing, if odometry has not been initialized
  if(!qdata.gyro || !qdata.sliding_map_odo) {
    CLOG(WARNING, "lidar.odometry_preintegration") << "No gyro data or odometry data, returning.";
    return;
  }

  CLOG(DEBUG, "lidar.odometry_preintegration") << "Number of gyro measurements: " << qdata.gyro->rows();

  // Reset preintegrated value
  Eigen::Matrix3d RMI_C = Eigen::Matrix3d::Identity(); // D C_ii
  Eigen::Matrix3d Sigma_RMI = Eigen::Matrix3d::Zero(); // Sigma_ii

  for (int i = 1; i < qdata.gyro->rows(); ++i) {
    // Inputs
    const auto prev_row = qdata.gyro->row(i-1).rightCols<3>();
    const auto &prev_gyro = std::make_shared<const Eigen::Vector3d>(prev_row);
    const auto &prev_time_stamp = qdata.gyro_stamp->at(i-1);
    const auto &current_time_stamp = qdata.gyro_stamp->at(i);

    // Distance between current and last time
    Time prev_time(static_cast<int64_t>(prev_time_stamp)); // get previous odometry timestamp
    Time cur_time(static_cast<int64_t>(current_time_stamp));
    const auto delta_time = (cur_time - prev_time).seconds();
    
    // Integrate last gyro measurement
    Eigen::Matrix3d delta_gyro = lgmath::so3::vec2rot((*prev_gyro) * delta_time).transpose(); // exp(u_k-1 * dT)    CLOG(DEBUG, "lidar.odometry_preintegration") << "Previous gyro data  : " << prev_gyro->transpose();
    RMI_C = RMI_C * delta_gyro; // want value to be _pc

    Eigen::Matrix3d F = -1 * delta_gyro.transpose();
    double L = -1 * delta_time;

    // approxmate with the assumption that DT is small
    Sigma_RMI = (F * Sigma_RMI * F.transpose()) + (L * config_->gyro_noise * L);
    Eigen::Matrix3d Sigma_new = Eigen::Matrix3d::Zero();
    Sigma_new.diagonal() = Sigma_RMI.diagonal();
    Sigma_RMI = Sigma_new;
  }

  CLOG(DEBUG, "lidar.odometry_preintegration") << "Final preintegrated value: " << std::endl << RMI_C;
  CLOG(DEBUG, "lidar.odometry_preintegration") << "Final preintegrated covar: " << std::endl << Sigma_RMI;

  // Save result
  if (!qdata.preintegrated_delta_gyro) {
    qdata.preintegrated_delta_gyro.emplace(lgmath::so3::rot2vec(RMI_C));
    qdata.preintegrated_gyro_cov.emplace(Sigma_RMI);
    qdata.stamp_start_pre_integration.emplace(qdata.gyro_stamp->front());
    qdata.stamp_end_pre_integration.emplace(qdata.gyro_stamp->back());
  } else {
    *qdata.preintegrated_delta_gyro = lgmath::so3::rot2vec(RMI_C);
    *qdata.preintegrated_gyro_cov = Sigma_RMI;
    *qdata.stamp_start_pre_integration = qdata.gyro_stamp->front();
    *qdata.stamp_end_pre_integration = qdata.gyro_stamp->back();
  }

  
  }
  // clang-format on
}  // namespace lidar
}  // namespace vtr