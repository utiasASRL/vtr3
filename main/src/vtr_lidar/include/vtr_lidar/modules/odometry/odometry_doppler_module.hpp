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
 * \file odometry_doppler_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "steam.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

#include <random>

namespace vtr {
namespace lidar {

/** \brief Doppler for odometry. */
class OdometryDopplerModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.odometry_doppler";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // continuous-time estimation
    bool use_trajectory_estimation = false;
    bool traj_lock_prev_pose = false;
    bool traj_lock_prev_vel = false;
    Eigen::Matrix<double, 6, 1> traj_qc_diag =
        Eigen::Matrix<double, 6, 1>::Ones();
    //
    int num_threads = 4;
    int loc_threshold = 1;
    
    // doppler odom parameters
    long int ransac_seed = 0;
    bool ransac_gyro = false;
    int ransac_max_iter = 20;
    double ransac_threshold = 0.2;
    double prior_threshold = 6.0;
    double ransac_min_range = 20.0;
    int integration_steps = 100;
    double zero_vel_tol = 0.03;

    // gyro
    Eigen::Matrix<double, 3, 3> gyro_invcov =
        Eigen::Matrix<double, 3, 3>::Identity();

    // inverse covariances
    Eigen::Matrix<double, 6, 6> Qkinv = Eigen::Matrix<double, 6, 6>::Identity(); 
    Eigen::Matrix<double, 6, 6> P0inv = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> Qzinv = Eigen::Matrix<double, 6, 6>::Identity();
    // init covariance
    Eigen::Matrix<double, 6, 6> P0 = Eigen::Matrix<double, 6, 6>::Identity();
    // Success criteria
    float max_trans_vel_diff = 1000.0; // m/s
    float max_rot_vel_diff = 1000.0; // m/s
    float max_trans_diff = 1000.0; // m
    float max_rot_diff = 1000.0; // rad
    
    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryDopplerModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

 protected: 
  int frame_count = 0;

  // ransac generator
  std::mt19937_64 random_engine_;  

  // extrinsic
  Eigen::Matrix<double,3,6> adT_sv_top3rows_;

  // precompute
  Eigen::Matrix<double, 12, 12> wnoa_lhs_;

  // save linear system
  Eigen::Matrix<double, 6, 6> last_lhs_;
  Eigen::Matrix<double, 6, 1> last_rhs_;

  // save for covariance of body centric velocity
  Eigen::Matrix<double, 12, 12> cov_k_k1;     // covariance of x_k, x_k+1
  Eigen::Matrix<double, 6, 6> Qc;             // matrix version of Qc_diag
  Eigen::Matrix<double, 6, 6> cov_T_k;        // covariance of pose T_k

  // gyro inverse covariance
  Eigen::Matrix3d gyro_invcov_;

  void run_(tactic::QueryCache &qdata, 
            tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryDopplerModule);
};

}  // namespace lidar
}  // namespace vtr