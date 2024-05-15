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
    int traj_num_extra_states = 0;
    bool traj_lock_prev_pose = false;
    bool traj_lock_prev_vel = false;
    Eigen::Matrix<double, 6, 1> traj_qc_diag =
        Eigen::Matrix<double, 6, 1>::Ones();

    int num_threads = 4;
    bool visualize = false;
    // steam parameters
    bool verbose = false;
    unsigned int max_iterations = 1;

    // DOPPLER
    int num_sensors = 1;
    int ransac_max_iter = 20;
    double ransac_thres = 0.2;
    double ransac_min_range = 20.0;
    int integration_steps = 100;
    double zero_vel_tol = 0.03;

    double min_dist = 20.0;
    double max_dist = 150.0;

    // inverse covariances
    Eigen::Matrix<double, 6, 6> Qkinv = Eigen::Matrix<double, 6, 6>::Identity(); 
    Eigen::Matrix<double, 6, 6> P0inv = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> Qzinv = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Vector3d const_gyro_bias = Eigen::Vector3d(-0.004580390732042348, -0.015914139544965403, 0.002919723147493117);
    
    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryDopplerModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

 protected: 
  // vector to track elapsed time
  std::vector<double> tot_timers{0.0, 0.0, 0.0, 0.0, 0.0};

  // precomputed measurement model (to avoid repeated calculations in RANSAC and main solve)
  Eigen::Matrix<double,Eigen::Dynamic,6> ransac_precompute_;
  Eigen::Matrix<double,Eigen::Dynamic,1> meas_precompute_;
  Eigen::Matrix<double,Eigen::Dynamic,1> alpha_precompute_;
  Eigen::Matrix<double,Eigen::Dynamic,1> malpha_precompute_;

  // ransac generator
  long int seed_ = 0;
  std::mt19937_64 random_engine_;  

  // extrinsic
  std::vector<Eigen::Matrix4d> T_sv_;
  std::vector<Eigen::Matrix<double,3,6>> adT_sv_top3rows_;

  // precompute
  Eigen::Matrix<double, 12, 12> wnoa_lhs_;

  // gyro inverse covariance
  std::vector<Eigen::Matrix3d> gyro_invcov_;

 private:
  std::vector<Eigen::MatrixXd> next_gyro(const double &start_time, 
                                         const double &end_time, 
                                         const std::vector<Eigen::MatrixXd> &gyro);

  void run_(tactic::QueryCache &qdata, 
            tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryDopplerModule);
};

}  // namespace lidar
}  // namespace vtr