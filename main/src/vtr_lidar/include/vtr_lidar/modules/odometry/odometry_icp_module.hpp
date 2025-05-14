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
 * \file odometry_icp_module.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "steam.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {

namespace lidar {

/** \brief ICP for odometry. */
class OdometryICPModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.odometry_icp";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // continuous-time estimation
    bool use_trajectory_estimation = false;
    int traj_num_extra_states = 0;
    Eigen::Matrix<double, 6, 1> traj_qc_diag =
        Eigen::Matrix<double, 6, 1>::Ones();

    /// ICP parameters
    // number of threads for nearest neighbor search
    int num_threads = 4;
    // initial alignment config
    size_t first_num_steps = 3;
    size_t initial_max_iter = 100;
    float initial_max_pairing_dist = 2.0;
    float initial_max_planar_dist = 0.3;
    // refined stage
    size_t refined_max_iter = 10;  // we use a fixed number of iters for now
    float refined_max_pairing_dist = 2.0;
    float refined_max_planar_dist = 0.1;
    // error calculation
    float averaging_num_steps = 5;
    float trans_diff_thresh = 0.01;              // threshold on variation of T
    float rot_diff_thresh = 0.1 * M_PI / 180.0;  // threshold on variation of R
    // steam optimizer
    bool verbose = false;
    unsigned int max_iterations = 1;

    // gyro weight
    double gyro_cov = 1e-3;
    bool remove_orientation = false;

    /// Success criteria
    float min_matched_ratio = 0.4;
    float max_trans_vel_diff = 1000.0; // m/s
    float max_rot_vel_diff = 1000.0; // m/s
    float max_trans_diff = 1000.0; // m
    float max_rot_diff = 1000.0; // rad

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryICPModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryICPModule);
};

}  // namespace lidar
}  // namespace vtr