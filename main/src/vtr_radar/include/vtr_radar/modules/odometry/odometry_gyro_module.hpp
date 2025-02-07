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
 * \file odometry_gyro_module.hpp
 * \author Yuchen Wu, Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/imu.hpp"

#include "steam.hpp"

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {

namespace radar {

/** \brief Gyro for odometry. */
class OdometryGyroModule : public tactic::BaseModule {
 public:
  using ImuMsg = sensor_msgs::msg::Imu;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.odometry_gyro";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // continuous-time estimation
    Eigen::Matrix<double, 6, 1> traj_qc_diag =
        Eigen::Matrix<double, 6, 1>::Ones();

    // gyro cov weight
    double gyro_cov = 0.1;


    // Steam optimization parameters 
    bool verbose = false;
    unsigned int max_iterations = 1;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryGyroModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryGyroModule);
};

}  // namespace radar
}  // namespace vtr