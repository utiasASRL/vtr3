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
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "steam.hpp"

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {

namespace radar {

/** \brief Direct Doppler-based odometry. */
class OdometryDopplerModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.odometry_doppler";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // Undistortion trajectory parameters
    Eigen::Matrix<double, 3, 1> traj_qc_diag =
        Eigen::Matrix<double, 3, 1>::Ones();
    // Number of threads for pointcloud processing
    int num_threads = 4;
    // Threshold for zeroing out velocity
    double zero_velocity_threshold = 0.1;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryDopplerModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  int64_t scan_stamp_prev_ = 0;
  Eigen::Vector2d v_r_v_in_r_prev_ = Eigen::Vector2d::Zero(); // Extracted velocity from previous frame
  double yaw_rate_prev_ = 0.0;  // Last gyro yaw rate from previous frame
  double preint_yaw_ = 0.0;
  double yaw_rate_avg_prev_ = 0.0;
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryDopplerModule);
};

}  // namespace radar
}  // namespace vtr