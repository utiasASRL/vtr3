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
 * \author Yuchen Wu, Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "steam.hpp"

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {

namespace radar {

/** \brief ICP for odometry. */
class OdometryICPModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.odometry_icp";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config,
                  public steam::VanillaGaussNewtonSolver::Params {
    PTR_TYPEDEFS(Config);

    /// Success criteria
    float min_matched_ratio = 0.4;

    // trajectory smoothing
    bool trajectory_smoothing = false;
    bool use_constant_acc = true;
    double lin_acc_std_dev_x = 1.0;
    double lin_acc_std_dev_y = 0.01;
    double lin_acc_std_dev_z = 0.01;
    double ang_acc_std_dev_x = 0.01;
    double ang_acc_std_dev_y = 0.01;
    double ang_acc_std_dev_z = 1.0;
    Eigen::Matrix<double, 6, 6> smoothing_factor_information =
        Eigen::Matrix<double, 6, 6>::Zero();

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

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr raw_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryICPModule);
};

}  // namespace radar
}  // namespace vtr