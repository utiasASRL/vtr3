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
 * \file odometry_dense_module.hpp
 * \author 
 */
#pragma once
#include "steam.hpp"
#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"
// #include "vtr_radar/modules/odometry/dense_utils/motion_models.hpp"

namespace vtr {
namespace radar {


class GPStateEstimator;   // <- no include here

/** \brief Dense Image for odometry. */
class OdometryDenseModule : public tactic::BaseModule {
 public:

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.odometry_dense";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // estimation //may need to change float to double
    bool doppler_cost = false;
    bool direct_cost = true;
    std::string motion_model = "const_body_vel_gyro";
    bool gyro_bias_estimation = true;
    bool estimate_gyro_bias = true;
    float max_acceleration = 2.0;
    float optimization_first_step = 0.1;
    double vy_bias_prior = 0.0;
    bool estimate_doppler_vy_bias = false;
    bool use_gyro = true;
    float ang_vel_bias = 0.0;

    // gp 
    float lengthscale_az = 2.0;
    float lengthscale_range = 4.0;
    double sz = 0.6;

    // radar
    double ft = 76.04E9;
    double meas_freq = 1600.0;
    double del_f = 893.0E6;
    double beta_corr_fact = 0.944;
    bool doppler_enabled = false;
    bool chirp_up = true;  // Ignored if doppler_enabled is true (the true/false might be inverted)
    double range_offset = 0.00;
    double radar_resolution = 0.040308;  // meters per pixel

    // direct
    float min_range = 4.0;
    float max_range = 68.0;  //30.0
    double max_local_map_range = 100.0;  //100.0
    double local_map_res = 0.05;  //0.1
    double local_map_update_alpha = 0.1;  // The local map is updated as (1-alpha)*prev_map + alpha*current_scan

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OdometryDenseModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;
  // GPStateEstimator state_estimator_;
  std::shared_ptr<GPStateEstimator> state_estimator_;
  bool initialized = false;
  int frame_idx = 0;

  VTR_REGISTER_MODULE_DEC_TYPE(OdometryDenseModule);
};

}  // namespace radar
}  // namespace vtr