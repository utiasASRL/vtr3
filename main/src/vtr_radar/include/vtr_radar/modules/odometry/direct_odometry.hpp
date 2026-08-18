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
 * \file odometry_icp_module.hpp
 * \author Alec Krawciw, Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"
#include "vtr_radar/dro/dro_wrapper.hpp"
#include <pybind11/embed.h> // Required for embedding Python in a C++ main binary


namespace vtr {

namespace radar {

/** \brief Direct Radar Odometry wrapper. */
class DROModule : public tactic::BaseModule {
 private:
  // 1. Initialized first, destroyed last.
  py::scoped_interpreter guard_{};
    
  // 2. Holds Python object references.
  std::unique_ptr<DroWrapper> dro_;

  // 3. Keeps the GIL released during the engine's lifetime.
  std::unique_ptr<py::gil_scoped_release> release_guard_;
 public:
  using ImuData = DroWrapper::ImuData;
  using RadarData = DroWrapper::RadarData;
  using ImageMsg = sensor_msgs::msg::Image;

  
  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.odometry_dro";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    struct Estimation {
      bool use_gyro = true;
      bool estimate_gyro_bias = false;
      bool estimate_vy_bias = false;
      double vy_bias_prior = 0.0;
      double max_acceleration = 10.0;
      double min_time_bias_init = 1.0;
      double gyro_bias_alpha = 0.01;
      std::vector<double> T_axle_radar = {
          1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0
      };
    } estimation;

    struct GP {
      double lengthscale_az = 2.0;
      double lengthscale_range = 4.0;
      double sz = 0.6;
    } gp;

    struct Radar {
      double del_f = 893.0e6;
      double ft = 76.04e9;
      double meas_freq = 1600.0;
      double beta_corr_fact = 0.944;
      double range_offset = -0.31;
      int64_t nb_azimuths = -1;
      double resolution = -1.0;
      bool doppler_enabled = false;
    } radar;

    struct Direct {
      double min_range = 4.0;
      double max_range = 70.0;
      double local_map_res = 0.1;
      double max_local_map_range = 120.0;
      double local_map_update_alpha = 0.1;
    } direct;

    struct Doppler {
      double min_range = 4.0;
      double max_range = 200.0;
    } doppler;

    struct Solver {
      int64_t nb_iter = 250;
      double cost_tol = 1e-6;
      double step_tol = 1e-5;
    } solver;

    bool visualize = false;

    py::dict toPythonDict() const;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  DROModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);
  
  ~DROModule() {
    release_guard_.reset();
    dro_.reset();
  }

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  bool publisher_initialized_ = false;
  rclcpp::Publisher<ImageMsg>::SharedPtr local_map_pub_;


  VTR_REGISTER_MODULE_DEC_TYPE(DROModule);
};

}  // namespace radar
}  // namespace vtr