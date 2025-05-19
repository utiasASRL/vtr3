// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file doppler_extraction_module.hpp
 * \author Daniil Lisus, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/** \brief Extracts keypoints from Navtech radar scans. */
class DopplerExtractionModule : public tactic::BaseModule {
 public:
  using ImageMsg = sensor_msgs::msg::Image;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.doppler_extractor";

  /** \brief Config parameters. */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    // General radar params
    double radar_resolution = 0.0438;
    double f_t = 76.04e9;    // Hz, start frequency ramp
    double meas_freq = 1600; // Hz, number of measurements per second
    double del_f = 893.0e6;  // Hz, frequency span during ramp

    // Extraction params
    double minr = 4;
    double maxr = 200;
    double beta_corr_fact = 0.944;
    int pad_num = 100;

    // Filter params
    double sigma_gauss = 15;
    double z_q = 2.5;

    // RANSAC params
    int vel_dim = 2;
    int ransac_max_iter = 1000;
    double ransac_threshold = 0.5;
    double ransac_prior_threshold = 0.5;

    // Estimation params
    int opt_max_iter = 100;
    double opt_threshold = 1.0e-6;
    double cauchy_rho = 0.8;
    double trim_dist = 1000.0;
    double x_bias_slope = 0.0;
    double x_bias_intercept = 0.0;
    double y_bias_slope = 0.0;
    double y_bias_intercept = 0.0;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  DopplerExtractionModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<ImageMsg>::SharedPtr scan_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr fft_scan_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr bev_scan_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr pointcloud_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(DopplerExtractionModule);
};

}  // namespace radar
}  // namespace vtr