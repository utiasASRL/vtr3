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
 * \file radar_extraction_module.hpp
 * \author Sam Qiao, Autonomous Space Robotics Lab (ASRL)
 */
// For more details on the pointcloud extraction methods and parameter selection,
// please refer to the following paper: https://arxiv.org/abs/2409.12256
#pragma once

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/** \brief Extracts keypoints from Navtech radar scans. */
class RadarExtractionModule : public tactic::BaseModule {
 public:
  using ImageMsg = sensor_msgs::msg::Image;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.pc_extractor";

  /** \brief Config parameters. */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    std::string detector = "kpeaks";
    double minr = 1;
    double maxr = 69;
    // kpeaks
    struct {
      int kstrong = 4;
      double threshold2 = 0.5;
      double threshold3 = 0.22;
    } kpeaks;
    // kstrongest
    struct {
      int kstrong = 10;
      double static_threshold = 0.22;
    } kstrongest;
    // cen2018
    struct {
      double zq = 3;
      int sigma = 17;
    } cen2018;
    // oscfar
    struct {
      int width = 40;
      int guard = 2;
      int kstat = 20;
      double threshold = 0.5;
    } oscfar;
    // tm_cfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 15.0;
      int N1 = 5;
      int N2 = 5;
    } tm_cfar;
    // cacfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 0.5;
    } cacfar;
    // modified_cacfar 
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 1.0;
      double threshold2 = 0.5;
      double threshold3 = 0.22;
    } modified_cacfar;
    // cago_cfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 15.0;
    } cago_cfar;
     // caso_cfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 0.5;
    } caso_cfar;
    //is_cfar
    struct {
      int width = 40;
      int guard = 2;
      double alpha_I = 0.05;
      int N_TI = 7;
      double beta_I = 20.02;
    } is_cfar;
    //vi_cfar
    struct {
      int width = 40;
      int guard = 2;
      double alpha_I = 0.05;
      double K_VI = 7.0;
      double K_MR = 1.5;
      double C_N = 20.02;
    } vi_cfar;
    // cfear_kstrong
    struct {
      int width = 40;
      int guard = 2;
      int kstrong = 12;
      double z_min = 0.2;
      double r = 3.5;
      double f = 1.0;
    } cfear_kstrong;
    // bfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 0.5;
      double static_threshold = 0.22;
    } bfar;
    // msca_cfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 0.5;
      int M = 5;
    } msca_cfar;
    // cen2019
    struct {
      int width = 40;
      int guard = 2;
      int l_max = 200;
    } cen2019;

    double radar_resolution = 0.0438;
    double range_offset = -0.31;
    double cart_resolution = 0.25;
    double beta = 0.05;
    bool upfront_range_corr = false;

    bool save_pointcloud_overlay = false;
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  RadarExtractionModule(
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

  VTR_REGISTER_MODULE_DEC_TYPE(RadarExtractionModule);
};

}  // namespace radar
}  // namespace vtr