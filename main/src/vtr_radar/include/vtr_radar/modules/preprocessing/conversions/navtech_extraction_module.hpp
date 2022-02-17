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
 * \file navtech_extraction_module.hpp
 * \author Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 * \brief NavtechExtractionModule class definition
 */
#pragma once

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/** \brief Extracts keypoints from Navtech radar scans. */
class NavtechExtractionModule : public tactic::BaseModule {
 public:
  using ImageMsg = sensor_msgs::msg::Image;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.navtech_extractor";

  /** \brief Config parameters. */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    std::string detector = "kstrongest";
    double minr = 2;
    double maxr = 100;

    // kstrong
    struct {
      int kstrong = 10;
      double threshold2 = 0.5;
      double threshold3 = 0.22;
    } kstrong;
    // cen2018
    struct {
      double zq = 3;
      int sigma = 17;
    } cen2018;
    // cacfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 0.5;
      double threshold2 = 0.5;
      double threshold3 = 0.22;
    } cacfar;
    // oscfar
    struct {
      int width = 40;
      int guard = 2;
      int kstat = 20;
      double threshold = 0.5;
      double threshold2 = 0.5;
      double threshold3 = 0.22;
    } oscfar;
    // modified_cacfar
    struct {
      int width = 40;
      int guard = 2;
      double threshold = 0.5;
      double threshold2 = 0.5;
      double threshold3 = 0.22;
    } modified_cacfar;

    double radar_resolution = 0.0438;
    double cart_resolution = 0.25;
    double beta = 0.049;

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  NavtechExtractionModule(
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

  VTR_REGISTER_MODULE_DEC_TYPE(NavtechExtractionModule);
};

}  // namespace radar
}  // namespace vtr