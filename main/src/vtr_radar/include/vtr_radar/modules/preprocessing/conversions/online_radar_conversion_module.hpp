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
 * \file online_radar_conversion_module.hpp
 * \author Sam Qiao, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_radar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace radar {

/** \brief Extracts keypoints from Navtech radar scans. */
class OnlineRadarConversionModule : public tactic::BaseModule {
 public:
  using ImageMsg = sensor_msgs::msg::Image;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "radar.online_converter";

    /** \brief Config parameters. */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    double maxr = 100;
    double radar_resolution = 0.0438;
    double cart_resolution = 0.25;
    int encoder_bin_size = 16000;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  OnlineRadarConversionModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;


  VTR_REGISTER_MODULE_DEC_TYPE(OnlineRadarConversionModule);
};

}  // namespace radar
}  // namespace vtr