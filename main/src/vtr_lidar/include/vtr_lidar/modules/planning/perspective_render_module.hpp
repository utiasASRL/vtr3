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
 * \file perspective_rnder_module.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"
#include "vtr_lidar/filters/perspective_image.hpp"

#include "sensor_msgs/msg/image.hpp"

namespace vtr {
namespace lidar {

class PerspectiveRenderModule : public tactic::BaseModule {
 public:
  PTR_TYPEDEFS(PerspectiveRenderModule);
  using ImageMsg = sensor_msgs::msg::Image;

  static constexpr auto static_name = "lidar.render_perspective";

  /** \brief Collection of config parameters */
  struct Config : public BaseModule::Config {
    PTR_TYPEDEFS(Config);

    PerspectiveImageParams perspective_params;

    //
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  PerspectiveRenderModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name);

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;

  rclcpp::Publisher<ImageMsg>::SharedPtr live_img_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr map_img_pub_;


  VTR_REGISTER_MODULE_DEC_TYPE(PerspectiveRenderModule);

};

}  // namespace lidar
}  // namespace vtr
