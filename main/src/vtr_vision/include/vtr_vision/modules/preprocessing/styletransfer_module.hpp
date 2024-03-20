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
 * \file style_transfer.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"
#include <vtr_torch/modules/torch_module.hpp>
#include "vtr_vision/cache.hpp"

#include <vector>
#include <list>
#include <vtr_vision/types.hpp>

#include "sensor_msgs/msg/image.hpp"



namespace vtr {
namespace vision {

// using ImageMsg = sensor_msgs::msg::Image;
// using PointCloudMsg = sensor_msgs::msg::PointCloud2;


/** \brief Load and store Torch Models */
class StyleTransferModule : public nn::TorchModule {
 public:
  PTR_TYPEDEFS(StyleTransferModule);    

  /** \brief Static module identifier. */
  static constexpr auto static_name = "vision.styletransfer";

  /** \brief Config parameters. */
  struct Config : public nn::TorchModule::Config {
    PTR_TYPEDEFS(Config);

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);

  };

  StyleTransferModule(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : nn::TorchModule{config, module_factory, name}, config_(config) {}


 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  // bool pub_init_ = false;

  // rclcpp::Publisher<ImageMsg>::SharedPtr mask_pub_;
  // rclcpp::Publisher<ImageMsg>::SharedPtr diff_pub_;
  // rclcpp::Publisher<ImageMsg>::SharedPtr live_img_pub_;
  // rclcpp::Publisher<ImageMsg>::SharedPtr map_img_pub_;
  // rclcpp::Publisher<PointCloudMsg>::SharedPtr filtered_pub_;




  VTR_REGISTER_MODULE_DEC_TYPE(StyleTransferModule);

};

}  // namespace vision
}  // namespace vtr