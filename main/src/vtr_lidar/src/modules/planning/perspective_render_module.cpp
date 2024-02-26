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
 * \file perspective_render_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/planning/perspective_render_module.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgproc.hpp"


namespace vtr {
namespace lidar {

using namespace tactic;

auto PerspectiveRenderModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off

  auto& params = config->perspective_params;

  params.width = node->declare_parameter<int>(param_prefix + ".img_width", 0);
  params.height = node->declare_parameter<int>(param_prefix + ".img_height", 0);
  params.h_fov = node->declare_parameter<double>(param_prefix + ".h_fov", M_PI/2);
  params.v_fov = node->declare_parameter<double>(param_prefix + ".v_fov", M_PI/4);
  
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);
  // clang-format on
  return config;
}

PerspectiveRenderModule::PerspectiveRenderModule(
    const Config::ConstPtr &config,
    const std::shared_ptr<tactic::ModuleFactory> &module_factory,
    const std::string &name)
    : tactic::BaseModule{module_factory, name}, config_(config) {}

void PerspectiveRenderModule::run_(QueryCache &qdata0, OutputCache &output0,
                                   const Graph::Ptr &graph,
                                   const TaskExecutor::Ptr &executor) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  if (!publisher_initialized_) {
    img_pub_ = qdata.node->create_publisher<ImageMsg>("live_range_coloured", 5);
    publisher_initialized_ = true;
  }

  cv::Mat index_img;
  index_img.create(config_->perspective_params.height, config_->perspective_params.width, CV_32S);

  cv::Mat hsv_img;
  hsv_img.create({config_->perspective_params.height, config_->perspective_params.width, 3}, CV_8UC3);

  generate_depth_image(*qdata.raw_point_cloud, hsv_img, index_img, config_->perspective_params);

  cv::Mat rgb_img;

  cv::cvtColor(hsv_img, rgb_img, cv::COLOR_HSV2RGB);

  if (config_->visualize) {
    ImageMsg::Ptr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb_img).toImageMsg();
    img_pub_->publish(msg);
  }
}

}  // namespace lidar
}  // namespace vtr