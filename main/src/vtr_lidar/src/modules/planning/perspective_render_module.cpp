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

#include "cv_bridge/cv_bridge.h"
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
  params.max_range = node->declare_parameter<double>(param_prefix + ".max_range", params.max_range);
  params.min_range = node->declare_parameter<double>(param_prefix + ".min_range", params.min_range);
  
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
  auto raw_point_cloud = *qdata.raw_point_cloud;

  if(!qdata.submap_loc.valid()) {
    CLOG(WARNING, "lidar.range_change") << "Range image requires a map to work";
    return;
  }

  const auto &T_s_r = *qdata.T_s_r;  
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &T_v_m_loc = *qdata.T_v_m_loc;
  const auto &sid_loc = *qdata.sid_loc;

  CLOG(DEBUG, "lidar.perspective") << "Hello world";


  if (!publisher_initialized_) {
    live_img_pub_ = qdata.node->create_publisher<ImageMsg>("live_range_coloured", 5);
    map_img_pub_ = qdata.node->create_publisher<ImageMsg>("map_range_coloured", 5);
    publisher_initialized_ = true;
    CLOG(DEBUG, "lidar.perspective") << "Creating publisher";
  }

  



  auto& sub_map= *qdata.submap_loc;
  auto map_point_cloud = sub_map.point_cloud();
  auto map_points_mat = map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());

  CLOG(DEBUG, "lidar.range") << "Before: (" << map_point_cloud[10].x << ", " << map_point_cloud[10].y << ", "<< map_point_cloud[10].z <<")";
  
  const auto T_s_m = (T_s_r * T_r_v_loc * T_v_m_loc).matrix();


  auto live_points_mat = raw_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());

  Eigen::Matrix4f T_c_s;

  T_c_s << 0, -1, 0, 0,
           0, 0, 1, 0,
           -1, 0, 0, 0,
           0, 0, 0, 1;

  live_points_mat = T_c_s * live_points_mat;
  map_points_mat = (T_c_s * T_s_m.cast<float>()) * map_points_mat;

  

  cv::Mat live_index_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_32S);
  cv::Mat live_hsv_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat live_rgb_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);

  generate_depth_image(raw_point_cloud, live_hsv_img, live_index_img, config_->perspective_params);

  cv::cvtColor(live_hsv_img, live_rgb_img, cv::COLOR_HSV2RGB);

  if (config_->visualize) {
    cv_bridge::CvImage live_cv_rgb_img;
    live_cv_rgb_img.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    live_cv_rgb_img.encoding = "rgb8";
    live_cv_rgb_img.image = live_rgb_img;
    live_img_pub_->publish(*live_cv_rgb_img.toImageMsg());
  }

  cv::Mat map_index_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_32S);
  cv::Mat map_hsv_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
  cv::Mat map_rgb_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);

  generate_depth_image(map_point_cloud, map_hsv_img, map_index_img, config_->perspective_params);

  cv::cvtColor(map_hsv_img, map_rgb_img, cv::COLOR_HSV2RGB);

  if (config_->visualize) {
    cv_bridge::CvImage map_cv_rgb_img;
    map_cv_rgb_img.header.frame_id = "lidar";
    //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
    map_cv_rgb_img.encoding = "rgb8";
    map_cv_rgb_img.image = map_rgb_img;
    map_img_pub_->publish(*map_cv_rgb_img.toImageMsg());
  }
}

}  // namespace lidar
}  // namespace vtr