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
 * \file change_detection_outputs.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/modules/testing/change_detection_outputs.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>
#include "sensor_msgs/msg/image.hpp"


namespace vtr {
namespace lidar {

using namespace tactic;

auto CDTestModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                       const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->save_point_cloud = node->declare_parameter<bool>(param_prefix + ".save_point_cloud", config->save_point_cloud);
  config->suffix = node->declare_parameter<std::string>(param_prefix + ".suffix", config->suffix);

  auto& params = config->perspective_params;

  params.width = node->declare_parameter<int>(param_prefix + ".img_width", 256);
  params.height = node->declare_parameter<int>(param_prefix + ".img_height", 128);
  params.h_fov = node->declare_parameter<double>(param_prefix + ".h_fov", M_PI/2);
  params.v_fov = node->declare_parameter<double>(param_prefix + ".v_fov", M_PI/4);
  params.max_range = node->declare_parameter<double>(param_prefix + ".max_range", params.max_range);
  params.min_range = node->declare_parameter<double>(param_prefix + ".min_range", params.min_range);
  // clang-format on
  return config;
}

void CDTestModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &graph, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);


  if (*qdata.vertex_test_result == VertexTestResult::CREATE_VERTEX) {

    auto vertex = graph->at(*qdata.vid_odo);

    const auto &T_s_r = *qdata.T_s_r;  
    const auto &T_r_v_loc = *qdata.T_r_v_loc;

    Eigen::Matrix4d T_c_s_temp;

    T_c_s_temp << 0, -1, 0, 0,
            0, 0, 1, 0,
            -1, 0, 0, 0,
            0, 0, 0, 1;

    const tactic::EdgeTransform T_c_s {T_c_s_temp};
    const tactic::EdgeTransform T_c_v_loc = T_c_s * T_s_r * T_r_v_loc;

    //Save the changed point cloud

    if (config_->save_point_cloud) {
      auto raw_scan_odo = std::make_shared<PointScan<PointWithInfo>>();
      raw_scan_odo->point_cloud() = *qdata.changed_points;
      raw_scan_odo->T_vertex_this() = *qdata.T_r_v_loc;
      raw_scan_odo->vertex_id() = *qdata.vid_odo;
      //
      using PointScanLM = storage::LockableMessage<PointScan<PointWithInfo>>;
      auto raw_scan_odo_msg =
          std::make_shared<PointScanLM>(raw_scan_odo, *qdata.stamp);
      vertex->insert<PointScan<PointWithInfo>>(
          "detected_points_" + config_->suffix, "vtr_lidar_msgs/msg/PointScan", raw_scan_odo_msg);
      CLOG(DEBUG, "lidar.pipeline") << "Saved raw pointcloud to vertex" << vertex;
    }

    auto changed_points_copy = pcl::PointCloud<PointWithInfo>(*qdata.changed_points);

    auto live_points_mat = changed_points_copy.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());
    live_points_mat = T_c_v_loc.matrix().cast<float>() * live_points_mat;

    cv::Mat changed_idx_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_32S);
    cv::Mat changed_hsv_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC3);
    cv::Mat changed_thresh_img = cv::Mat::zeros(config_->perspective_params.height, config_->perspective_params.width, CV_8UC1);

    generate_depth_image(changed_points_copy, changed_hsv_img, changed_idx_img, config_->perspective_params);

    cv::Mat hsv_chans[3];   //destination array
    cv::split(changed_hsv_img, hsv_chans);//split source  


    cv_bridge::CvImage live_mask_img_msg;
    live_mask_img_msg.header.frame_id = "lidar";
    live_mask_img_msg.encoding = "mono8";
    live_mask_img_msg.image = hsv_chans[1];

    using Image_LockMsg = storage::LockableMessage<sensor_msgs::msg::Image>;
    auto locked_image_msg =
            std::make_shared<Image_LockMsg>(live_mask_img_msg.toImageMsg(), *qdata.stamp);
    vertex->insert<sensor_msgs::msg::Image>("detection_mask_" + config_->suffix, "sensor_msgs/msg/Image", locked_image_msg);

  }

  qdata.changed_points.clear();
}

}  // namespace lidar
}  // namespace vtr