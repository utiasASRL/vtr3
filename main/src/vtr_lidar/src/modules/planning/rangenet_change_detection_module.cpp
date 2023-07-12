// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file rangenet_change_detection_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/planning/rangenet_change_detection_module.hpp>
#include "vtr_lidar/filters/range_image.hpp"
#include "vtr_common/timing/stopwatch.hpp"


namespace vtr {
namespace lidar {

namespace {
  void velodyneCart2Pol(pcl::PointCloud<PointWithInfo> &point_cloud) {
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto &p = point_cloud[i];
    auto &pm1 = i > 0 ? point_cloud[i - 1] : point_cloud[i];

    p.rho = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    p.theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
    p.phi = atan2(p.y, p.x); // + M_PI / 2;

    if (i > 0 && (p.phi - pm1.phi) > 1.5 * M_PI)
      p.phi -= 2 * M_PI;
    else if (i > 0 && (p.phi - pm1.phi) < -1.5 * M_PI)
      p.phi += 2 * M_PI;
  }
}
}

using namespace tactic;

auto RangeChangeNetModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<RangeChangeNetModule::Config>();
  auto base_config = std::static_pointer_cast<TorchModule::Config>(config);
  *base_config =  *nn::TorchModule::Config::fromROS(node, param_prefix);

  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);

  config->img_width = node->declare_parameter<int>(param_prefix + ".img_width", config->img_width);
  config->img_height = node->declare_parameter<int>(param_prefix + ".img_height", config->img_height);

  // clang-format on
  return config;
}


void RangeChangeNetModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);


  auto nn_point_cloud = *qdata.nn_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;  
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &T_v_m_loc = *qdata.T_v_m_loc;



  if(!qdata.submap_loc.valid()) {
    CLOG(WARNING, "lidar.range_change") << "Range image requires a map to work";
    return;
  }

  if(!pub_init){
    mask_pub_ = qdata.node->create_publisher<Image>("detection_mask", 5);
    live_range_pub_ = qdata.node->create_publisher<Image>("live_range_image", 5);
    map_range_pub_ = qdata.node->create_publisher<Image>("map_range_image", 5);
    diffpcd_pub_ = qdata.node->create_publisher<PointCloudMsg>("detection_cloud", 5);
    mappcd_pub_ = qdata.node->create_publisher<PointCloudMsg>("transformed_map", 5);
  }

  auto& sub_map= *qdata.submap_loc;
  auto map_point_cloud = sub_map.point_cloud();
  auto points_mat = map_point_cloud.getMatrixXfMap(4, PointWithInfo::size(), PointWithInfo::cartesian_offset());

  CLOG(DEBUG, "lidar.range") << "Before: (" << map_point_cloud[10].x << ", " << map_point_cloud[10].y << ", "<< map_point_cloud[10].z <<")";
  const auto T_s_m = (T_s_r * T_r_v_loc * T_v_m_loc).matrix();
  points_mat = T_s_m.cast<float>() * points_mat;
  velodyneCart2Pol(map_point_cloud);


  CLOG(DEBUG, "lidar.range") << "After: (" << map_point_cloud[10].x << ", " << map_point_cloud[10].y << ", "<< map_point_cloud[10].z <<")";


  RangeImageParams image_params;
  image_params.fov_up = 3 * M_PI / 180;
  image_params.fov_down = -25 * M_PI / 180;
  image_params.crop_range = 8.0;

  RowMatrixXf scan_image = Eigen::MatrixXf::Constant(64, 1024, -1);
  RowMatrixXf mask_image = Eigen::MatrixXf::Zero(64, 1024);
  Eigen::MatrixXi scan_idxs = Eigen::MatrixXi::Constant(64, 1024, -1);
  RowMatrixXf map_image = Eigen::MatrixXf::Constant(64, 1024, -1);

  common::timing::Stopwatch<boost::chrono::thread_clock> timer;
  timer.start();
  generate_range_image(nn_point_cloud, scan_image, scan_idxs, image_params);
  generate_range_image(map_point_cloud, map_image, image_params);
  timer.stop();
  CLOG(DEBUG, "lidar.range") << "Range image creation takes " << timer;

  using namespace torch::indexing;

  auto scan_tensor = torch::from_blob(scan_image.data(), {64, 1024});
  auto map_tensor = torch::from_blob(map_image.data(), {64, 1024});
  auto input = at::unsqueeze(at::stack({scan_tensor, map_tensor}), 0);

  timer.reset();
  auto tensor = evaluateModel(input, {1, 2, 64, 1024});
  auto mask = at::squeeze(at::argmax(tensor, 1), 0).to(at::kFloat);
  timer.stop();
  CLOG(DEBUG, "lidar.range") << "Running inference takes " << timer;


  torch::from_blob(mask_image.data(), {64, 1024}) = mask;

  unproject_range_image(nn_point_cloud, mask_image, scan_idxs);

  /// publish the transformed pointcloud
  if (config_->visualize) {
    PointCloudMsg filter_msg;
    pcl::toROSMsg(nn_point_cloud, filter_msg);
    filter_msg.header.frame_id = "odo vertex frame";
    // pointcloud_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    diffpcd_pub_->publish(filter_msg);

    PointCloudMsg map_msg;
    pcl::toROSMsg(map_point_cloud, map_msg);
    map_msg.header.frame_id = "odo vertex frame";
    // pointcloud_msg.header.stamp = rclcpp::Time(*qdata.stamp);
    mappcd_pub_->publish(map_msg);


    mask_pub_->publish(range_to_image(mask_image));
    live_range_pub_->publish(range_to_image(scan_image));
    map_range_pub_->publish(range_to_image(map_image));

  }
                            
}

}  // namespace nn
}  // namespace vtr