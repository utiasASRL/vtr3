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


namespace vtr {
namespace lidar {

using namespace tactic;

auto RangeChangeNetModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<RangeChangeNetModule::Config>();
  auto base_config = std::static_pointer_cast<TorchModule::Config>(config);
  *base_config =  *nn::TorchModule::Config::fromROS(node, param_prefix);
  // clang-format off
  // clang-format on
  return config;
}


void RangeChangeNetModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);


  auto& nn_point_cloud = *qdata.nn_point_cloud;

  if(!qdata.submap_loc.valid()) {
    CLOG(WARNING, "lidar.range_change") << "Range image requires a map to work";
    return;
  }

  if(!pub_init){
    mask_pub_ = qdata.node->create_publisher<Image>("detection_mask", 5);
  }
  auto& sub_map= *qdata.submap_loc;
  auto& map_point_cloud = sub_map.point_cloud();

  RangeImageParams image_params;
  image_params.fov_up = 3 * M_PI / 180;
  image_params.fov_down = -25 * M_PI / 180;

  Eigen::MatrixXf scan_image = Eigen::MatrixXf::Zero(64, 1024);
  Eigen::MatrixXf map_image = Eigen::MatrixXf::Zero(64, 1024);

  generate_range_image(nn_point_cloud, scan_image, image_params);
  generate_range_image(map_point_cloud, map_image, image_params);

  auto scan_tensor = torch::from_blob(scan_image.data(), {64, 1024});
  auto map_tensor = torch::from_blob(map_image.data(), {64, 1024});
  auto input = at::unsqueeze(at::stack({scan_tensor, map_tensor}), 0);

  auto tensor = evaluateModel(input, {1, 2, 64, 1024});
  auto mask = at::squeeze(at::argmax(tensor, 1), 0).to(torch::kUInt8);

  Image mask_im;
  mask_im.width = 1024;
  mask_im.height = 64;
  mask_im.encoding = "mono8";
  mask_im.data = *mask.data_ptr<uint8_t>();
  mask_im.step = 1024;


  mask_pub_->publish(mask_im);
                            
}

}  // namespace nn
}  // namespace vtr