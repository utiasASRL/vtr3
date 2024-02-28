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
 * \file segmentanything_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/modules/planning/segmentanything_module.hpp>



namespace vtr {
namespace lidar {

using namespace tactic;

auto SegmentAnythingModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<SegmentAnythingModule::Config>();
  auto base_config = std::static_pointer_cast<TorchModule::Config>(config);
  *base_config =  *nn::TorchModule::Config::fromROS(node, param_prefix);

  // clang-format off
  config->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config->visualize);

  // clang-format on
  return config;
}


void SegmentAnythingModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &graph, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);


  auto nn_point_cloud = *qdata.nn_point_cloud;
  const auto &T_s_r = *qdata.T_s_r;  
  const auto &T_r_v_loc = *qdata.T_r_v_loc;
  const auto &T_v_m_loc = *qdata.T_v_m_loc;
  const auto &sid_loc = *qdata.sid_loc;
  
  CLOG(DEBUG, "running SAM");


  if(!qdata.rendered_images.valid()) {
    CLOG(WARNING, "lidar.perspective") << "Rendered perspective images required a map to work!";
    return;
  }

  if(!pub_init_){
    mask_pub_ = qdata.node->create_publisher<ImageMsg>("detection_mask", 5);
    pub_init_ = true;
  }

  auto& [live_img, map_img]= *qdata.rendered_images;

  CLOG(DEBUG, "lidar.perspective") << "Received images! ";


  using namespace torch::indexing;


  auto input = torch::zeros({1, 3, 1024, 1024});
  input.index_put_({0, "...", Slice(0, 128), Slice(0, 256)}, torch::from_blob(live_img.ptr(), {128, 256}));

  torch::NoGradGuard no_grad;
  std::vector<torch::jit::IValue> jit_inputs;

  jit_inputs.push_back(input.to(device));

  auto output = network(jit_inputs);

  CLOG(DEBUG, "lidar.perspective") << "Ran model!";

  auto mask = output.toGenericDict().at("masks");


  
  /// publish the transformed pointcloud
  if (config_->visualize) {


    // mask_pub_->publish(range_to_image(mask_image));

  }
                            
}

}  // namespace nn
}  // namespace vtr