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
#include <vtr_vision/modules/preprocessing/styletransfer_module.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include <vtr_vision/visualize.hpp>


namespace vtr {
namespace vision {

using namespace tactic;

auto StyleTransferModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                          const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<StyleTransferModule::Config>();
  auto base_config = std::static_pointer_cast<TorchModule::Config>(config);
  *base_config =  *nn::TorchModule::Config::fromROS(node, param_prefix);


  // params.width = node->declare_parameter<int>(param_prefix + ".img_width", 0);

  // clang-format on
  return config;
}


void StyleTransferModule::run_(QueryCache &qdata0, OutputCache &,
                            const Graph::Ptr &, const TaskExecutor::Ptr &) {
  
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  const auto &rig_images = *qdata.rig_images;  

  CLOG(DEBUG, "stereo.learned_features") << "Live image pulled!";
  CLOG(DEBUG, "stereo.learned_features") << qdata.rig_images->front().channels.at(0).cameras.front().data.size();

  // Extract RGB image from rig_images
  cv::Mat live_image_l = qdata.rig_images->front().channels.at(0).cameras.front().data;
  cv::Mat live_image_r = qdata.rig_images->front().channels.at(0).cameras.back().data;

  // cv::resize(live_rgb_img, live_image, cv::Size(1024, 512), cv::INTER_LINEAR);

  if (!live_image_l.isContinuous()){
    live_image_l = live_image_l.clone();
    CLOG(WARNING, "stereo.learned_features") << "Live image was not contiguous in memory!";
  }
  if (!live_image_r.isContinuous()){
    live_image_r = live_image_r.clone();
    CLOG(WARNING, "stereo.learned_features") << "Live image was not contiguous in memory!";
  }

  torch::Tensor live_tensor_l = torch::from_blob(live_image_l.data, {live_image_l.rows, live_image_l.cols, live_image_l.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1}).contiguous() /255;
  torch::Tensor live_tensor_r = torch::from_blob(live_image_r.data, {live_image_r.rows, live_image_r.cols, live_image_r.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1}).contiguous() /255;
  
  torch::Tensor live_tensor = torch::stack({live_tensor_l, live_tensor_r});

  using namespace torch::indexing;

  torch::NoGradGuard no_grad;

  std::vector<torch::jit::IValue> jit_inputs;

  jit_inputs.push_back(live_tensor.to(device));
  // jit_inputs.push_back(prompts);

  auto outputs = network(jit_inputs).toTensor().to("cpu");

  auto clamped_outputs = torch::clamp(outputs, 0.0, 255.0) / 255;

  CLOG(DEBUG, "stereo.learned_features") << "Converted image style!";

  // auto teach_masks = outputs.at("teach_masks").toTensor();

  auto scaled_clamped = clamped_outputs.clone().contiguous() * 255;

  auto disp_outputs_l = scaled_clamped[0].squeeze().permute({1, 2, 0}).to(torch::kByte).contiguous();
  cv::Mat daytime_l{disp_outputs_l.size(0), disp_outputs_l.size(1), CV_8UC3, disp_outputs_l.data_ptr<uint8_t>()};

  auto disp_outputs_r = scaled_clamped[1].squeeze().permute({1, 2, 0}).to(torch::kByte).contiguous();
  cv::Mat daytime_r{disp_outputs_r.size(0), disp_outputs_r.size(1), CV_8UC3, disp_outputs_r.data_ptr<uint8_t>()};

  CLOG(DEBUG, "stereo.learned_features") << daytime_l.size() << " left size";
  CLOG(DEBUG, "stereo.learned_features") << daytime_r.size() << " right size";

  if (!daytime_l.isContinuous()){
    CLOG(WARNING, "stereo.learned_features") << "Day image was not contiguous in memory!";
    daytime_l = daytime_l.clone();
  }


  {
    CLOG(DEBUG, "stereo.learned_features") << "mutex";
    std::lock_guard<std::mutex> lock(*qdata.vis_mutex);
    CLOG(DEBUG, "stereo.learned_features") << "window";
    cv::namedWindow("left daytime", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    CLOG(DEBUG, "stereo.learned_features") << "show";
    cv::imshow("left daytime", daytime_l);
    CLOG(DEBUG, "stereo.learned_features") << "success";

    CLOG(DEBUG, "stereo.learned_features") << "window";
    cv::namedWindow("right daytime", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    CLOG(DEBUG, "stereo.learned_features") << "show";
    cv::imshow("right daytime", daytime_r);
    CLOG(DEBUG, "stereo.learned_features") << "success";
  }


  qdata.rig_images->front().channels.at(0).cameras.front().data = daytime_l.clone();
  qdata.rig_images->front().channels.at(0).cameras.back().data = daytime_r.clone();



  // /// publish the transformed pointcloud
  // if (config_->visualize) {
  //   cv_bridge::CvImage mask_img_msg;
  //   mask_img_msg.header.frame_id = "lidar";
  //   //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
  //   mask_img_msg.encoding = "mono8";
  //   mask_img_msg.image = repeat_mask;
  //   mask_pub_->publish(*mask_img_msg.toImageMsg());

  //   diff = diff.div(diff.max()).mul(255).to(torch::kByte).contiguous();
  //   CLOG(DEBUG, "lidar.perspective") << "Max: " << diff.max() << " Min: " << diff.min();

  //   cv_bridge::CvImage diff_img_msg;
  //   diff_img_msg.header.frame_id = "lidar";
  //   //cv_rgb_img.header.stamp = qdata.stamp->header.stamp;
  //   diff_img_msg.encoding = "mono8";
  //   diff_img_msg.image = cv::Mat{diff.size(0), diff.size(1), CV_8UC1, diff.data_ptr<uint8_t>()};;
  //   diff_pub_->publish(*diff_img_msg.toImageMsg());
  // }
                            
}

}  // namespace nn
}  // namespace vtr