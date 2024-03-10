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
  cv::Mat live_image = qdata.rig_images->front().channels.at(0).cameras.front().data;

  // cv::resize(live_rgb_img, live_image, cv::Size(1024, 512), cv::INTER_LINEAR);

  if (!live_image.isContinuous()){
    live_image = live_image.clone();
    CLOG(WARNING, "stereo.learned_features") << "Live image was not contiguous in memory!";
  }

  torch::Tensor live_tensor = torch::from_blob(live_image.data, {live_image.rows, live_image.cols, live_image.channels()}, torch::kByte).to(torch::kFloat32).permute({2, 0, 1}).contiguous();
  live_tensor = live_tensor.unsqueeze(0) / 255;

  using namespace torch::indexing;

  // int data[] = { 512, 256,
  //                256, 256,
  //                768, 256 };

  // std::vector<torch::Tensor> prompts;

  // auto idxs_a = topk_idx.accessor<long, 1>();

  // for (size_t i = 0; i < idxs_a.size(0); i++) {
  //   const long& idx = idxs_a[i];
  //   int prompt[] = {idx % diff.size(1), idx / diff.size(1)};
  //   CLOG(DEBUG, "lidar.perspective") << "Diff Tensor prompts (" << prompt[0] << ", " << prompt[1] << ")";

  //   if (map_tensor.index({0, idx % diff.size(1), idx / diff.size(1)}) > 0)
  //     prompts.push_back(torch::from_blob(prompt, {2}, torch::kInt).to(device));  
  //   else
  //     CLOG(DEBUG, "lidar.perspective") << "Prompt point on an empty map pixel. Try again."
  // }
  
  // prompts.push_back(torch::from_blob(&data[2], {2}, torch::kInt).to(device));  
  // prompts.push_back(torch::from_blob(&data[4], {2}, torch::kInt).to(device));  

  torch::NoGradGuard no_grad;

  std::vector<torch::jit::IValue> jit_inputs;

  jit_inputs.push_back(live_tensor.to(device));
  // jit_inputs.push_back(prompts);

  auto outputs = network(jit_inputs).toTensor().to("cpu");

  // auto sizes = outputs.sizes();

  // CLOG(DEBUG, "stereo.learned_features") << "Shape: [";
  // for(size_t i = 0; i < sizes.size(); ++i) {
  //     CLOG(DEBUG, "stereo.learned_features") << sizes[i];
  //     if (i < sizes.size() - 1) {
  //         CLOG(DEBUG, "stereo.learned_features") << ", ";
  //     }
  // }

  // auto clamped_outputs = torch::clamp(outputs, 0.0, 255.0) / 255;
  auto clamped_outputs = torch::clamp(outputs, 0.0, 255.0);

  CLOG(DEBUG, "stereo.learned_features") << "Converted image style!";

  // auto teach_masks = outputs.at("teach_masks").toTensor();
  // auto repeat_masks = outputs.at("repeat_masks").toTensor();

  auto clamped_image = clamped_outputs.squeeze().permute({1, 2, 0}).to(torch::kByte).contiguous();
  // auto disp_outputs = clamped_image.clone().contiguous() * 255;
  auto disp_outputs = clamped_image.clone().contiguous();


  cv::Mat daytime{disp_outputs.size(0), disp_outputs.size(1), CV_8UC3, disp_outputs.data_ptr<uint8_t>()};

  CLOG(DEBUG, "stereo.learned_features") << daytime.size() << "size";
  CLOG(DEBUG, "stereo.learned_features") << daytime.rows << "rows";
  CLOG(DEBUG, "stereo.learned_features") << daytime.cols << "cols";
  CLOG(DEBUG, "stereo.learned_features") << daytime.channels() << "channels";

  // auto display_image = vtr::vision::visualize::setupDisplayImage(daytime);
  // cv::Mat display_image = daytime.clone();

  if (!daytime.isContinuous()){
    CLOG(WARNING, "stereo.learned_features") << "Day image was not contiguous in memory!";
    daytime = daytime.clone();
  }


  {
    CLOG(DEBUG, "stereo.learned_features") << "mutex";
    std::lock_guard<std::mutex> lock(*qdata.vis_mutex);
    CLOG(DEBUG, "stereo.learned_features") << "window";
    cv::namedWindow("daytime", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    CLOG(DEBUG, "stereo.learned_features") << "show";
    cv::imshow("daytime", daytime);
    CLOG(DEBUG, "stereo.learned_features") << "success";
  }


  // qdata.rig_images->front().channels.at(0).cameras.front().data = daytime;








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