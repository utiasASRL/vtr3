// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
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
 * \file calibration_module.cpp
 * \brief CalibrationModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
// #include <vtr_vision/features/extractor/feature_extractor_factory.hpp>
// #include <vtr_vision/image_conversions.hpp>
#include <vtr_vision/modules/preprocessing/calibration_module.hpp>
#include <vtr_vision/types.hpp>
#include <cv_bridge/cv_bridge.h>

namespace vtr {
namespace vision {

using namespace tactic;


auto CalibrationModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) 
    -> ConstPtr {

  auto config = std::make_shared<Config>();

  auto distortion = node->declare_parameter<std::vector<double>>(param_prefix + ".distortion");
  auto intrinsic = node->declare_parameter<std::vector<double>>(param_prefix + ".intrinsic");
  auto extrinsic = node->declare_parameter<std::vector<double>>(param_prefix + ".extrinsic");
  config->distortion=Eigen::Map<CameraDistortion>(distortion.data());
  config->intrinsic=Eigen::Map<CameraIntrinsic>(intrinsic.data());
  // config->extrinsic=Eigen::Map<Transform>(extrinsic.data());

  return config;
}

void CalibrationModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);
  RigImages rig_images;
  ChannelImages channel_images;
  RigCalibration rig_calibration;


  CameraDistortions camera_distortions {config_->distortion};
  CameraIntrinsics camera_intrinsics {config_->intrinsic};
  // Transforms extrinsics {config_->extrinsic};

  rig_calibration.distortions=camera_distortions;
  rig_calibration.intrinsics=camera_intrinsics;
  // rig_calibration.extrinsics=extrinsics;

  Image left_im =  {};
  left_im.stamp = *qdata.stamp;
  left_im.name = "left";
  left_im.data = cv_bridge::toCvShare(qdata.left_image.ptr(), qdata.left_image->encoding)->image;
  // {qdata.stamp,                   "left",      cv_bridge::toCvShare(qdata.left_image.ptr(), qdata.left_image->encoding)};
  // Image right_im = {qdata.stamp, "right", cv_bridge::toCvShare(qdata.right_image.ptr(), qdata.right_image->encoding)};
  
  // channel_images



}


}  // namespace vision
}  // namespace vtr