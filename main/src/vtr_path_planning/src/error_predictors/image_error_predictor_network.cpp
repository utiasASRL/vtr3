// Copyright 2026, Autonomous Space Robotics Lab (ASRL)
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
 * \file image_error_predictor_network.cpp
 * \author Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */

#include <torch/script.h>
#include <torch/cuda.h>
#include <vtr_torch/types.hpp>

#include "vtr_path_planning/error_predictors/image_error_predictor_network.hpp"

#include <stdexcept>

namespace vtr {
namespace path_planning {

struct ImageErrorPredictorNetwork::Impl {
  Module model;
  torch::Device device{torch::kCPU};
};

namespace {

torch::Tensor imageToTensor(const ImageMsg& msg, torch::Device device) {
  const int h = static_cast<int>(msg.height);
  const int w = static_cast<int>(msg.width);
  const int c = 3;

  auto opts = torch::TensorOptions().dtype(torch::kUInt8);
  torch::Tensor hwc =
      torch::from_blob(const_cast<uint8_t*>(msg.data.data()), {h, w, c}, opts)
          .clone();

  return hwc.permute({2, 0, 1}).to(torch::kFloat32).div(255.0f).to(device);
}

torch::Tensor imuToTensor(const ImuMsg& msg, torch::Device device) {
  std::array<float, 6> vals{
      static_cast<float>(msg.linear_acceleration.x),
      static_cast<float>(msg.linear_acceleration.y),
      static_cast<float>(msg.linear_acceleration.z),
      static_cast<float>(msg.angular_velocity.x),
      static_cast<float>(msg.angular_velocity.y),
      static_cast<float>(msg.angular_velocity.z),
  };
  return torch::tensor(torch::ArrayRef<float>(vals.data(), vals.size())).to(device);
}

}  // namespace

ImageErrorPredictorNetwork::ImageErrorPredictorNetwork(
    std::string model_path, bool use_gpu)
    : impl_(std::make_unique<Impl>()) {
  if (use_gpu) {
    if (torch::cuda::is_available()) {
      impl_->device = torch::kCUDA;
    } else {
      CLOG(WARNING, "path_planning")
          << "ImageErrorPredictorNetwork: CUDA requested but not available, falling back to CPU.";
    }
  }

  try {
    impl_->model = torch::jit::load(model_path);
  } catch (const c10::Error& e) {
    throw std::runtime_error(
        "ImageErrorPredictorNetwork: failed to load model from " + model_path +
        ": " + e.what());
  }

  impl_->model.to(impl_->device);
  impl_->model.eval();
  CLOG(INFO, "path_planning")
      << "ImageErrorPredictorNetwork: using device " << impl_->device;
}

ImageErrorPredictorNetwork::~ImageErrorPredictorNetwork() = default;

void ImageErrorPredictorNetwork::setInputs(const ImageMsg& sig_img_msg,
                                            const ImageMsg& dpt_img_msg,
                                            const ImuMsg& imu_msg) {
  cur_sig_img_msg_ = std::make_shared<ImageMsg>(sig_img_msg);
  cur_dpt_img_msg_ = std::make_shared<ImageMsg>(dpt_img_msg);
  cur_imu_msg_     = std::make_shared<ImuMsg>(imu_msg);
}

void ImageErrorPredictorNetwork::predictError(
    const RobotState& /* robot_state */, const tactic::Timestamp& /* curr_time */) {
  if (!cur_sig_img_msg_ || !cur_dpt_img_msg_ || !cur_imu_msg_) {
    CLOG(WARNING, "path_planning")
        << "ImageErrorPredictorNetwork::predictError called before inputs set.";
    return;
  }

  torch::NoGradGuard no_grad;

  torch::Tensor sig_tensor = imageToTensor(*cur_sig_img_msg_, impl_->device).unsqueeze(0);
  torch::Tensor dpt_tensor = imageToTensor(*cur_dpt_img_msg_, impl_->device).unsqueeze(0);
  torch::Tensor imu_tensor = imuToTensor(*cur_imu_msg_, impl_->device).unsqueeze(0);

  std::vector<torch::jit::IValue> inputs{sig_tensor, dpt_tensor, imu_tensor};

  torch::Tensor output;
  try {
    output = impl_->model.forward(inputs).toTensor().cpu();
  } catch (const c10::Error& e) {
    CLOG(ERROR, "path_planning")
        << "ImageErrorPredictorNetwork: model forward pass failed: " << e.what();
    return;
  }

  auto flat = output.squeeze().contiguous();
  const int64_t n = flat.numel();
  Eigen::VectorXd error(n);
  auto* data = flat.data_ptr<float>();
  for (int64_t i = 0; i < n; ++i) error(i) = static_cast<double>(data[i]);

  CLOG(DEBUG, "path_planning")
      << "ImageErrorPredictorNetwork: predicted error vector of size " << n;
}

}  // namespace path_planning
}  // namespace vtr
