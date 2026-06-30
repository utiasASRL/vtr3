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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

#include <stdexcept>
#include "vtr_common/timing/stopwatch.hpp"

namespace vtr {
namespace path_planning {

struct ImageErrorPredictorNetwork::Impl {
  Module model;
  torch::Device device{torch::kCPU};
};

namespace {

torch::Tensor imageToTensor(const ImageMsg& msg, torch::Device device) {
  const int64_t h = static_cast<int64_t>(msg.height);
  const int64_t w = static_cast<int64_t>(msg.width);

  torch::Tensor hw;
  if (msg.encoding == "mono16" || msg.encoding == "16UC1") {
    // 16-bit: reinterpret the byte buffer as uint16
    auto opts = torch::TensorOptions().dtype(torch::kInt16);
    hw = torch::from_blob(const_cast<uint8_t*>(msg.data.data()), {h, w}, opts)
             .clone()
             .to(torch::kFloat32)
             .view_as(torch::zeros({h, w}, torch::kFloat32));
    hw = torch::from_blob(const_cast<uint8_t*>(msg.data.data()), {h, w},
                          torch::TensorOptions().dtype(torch::kInt16))
             .clone()
             .to(torch::kInt32)
             .bitwise_and(0xFFFF)
             .to(torch::kFloat32);
  } else {
    // 8-bit default
    hw = torch::from_blob(const_cast<uint8_t*>(msg.data.data()), {h, w},
                          torch::TensorOptions().dtype(torch::kUInt8))
             .clone()
             .to(torch::kFloat32);
  }

  // Per-image min-max stretch → [0, 1], matching cv2.NORM_MINMAX + /255
  const float mn = hw.min().item<float>();
  const float mx = hw.max().item<float>();
  if (mx > mn) {
    hw = (hw - mn) / (mx - mn);
  } else {
    hw = torch::zeros_like(hw);
  }

  // Resize to the training resolution (64×512) if needed
  constexpr int64_t kTrainH = 64;
  constexpr int64_t kTrainW = 512;
  hw = hw.unsqueeze(0).unsqueeze(0);  
  if (h != kTrainH || w != kTrainW) {
    hw = torch::nn::functional::interpolate(
        hw,
        torch::nn::functional::InterpolateFuncOptions()
            .size(std::vector<int64_t>{kTrainH, kTrainW})
            .mode(torch::kBilinear)
            .align_corners(false));
  }
  return hw.squeeze(0).to(device);  
}

torch::Tensor imuToGravityTensor(const ImuMsg& msg, torch::Device device) {
  const double ax = msg.linear_acceleration.x;
  const double ay = msg.linear_acceleration.y;
  const double az = msg.linear_acceleration.z;

  tf2::Quaternion q;
  tf2::fromMsg(msg.orientation, q);
  tf2::Matrix3x3 R(q);

  const double wx = R[0][0]*ax + R[0][1]*ay + R[0][2]*az;
  const double wy = R[1][0]*ax + R[1][1]*ay + R[1][2]*az;
  const double wz = R[2][0]*ax + R[2][1]*ay + R[2][2]*az;

  std::array<float, 3> grav{
      static_cast<float>(wx),
      static_cast<float>(wy),
      static_cast<float>(wz),
  };
  return torch::tensor(torch::ArrayRef<float>(grav.data(), grav.size())).to(device);
}

// SE3 transformation → lie algebra vector [tx, ty, tz, φx, φy, φz].
torch::Tensor poseToVec(const lgmath::se3::Transformation& T) {
  const auto v = T.vec();
  std::array<float, 6> vals{
      static_cast<float>(v(0)), static_cast<float>(v(1)),
      static_cast<float>(v(2)), static_cast<float>(v(3)),
      static_cast<float>(v(4)), static_cast<float>(v(5)),
  };
  return torch::tensor(torch::ArrayRef<float>(vals.data(), vals.size()));
}

}  

ImageErrorPredictorNetwork::ImageErrorPredictorNetwork(
    rclcpp::Node::SharedPtr node,
    const std::string& sig_img_topic,
    const std::string& dpt_img_topic,
    const std::string& imu_topic,
    const std::string& model_path,
    bool use_gpu)
    : impl_(std::make_unique<Impl>()) {
  if (use_gpu) {
    if (torch::cuda::is_available()) {
      impl_->device = torch::kCUDA;
    } else {
      CLOG(WARNING, "path_planning")
          << "ImageErrorPredictorNetwork: CUDA requested but not available, "
             "falling back to CPU.";
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

  sig_img_sub_ = node->create_subscription<ImageMsg>(
      sig_img_topic, rclcpp::QoS(1),
      [this](const ImageMsg::SharedPtr msg) { cur_sig_img_msg_ = msg; });
  dpt_img_sub_ = node->create_subscription<ImageMsg>(
      dpt_img_topic, rclcpp::QoS(1),
      [this](const ImageMsg::SharedPtr msg) { cur_dpt_img_msg_ = msg; });
  imu_sub_ = node->create_subscription<ImuMsg>(
      imu_topic, rclcpp::QoS(1),
      [this](const ImuMsg::SharedPtr msg) { cur_imu_msg_ = msg; });

  CLOG(INFO, "path_planning")
      << "ImageErrorPredictorNetwork: loaded model from " << model_path
      << ", device = " << impl_->device;
}

ImageErrorPredictorNetwork::~ImageErrorPredictorNetwork() = default;


void ImageErrorPredictorNetwork::predictError(
    const RobotState& robot_state,
    const tactic::Timestamp&,
    std::vector<lgmath::se3::Transformation>& reference_poses) {

  if (!cur_sig_img_msg_ || !cur_dpt_img_msg_ || !cur_imu_msg_) {
    CLOG(WARNING, "path_planning")
        << "ImageErrorPredictorNetwork::predictError called before inputs set.";
    return;
  }
  if (reference_poses.empty()) {
    CLOG(WARNING, "path_planning")
        << "ImageErrorPredictorNetwork::predictError: reference_poses is empty.";
    return;
  }
  using Stopwatch = common::timing::Stopwatch<>;
  std::vector<std::unique_ptr<Stopwatch>> timer;
  std::vector<std::string> clock_str;
  clock_str.push_back("ImageErrorPredictorNetwork::predictError: ");
  timer.push_back(std::make_unique<Stopwatch>());
  clock_str.push_back("ImageErrorPredictorNetwork::input_setup: ");
  timer.push_back(std::make_unique<Stopwatch>());
  clock_str.push_back("ImageError:PredictorNetwork::forward_pass: ");
  timer.push_back(std::make_unique<Stopwatch>());
  clock_str.push_back("ImageError:PredictorNetwork::post_processing: ");
  timer.push_back(std::make_unique<Stopwatch>());

  timer[0]->start();
  timer[1]->start();
  auto& chain = robot_state.chain.ptr();

  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);

  torch::NoGradGuard no_grad;

  // Image input: (1, 2, H, W) signal and range stacked on channel dim
  torch::Tensor sig = imageToTensor(*cur_sig_img_msg_, impl_->device);  // (1,H,W)
  torch::Tensor dpt = imageToTensor(*cur_dpt_img_msg_, impl_->device);  // (1,H,W)
  torch::Tensor image = torch::cat({sig, dpt}, /*dim=*/0).unsqueeze(0); // (1,2,H,W)

  // Sequence input: (1, T, 6): reference poses from MPC in SE3
  const int64_t T = static_cast<int64_t>(reference_poses.size());
  std::vector<torch::Tensor> pose_vecs;
  pose_vecs.reserve(T);
  for (const auto& pose : reference_poses) {
    pose_vecs.push_back(poseToVec(pose));
  }

  // Stack (T, 6) then unsqueeze batch dim -> (1, T, 6)
  torch::Tensor sequence = torch::stack(pose_vecs, 0)
                               .unsqueeze(0)
                               .to(impl_->device);
  CLOG(DEBUG, "path_planning") << "ImageErrorPredictorNetwork: sequence = "
                              << sequence;

  // Dataset stores ne.T = T_r_p (robot→path), which is T_p_r.inverse().
  auto loc_res_tensor = poseToVec(T_p_r.inverse()).unsqueeze(0).to(impl_->device);  // (1,6)
  CLOG(DEBUG, "path_planning") << "ImageErrorPredictorNetwork: loc_res_tensor = "
                              << loc_res_tensor;

  // odom: body-frame velocity twist w_p_r_in_r
  const auto& odom_vel = w_p_r_in_r;
  std::array<float, 6> odom_vel_arr{
      static_cast<float>(odom_vel(0)), static_cast<float>(odom_vel(1)),
      static_cast<float>(odom_vel(2)), static_cast<float>(odom_vel(3)),
      static_cast<float>(odom_vel(4)), static_cast<float>(odom_vel(5)),
  };
  CLOG(DEBUG, "path_planning") << "ImageErrorPredictorNetwork: odom_vel_arr = "
                              << odom_vel_arr[0] << ", " << odom_vel_arr[1]
                              << ", " << odom_vel_arr[2] << ", "
                              << odom_vel_arr[3] << ", " << odom_vel_arr[4]
                              << ", " << odom_vel_arr[5];

  auto odom_vel_tensor =
      torch::tensor(torch::ArrayRef<float>(odom_vel_arr.data(), odom_vel_arr.size()))
          .unsqueeze(0).to(impl_->device);  // (1,6)

  // grav_vec: linear_acceleration rotated to world frame → (1,3).
  torch::Tensor grav_vec = imuToGravityTensor(*cur_imu_msg_, impl_->device).unsqueeze(0);  // (1,3)

  std::vector<torch::Tensor> vec_inputs_list{loc_res_tensor, odom_vel_tensor, grav_vec};
  torch::Tensor vector_inputs = torch::cat(vec_inputs_list, 1);  // (1,15)
  timer[1]->stop();

  // Run inference 
  timer[2]->start();
  std::vector<torch::jit::IValue> inputs{vector_inputs, sequence, image};
  torch::Tensor output;
  try {
    // Model output: (1, T, output_size) — typically output_size = 3 [x, y, yaw]
    output = impl_->model.forward(inputs).toTensor().cpu();
  } catch (const c10::Error& e) {
    CLOG(ERROR, "path_planning")
        << "ImageErrorPredictorNetwork: forward pass failed: " << e.what();
    return;
  }
  timer[2]->stop();

  // Apply predicted errors to reference poses 
  timer[3]->start();
  auto out = output.squeeze(0);  // (T, 3)
  const int64_t n = std::min(T, out.size(0));
  const float* data = out.data_ptr<float>();
  CLOG(DEBUG, "path_planning") << "ImageErrorPredictorNetwork: output = "
                              << output;

  for (int64_t i = 0; i < n; ++i) {
    const float dx   = data[i * 3 + 0];
    const float dy   = data[i * 3 + 1];
    const float dyaw = data[i * 3 + 2];

    // xi is the predicted tracking error (ref - actual), so we shift the
    // reference by -xi to pre-compensate: corrected_ref * T(xi) = original_ref
    Eigen::Matrix<double, 6, 1> xi;
    xi << -static_cast<double>(dx),
          -static_cast<double>(dy),
          0.0,
          0.0,
          0.0,
          -static_cast<double>(dyaw);
    CLOG(DEBUG, "path_planning") << "ImageErrorPredictorNetwork: applying correction "
                                  << xi.transpose() << " to reference pose " << i;
    reference_poses[i] = reference_poses[i] * lgmath::se3::Transformation(xi);
    CLOG(DEBUG, "path_planning") << "ImageErrorPredictorNetwork: updated reference pose: "
                                  << reference_poses[i];
  }
  timer[3]->stop();
  timer[0]->stop();
  CLOG(DEBUG, "path_planning") << "Dump timing info: ";
  for (size_t i = 0; i < clock_str.size(); i++) {
    CLOG(DEBUG, "path_planning") << clock_str[i] << timer[i]->count() << "ms";
  }

  CLOG(DEBUG, "path_planning")
      << "ImageErrorPredictorNetwork: applied corrections to " << n
      << " reference poses.";
}

}  // namespace path_planning
}  // namespace vtr
