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
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
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

// ROS Image msg (single-channel uint8/uint16) → (1, H, W) float32 in [0,1].
torch::Tensor imageToTensor(const ImageMsg& msg, torch::Device device) {
  const int64_t h = static_cast<int64_t>(msg.height);
  const int64_t w = static_cast<int64_t>(msg.width);

  auto opts = torch::TensorOptions().dtype(torch::kUInt8);
  torch::Tensor hw =
      torch::from_blob(const_cast<uint8_t*>(msg.data.data()), {h, w}, opts)
          .clone();

  // (H, W) → (1, H, W), normalise to [0, 1]
  return hw.unsqueeze(0).to(torch::kFloat32).div(255.0f).to(device);
}

torch::Tensor imuToOrientationTensor(const ImuMsg& msg, torch::Device device) {
  // 1. Assume you receive a ROS 2 geometry_msgs Quaternion
  geometry_msgs::msg::Quaternion msg_quat = msg.orientation;
  
  // 2. Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(msg_quat, tf2_quat);
  
  // 3. Convert tf2::Quaternion to tf2::Matrix3x3
  tf2::Matrix3x3 matrix(tf2_quat);
  
  // 4. Extract Euler angles (Roll, Pitch, Yaw) in radians
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);

  std::array<float, 3> grav_vec{
      static_cast<float>(roll),
      static_cast<float>(pitch),
      static_cast<float>(yaw),
  };

  // TODO: Need to change how we calculate the grav vec here

  return torch::tensor(torch::ArrayRef<float>(grav_vec.data(), grav_vec.size())).to(device);
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

  auto T_w_r = T_w_p * T_p_r.inverse();

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

  // TODO: Need to correct this using current loc_res, the odom velocity and gravity vector
  auto loc_res = poseToVec(T_w_r);
  auto loc_res_tensor = loc_res.unsqueeze(0).to(impl_->device);  // (1,6)
  const auto& odom_vel = w_p_r_in_r;
  std::array<float, 6> odom_vel_arr{
      static_cast<float>(odom_vel(0)), static_cast<float>(odom_vel(1)),
      static_cast<float>(odom_vel(2)), static_cast<float>(odom_vel(3)),
      static_cast<float>(odom_vel(4)), static_cast<float>(odom_vel(5)),
  };
  auto odom_vel_tensor =
      torch::tensor(torch::ArrayRef<float>(odom_vel_arr.data(), odom_vel_arr.size()))
          .unsqueeze(0).to(impl_->device);  // (1,6)
  torch::Tensor grav_vec = imuToOrientationTensor(*cur_imu_msg_, impl_->device).unsqueeze(0);  // (1,3)

  std::vector<torch::Tensor> vec_inputs_list{loc_res_tensor, odom_vel_tensor, grav_vec};
  torch::Tensor vector_inputs = torch::cat(vec_inputs_list, /*dim=*/1);
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
  // output shape: (1, T, 3) where each row is [dx, dy, d_yaw] in the path frame
  timer[3]->start();
  auto out = output.squeeze(0);  // (T, 3)
  const int64_t n = std::min(T, out.size(0));
  const float* data = out.data_ptr<float>();

  for (int64_t i = 0; i < n; ++i) {
    const float dx   = data[i * 3 + 0];
    const float dy   = data[i * 3 + 1];
    const float dyaw = data[i * 3 + 2];

    // Build a small SE3 correction from the predicted lateral/longitudinal/yaw error
    Eigen::Matrix<double, 6, 1> xi;
    xi << static_cast<double>(dx),
          static_cast<double>(dy),
          0.0,
          0.0,
          0.0,
          static_cast<double>(dyaw);

    reference_poses[i] = reference_poses[i] * lgmath::se3::Transformation(xi);
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
