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
 * \file numerical_error_predictor_network.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/error_predictors/numerical_error_predictor_network.hpp"

#include <torch/script.h>
#include <torch/cuda.h>
#include <vtr_torch/types.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

#include <stdexcept>

namespace vtr {
namespace path_planning {

struct NumericalErrorPredictorNetwork::Impl {
  Module model;
  torch::Device device{torch::kCPU};
};

namespace {

// Rotate body-frame accel to world frame using the IMU orientation quaternion
std::array<double, 3> accelToWorld(const ImuMsg& msg) {
  const double ax = msg.linear_acceleration.x;
  const double ay = msg.linear_acceleration.y;
  const double az = msg.linear_acceleration.z;
  tf2::Quaternion q;
  tf2::fromMsg(msg.orientation, q);
  tf2::Matrix3x3 R(q);
  return {R[0][0]*ax + R[0][1]*ay + R[0][2]*az,
          R[1][0]*ax + R[1][1]*ay + R[1][2]*az,
          R[2][0]*ax + R[2][1]*ay + R[2][2]*az};
}

// Average world-frame accel samples and rotate into the robot body frame
torch::Tensor gravityTensor(const std::vector<Eigen::Vector3d>& world_accels,
                            const lgmath::se3::Transformation& T_r_w,
                            torch::Device device) {
  Eigen::Vector3d g_world = Eigen::Vector3d::Zero();
  for (const auto& a : world_accels) g_world += a;
  if (!world_accels.empty()) g_world /= static_cast<double>(world_accels.size());

  Eigen::Vector3d g_body = T_r_w.matrix().topLeftCorner<3,3>() * g_world;

  std::array<float, 3> grav{
      static_cast<float>(g_body(0)),
      static_cast<float>(g_body(1)),
      static_cast<float>(g_body(2)),
  };
  return torch::tensor(torch::ArrayRef<float>(grav.data(), grav.size())).to(device);
}

torch::Tensor poseToVec(const lgmath::se3::Transformation& T) {
  const Eigen::Matrix4d M = T.matrix();
  const double tx = M(0, 3), ty = M(1, 3), tz = M(2, 3);

  const double r00 = M(0,0), r01 = M(0,1), r02 = M(0,2);
  const double r10 = M(1,0), r11 = M(1,1), r12 = M(1,2);
  const double r20 = M(2,0), r21 = M(2,1), r22 = M(2,2);

  const double cos_phi = std::clamp((r00 + r11 + r22 - 1.0) / 2.0, -1.0, 1.0);
  const double phi = std::acos(cos_phi);

  // scale = phi / (2 sin phi); use sinc to handle phi→0 safely
  const double sinc_val = (std::abs(phi) < 1e-8)
      ? 1.0
      : std::sin(phi) / phi;
  const double scale = 0.5 / sinc_val;

  std::array<float, 6> vals{
      static_cast<float>(tx),
      static_cast<float>(ty),
      static_cast<float>(tz),
      static_cast<float>(scale * (r21 - r12)),
      static_cast<float>(scale * (r02 - r20)),
      static_cast<float>(scale * (r10 - r01)),
  };
  return torch::tensor(torch::ArrayRef<float>(vals.data(), vals.size()));
}

}  // namespace

NumericalErrorPredictorNetwork::NumericalErrorPredictorNetwork(
    rclcpp::Node::SharedPtr node,
    const std::string& imu_topic,
    const std::string& model_path,
    bool use_gpu)
    : impl_(std::make_unique<Impl>()) {
  if (use_gpu) {
    if (torch::cuda::is_available()) {
      impl_->device = torch::kCUDA;
    } else {
      CLOG(WARNING, "path_planning")
          << "NumericalErrorPredictorNetwork: CUDA requested but not available, "
             "falling back to CPU.";
    }
  }

  try {
    impl_->model = torch::jit::load(model_path);
  } catch (const c10::Error& e) {
    throw std::runtime_error(
        "NumericalErrorPredictorNetwork: failed to load model from " + model_path +
        ": " + e.what());
  }

  impl_->model.to(impl_->device);
  impl_->model.eval();

  imu_sub_ = node->create_subscription<ImuMsg>(
      imu_topic, rclcpp::QoS(10),
      [this](const ImuMsg::SharedPtr msg) {
        cur_imu_msg_ = msg;
        if (!msg->header.stamp.sec && !msg->header.stamp.nanosec) return;
        const int64_t stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
                                 + msg->header.stamp.nanosec;
        auto world_accel = accelToWorld(*msg);
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_buffer_.push_back({stamp_ns, world_accel});
        const int64_t window_ns = static_cast<int64_t>(kGravWindowSeconds * 1e9);
        while (!imu_buffer_.empty() &&
               (stamp_ns - imu_buffer_.front().stamp_ns) > window_ns) {
          imu_buffer_.pop_front();
        }
      });

  CLOG(INFO, "path_planning")
      << "NumericalErrorPredictorNetwork: loaded model from " << model_path
      << ", device = " << impl_->device;
}

NumericalErrorPredictorNetwork::~NumericalErrorPredictorNetwork() = default;

void NumericalErrorPredictorNetwork::predictError(
    const RobotState& robot_state,
    const tactic::Timestamp&,
    std::vector<lgmath::se3::Transformation>& reference_poses) {

  if (reference_poses.empty()) {
    CLOG(WARNING, "path_planning")
        << "NumericalErrorPredictorNetwork::predictError: reference_poses is empty.";
    return;
  }

  auto& chain = robot_state.chain.ptr();
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);

  torch::NoGradGuard no_grad;

  // Sequence input: (1, T, 6): reference poses from MPC in SE3
  const int64_t T = static_cast<int64_t>(reference_poses.size());
  std::vector<torch::Tensor> pose_vecs;
  pose_vecs.reserve(T);
  for (const auto& pose : reference_poses) {
    pose_vecs.push_back(poseToVec(pose));
  }
  torch::Tensor sequence = torch::stack(pose_vecs, 0)
                               .unsqueeze(0)
                               .to(impl_->device);

  // Dataset: loc_res = ne.T.vec() with xi[:2] replaced by the exact robot
  // position in path frame (r_ba_ina), not xi[:2] negated (only an approximation
  // for small rotation between path and robot frames). z/rotation stay as raw xi.
  const Eigen::Matrix<double, 6, 1> loc_res_xi = T_p_r.vec();
  const Eigen::Vector3d robot_xyz_in_path = T_p_r.r_ba_ina();
  std::array<float, 6> loc_res_arr{
      static_cast<float>(robot_xyz_in_path(0)),
      static_cast<float>(robot_xyz_in_path(1)),
      static_cast<float>(loc_res_xi(2)),
      static_cast<float>(loc_res_xi(3)),
      static_cast<float>(loc_res_xi(4)),
      static_cast<float>(loc_res_xi(5)),
  };
  auto loc_res_tensor = torch::tensor(torch::ArrayRef<float>(loc_res_arr.data(), loc_res_arr.size()))
                            .unsqueeze(0).to(impl_->device);  // (1,6)

  // odom: body-frame velocity twist. w_p_r_in_r from the chain is actually the
  const Eigen::Matrix<double, 6, 1> odom_vel = -w_p_r_in_r;
  std::array<float, 6> odom_vel_arr{
      static_cast<float>(odom_vel(0)), static_cast<float>(odom_vel(1)),
      static_cast<float>(odom_vel(2)), static_cast<float>(odom_vel(3)),
      static_cast<float>(odom_vel(4)), static_cast<float>(odom_vel(5)),
  };
  auto odom_vel_tensor =
      torch::tensor(torch::ArrayRef<float>(odom_vel_arr.data(), odom_vel_arr.size()))
          .unsqueeze(0).to(impl_->device);  // (1,6)

  // grav_vec: world-frame gravity averaged over 1s at beginning, then rotated to robot body frame
  const lgmath::se3::Transformation T_r_w = (T_w_p * T_p_r).inverse();
  std::vector<Eigen::Vector3d> world_accels;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    world_accels.reserve(imu_buffer_.size());
    for (const auto& s : imu_buffer_) {
      world_accels.emplace_back(s.accel_world[0], s.accel_world[1], s.accel_world[2]);
    }
  }
  if (world_accels.empty()) {
    CLOG(WARNING, "path_planning")
        << "NumericalErrorPredictorNetwork::predictError: IMU buffer empty, gravity will be zero.";
  }
  torch::Tensor grav_vec = gravityTensor(world_accels, T_r_w, impl_->device).unsqueeze(0);  // (1,3)

  std::vector<torch::Tensor> vec_inputs_list{loc_res_tensor, odom_vel_tensor, grav_vec};
  torch::Tensor vector_inputs = torch::cat(vec_inputs_list, 1);  // (1,15)

  // Run inference
  std::vector<torch::jit::IValue> inputs{vector_inputs, sequence};
  torch::Tensor output;
  try {
    // Model output: (1, T, 3) 
    output = impl_->model.forward(inputs).toTensor().cpu();
  } catch (const c10::Error& e) {
    CLOG(ERROR, "path_planning")
        << "NumericalErrorPredictorNetwork: forward pass failed: " << e.what();
    return;
  }

  // Apply predicted errors to reference poses
  auto out = output.squeeze(0);  // (T, 3)
  const int64_t n = std::min(T, out.size(0));
  const float* data = out.data_ptr<float>();

  for (int64_t i = 0; i < n; ++i) {
    const double dx   = static_cast<double>(data[i * 3 + 0]);
    const double dy   = static_cast<double>(data[i * 3 + 1]);
    const double dyaw = static_cast<double>(data[i * 3 + 2]);

    // target_error = ref_xyz - actual_xyz in path frame
    Eigen::Matrix4d M = reference_poses[i].matrix();
    M(0, 3) -= dx;
    M(1, 3) -= dy;
    // yaw: apply as body-frame rotation
    Eigen::Matrix4d R_dyaw = Eigen::Matrix4d::Identity();
    R_dyaw(0,0) =  std::cos(-dyaw);  R_dyaw(0,1) = -std::sin(-dyaw);
    R_dyaw(1,0) =  std::sin(-dyaw);  R_dyaw(1,1) =  std::cos(-dyaw);
    M = M * R_dyaw;
    reference_poses[i] = lgmath::se3::Transformation(M);
  }

  CLOG(DEBUG, "path_planning")
      << "NumericalErrorPredictorNetwork: applied corrections to " << n
      << " reference poses.";
}

}  // namespace path_planning
}  // namespace vtr
