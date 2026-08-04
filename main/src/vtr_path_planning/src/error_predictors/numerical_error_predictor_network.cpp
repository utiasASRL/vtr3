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
#include "vtr_path_planning/error_predictors/correction_utils.hpp"

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

// Rotate the frozen world-frame gravity estimate into the robot body frame
torch::Tensor gravityTensor(const Eigen::Vector3d& g_world,
                            const lgmath::se3::Transformation& T_r_w,
                            torch::Device device) {
  Eigen::Vector3d g_body = T_r_w.matrix().topLeftCorner<3,3>() * g_world;
  std::array<float, 3> grav{
      static_cast<float>(g_body(0)),
      static_cast<float>(g_body(1)),
      static_cast<float>(g_body(2)),
  };
  return torch::tensor(torch::ArrayRef<float>(grav.data(), grav.size())).to(device);
}

std::array<double, 6> poseToVecArr(const lgmath::se3::Transformation& T) {
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

  return {tx, ty, tz,
          scale * (r21 - r12),
          scale * (r02 - r20),
          scale * (r10 - r01)};
}

torch::Tensor poseToVec(const lgmath::se3::Transformation& T) {
  const auto vals64 = poseToVecArr(T);
  std::array<float, 6> vals{
      static_cast<float>(vals64[0]), static_cast<float>(vals64[1]),
      static_cast<float>(vals64[2]), static_cast<float>(vals64[3]),
      static_cast<float>(vals64[4]), static_cast<float>(vals64[5]),
  };
  return torch::tensor(torch::ArrayRef<float>(vals.data(), vals.size()));
}

}  // namespace

NumericalErrorPredictorNetwork::NumericalErrorPredictorNetwork(
    rclcpp::Node::SharedPtr node,
    const std::string& imu_topic,
    const std::string& model_path,
    bool use_gpu,
    bool invert_correction,
    bool lateral_only_correction,
    NNTargetMode nn_target_mode,
    double smoothing_alpha)
    : impl_(std::make_unique<Impl>()),
      invert_correction_(invert_correction),
      lateral_only_correction_(lateral_only_correction),
      nn_target_mode_(nn_target_mode),
      smoothing_alpha_(smoothing_alpha) {
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

        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (gravity_locked_) return;

        if (gravity_first_stamp_ns_ < 0) gravity_first_stamp_ns_ = stamp_ns;
        const int64_t window_ns = static_cast<int64_t>(kGravWindowSeconds * 1e9);
        if ((stamp_ns - gravity_first_stamp_ns_) > window_ns ||
            gravity_sample_count_ >= kGravMaxSamples) {
          gravity_world_ = gravity_sample_count_ > 0
                                ? Eigen::Vector3d(gravity_world_sum_ / static_cast<double>(gravity_sample_count_))
                                : Eigen::Vector3d(Eigen::Vector3d::Zero());
          gravity_locked_ = true;
          CLOG(INFO, "path_planning")
              << "NumericalErrorPredictorNetwork: locked gravity estimate after "
              << gravity_sample_count_ << " samples: " << gravity_world_.transpose();
          return;
        }

        auto world_accel = accelToWorld(*msg);
        gravity_world_sum_ += Eigen::Vector3d(world_accel[0], world_accel[1], world_accel[2]);
        ++gravity_sample_count_;
      });

  CLOG(INFO, "path_planning")
      << "NumericalErrorPredictorNetwork: loaded model from " << model_path
      << ", device = " << impl_->device;
}

NumericalErrorPredictorNetwork::~NumericalErrorPredictorNetwork() = default;

std::vector<std::array<double, 3>> NumericalErrorPredictorNetwork::predictError(
    const RobotState& robot_state,
    const tactic::Timestamp& curr_time,
    std::vector<lgmath::se3::Transformation>& reference_poses,
    bool apply_correction,
    PredictorInputSnapshot* input_snapshot) {

  std::vector<std::array<double, 3>> corrections(reference_poses.size(), {0.0, 0.0, 0.0});

  if (reference_poses.empty()) {
    CLOG(WARNING, "path_planning")
        << "NumericalErrorPredictorNetwork::predictError: reference_poses is empty.";
    return corrections;
  }

  auto& chain = robot_state.chain.ptr();
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] =
      getChainInfo(*chain);
  CLOG(DEBUG, "path_planning")
      << "NumericalErrorPredictorNetwork: cycle curr_time=" << static_cast<int64_t>(curr_time)
      << " stamp=" << static_cast<int64_t>(stamp) << " curr_sid=" << curr_sid;

  if (input_snapshot) {
    input_snapshot->sid = curr_sid;
    input_snapshot->T_p_r = T_p_r;
    input_snapshot->T_w_p = T_w_p;
    input_snapshot->w_p_r_in_r = w_p_r_in_r;
    input_snapshot->reference_poses_raw = reference_poses;

    const double dt = static_cast<double>(curr_time - stamp) * 1e-9;
    const Eigen::Matrix<double, 6, 1> xi_p_r_in_r(-dt * w_p_r_in_r);
    input_snapshot->T_p_r_extp = T_p_r * lgmath::se3::Transformation(xi_p_r_in_r);
    input_snapshot->lidar_stamp = stamp;
    input_snapshot->extrap_wall_time = curr_time;
  }

  torch::NoGradGuard no_grad;

  // Sequence input: (1, T, 6): reference poses from MPC in SE3
  const int64_t T = static_cast<int64_t>(reference_poses.size());
  std::vector<torch::Tensor> pose_vecs;
  pose_vecs.reserve(T);
  if (input_snapshot) input_snapshot->sequence.reserve(T);
  for (const auto& pose : reference_poses) {
    pose_vecs.push_back(poseToVec(pose));
    if (input_snapshot) input_snapshot->sequence.push_back(poseToVecArr(pose));
  }
  torch::Tensor sequence = torch::stack(pose_vecs, 0)
                               .unsqueeze(0)
                               .to(impl_->device);
  CLOG(DEBUG, "path_planning") << "NumericalErrorPredictorNetwork: sequence = " << sequence;

  const Eigen::Matrix<double, 6, 1> loc_res_xi = T_p_r.vec();

  const Eigen::Vector3d robot_xyz_in_path = T_p_r.r_ab_inb();
  // TODO: Try inverse here for the TF
  std::array<float, 6> loc_res_arr{
      static_cast<float>(robot_xyz_in_path(0)),
      static_cast<float>(robot_xyz_in_path(1)),
      static_cast<float>(robot_xyz_in_path(2)),
      static_cast<float>(loc_res_xi(3)),
      static_cast<float>(loc_res_xi(4)),
      static_cast<float>(loc_res_xi(5)),
  };
  auto loc_res_tensor = torch::tensor(torch::ArrayRef<float>(loc_res_arr.data(), loc_res_arr.size()))
                            .unsqueeze(0).to(impl_->device);  // (1,6)
  CLOG(DEBUG, "path_planning") << "NumericalErrorPredictorNetwork: loc_res_tensor = "
                              << loc_res_tensor;
  if (input_snapshot) {
    for (size_t i = 0; i < loc_res_arr.size(); ++i)
      input_snapshot->loc_res[i] = static_cast<double>(loc_res_arr[i]);
  }

  // Negate x vel
  const Eigen::Matrix<double, 6, 1> odom_vel = -w_p_r_in_r;
  // TODO: Double check this
  std::array<float, 6> odom_vel_arr{
      static_cast<float>(odom_vel(0)), static_cast<float>(odom_vel(1)),
      static_cast<float>(odom_vel(2)), static_cast<float>(odom_vel(3)),
      static_cast<float>(odom_vel(4)), static_cast<float>(odom_vel(5)),
  };
  CLOG(DEBUG, "path_planning") << "NumericalErrorPredictorNetwork: odom_vel_arr = "
                              << odom_vel_arr[0] << ", " << odom_vel_arr[1]
                              << ", " << odom_vel_arr[2] << ", "
                              << odom_vel_arr[3] << ", " << odom_vel_arr[4]
                              << ", " << odom_vel_arr[5];
  auto odom_vel_tensor =
      torch::tensor(torch::ArrayRef<float>(odom_vel_arr.data(), odom_vel_arr.size()))
          .unsqueeze(0).to(impl_->device);  // (1,6)
  if (input_snapshot) {
    for (size_t i = 0; i < odom_vel_arr.size(); ++i)
      input_snapshot->odom_vel[i] = static_cast<double>(odom_vel_arr[i]);
  }

  const lgmath::se3::Transformation T_r_w = (T_w_p * T_p_r).inverse();
  // Double check this as well
  Eigen::Vector3d g_world;
  bool grav_ready;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    g_world = gravity_world_;
    grav_ready = gravity_locked_;
  }
  if (!grav_ready) {
    CLOG(WARNING, "path_planning")
        << "NumericalErrorPredictorNetwork::predictError: gravity estimate not yet "
           "locked (still accumulating first " << kGravWindowSeconds
        << "s of IMU data); using zero gravity for this cycle.";
  }
  torch::Tensor grav_vec = gravityTensor(g_world, T_r_w, impl_->device).unsqueeze(0);  // (1,3)
  CLOG(DEBUG, "path_planning") << "NumericalErrorPredictorNetwork: grav_vec = " << grav_vec;
  if (input_snapshot) {
    const Eigen::Vector3d g_body = T_r_w.matrix().topLeftCorner<3,3>() * g_world;
    input_snapshot->grav_vec = {g_body(0), g_body(1), g_body(2)};
    input_snapshot->valid = true;
  }

  std::vector<torch::Tensor> vec_inputs_list{loc_res_tensor, odom_vel_tensor, grav_vec};
  torch::Tensor vector_inputs = torch::cat(vec_inputs_list, 1);  // (1,15)
  CLOG(DEBUG, "path_planning") << "NumericalErrorPredictorNetwork: vector_inputs "
                              << "[loc_res(6), odom_vel(6), grav_vec(3)] = "
                              << vector_inputs;

  // Run inference
  std::vector<torch::jit::IValue> inputs{vector_inputs, sequence};
  torch::Tensor output;
  try {
    // Model output: (1, T, 3) 
    output = impl_->model.forward(inputs).toTensor().cpu();
  } catch (const c10::Error& e) {
    CLOG(ERROR, "path_planning")
        << "NumericalErrorPredictorNetwork: forward pass failed: " << e.what();
    return corrections;
  }

  // Apply predicted errors to reference poses
  auto out = output.squeeze(0);  // (T, 3)
  const int64_t n = std::min(T, out.size(0));
  const float* data = out.data_ptr<float>();
  CLOG(DEBUG, "path_planning") << "NumericalErrorPredictorNetwork: output = " << output;

  const bool have_prev = (smoothing_alpha_ < 1.0) &&
                         (prev_sid_ == static_cast<int>(curr_sid)) &&
                         (static_cast<int64_t>(prev_corrections_.size()) >= n);
  std::vector<std::array<double, 3>> filtered(n);

  if (input_snapshot) {
    input_snapshot->raw_output.resize(n);
    input_snapshot->final_output.resize(n);
  }

  for (int64_t i = 0; i < n; ++i) {
    const double raw_dx   = static_cast<double>(data[i * 3 + 0]);
    const double raw_dy   = static_cast<double>(data[i * 3 + 1]);
    const double raw_dyaw = static_cast<double>(data[i * 3 + 2]);
    if (input_snapshot) input_snapshot->raw_output[i] = {raw_dx, raw_dy, raw_dyaw};

    double dx = raw_dx, dy = raw_dy, dyaw = raw_dyaw;
    if (invert_correction_) {
      dx = -dx;
      dy = -dy;
    }

    if (have_prev) {
      const auto& prev = prev_corrections_[i];
      dx   = smoothing_alpha_ * dx   + (1.0 - smoothing_alpha_) * prev[0];
      dy   = smoothing_alpha_ * dy   + (1.0 - smoothing_alpha_) * prev[1];
      dyaw = smoothing_alpha_ * dyaw + (1.0 - smoothing_alpha_) * prev[2];
    }
    filtered[i] = {dx, dy, dyaw};

    corrections[i] = {dx, dy, dyaw};
    if (input_snapshot) input_snapshot->final_output[i] = {dx, dy, dyaw};

    if (apply_correction) {
      reference_poses[i] = (nn_target_mode_ == NNTargetMode::SE2)
          ? applySE2Correction(reference_poses[i], dx, dy, dyaw)
          : applyPoseCorrection(reference_poses[i], dx, dy, dyaw);
    }
  }

  if (smoothing_alpha_ < 1.0) {
    prev_corrections_ = std::move(filtered);
    prev_sid_ = static_cast<int>(curr_sid);
  }

  CLOG(DEBUG, "path_planning")
      << "NumericalErrorPredictorNetwork: computed corrections for " << n
      << " reference poses (applied=" << apply_correction << ").";

  return corrections;
}

}  // namespace path_planning
}  // namespace vtr
