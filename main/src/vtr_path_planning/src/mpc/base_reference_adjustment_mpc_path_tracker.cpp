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
 * \file bicycle_mpc_path_tracker.cpp
 * \author Luka Antonyshyn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/base_reference_adjustment_mpc_path_tracker.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vtr_path_planning/cbit/utils.hpp>
#include <fstream>
#include <iomanip>

namespace vtr::path_planning {

auto BaseReferenceAdjustmentMPCPathTracker::Config::loadConfig(BaseReferenceAdjustmentMPCPathTracker::Config::Ptr config, 
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix)->void{
  // MPC Configs:
  // SPEED SCHEDULER PARAMETERS
  config->planar_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.planar_curv_weight", config->planar_curv_weight);
  config->profile_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.profile_curv_weight", config->profile_curv_weight);
  config->eop_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.eop_weight", config->eop_weight);
  config->min_vel = node->declare_parameter<double>(prefix + ".speed_scheduler.min_vel", config->min_vel);
  config->eop_min_vel = node->declare_parameter<double>(prefix + ".speed_scheduler.eop_min_vel", config->min_vel);

  // CONTROLLER PARAMS
  config->extrapolate_robot_pose = node->declare_parameter<bool>(prefix + ".mpc.extrapolate_robot_pose", config->extrapolate_robot_pose);
  config->mpc_verbosity = node->declare_parameter<bool>(prefix + ".mpc.mpc_verbosity", config->mpc_verbosity);
  config->forward_vel = node->declare_parameter<double>(prefix + ".mpc.forward_vel", config->forward_vel);
  config->max_lin_vel = node->declare_parameter<double>(prefix + ".mpc.max_lin_vel", config->max_lin_vel);
  config->max_ang_vel = node->declare_parameter<double>(prefix + ".mpc.max_ang_vel", config->max_ang_vel);
  config->max_lin_acc = node->declare_parameter<double>(prefix + ".mpc.max_lin_acc", config->max_lin_acc);
  config->max_ang_acc = node->declare_parameter<double>(prefix + ".mpc.max_ang_acc", config->max_ang_acc);
  config->robot_linear_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_linear_velocity_scale", config->robot_linear_velocity_scale);
  config->robot_angular_velocity_scale = node->declare_parameter<double>(prefix + ".mpc.robot_angular_velocity_scale", config->robot_angular_velocity_scale);
  config->repeat_flipped = node->declare_parameter<bool>(prefix + ".mpc.repeat_flipped", config->repeat_flipped);
  config->alpha = node->declare_parameter<float>(prefix + ".mpc.alpha", config->alpha);

  // REFERENCE ADJUSTMENT PARAMS
  config->sig_img_topic = node->declare_parameter<std::string>(prefix + ".error_predictor.sig_img_topic", config->sig_img_topic);
  config->dpt_img_topic = node->declare_parameter<std::string>(prefix + ".error_predictor.dpt_img_topic", config->dpt_img_topic);
  config->imu_topic     = node->declare_parameter<std::string>(prefix + ".error_predictor.imu_topic", config->imu_topic);
  config->reference_adjustment_model_path = node->declare_parameter<std::string>(prefix + ".error_predictor.model_path", config->reference_adjustment_model_path);
  config->reference_adjustment_mode = static_cast<ReferenceAdjustmentMode>(
      node->declare_parameter<int>(prefix + ".error_predictor.mode", static_cast<int>(config->reference_adjustment_mode)));
  config->prediction_log_path = node->declare_parameter<std::string>(prefix + ".error_predictor.prediction_log_path", config->prediction_log_path);
}

// Subclasses must implement their own Config::fromROS.
auto BaseReferenceAdjustmentMPCPathTracker::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  auto base_config = std::static_pointer_cast<BasePathPlanner::Config>(config);
  *base_config =  *BasePathPlanner::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);

  return config;
}

BaseReferenceAdjustmentMPCPathTracker::BaseReferenceAdjustmentMPCPathTracker(const Config::ConstPtr& config,
                                       const RobotState::Ptr& robot_state,
                                       const tactic::GraphBase::Ptr& graph,
                                       const Callback::Ptr& callback)
    : BaseMPCPathTracker(config, robot_state, graph, callback),
      base_config_(config),
      robot_state_{robot_state},
      log_path_(config->prediction_log_path) {

  auto node = robot_state_->node.ptr();
  if (config->reference_adjustment_mode == ReferenceAdjustmentMode::ImageNeuralNetwork) {
    if (!config->reference_adjustment_model_path.empty()) {
      error_predictor_ = std::make_shared<ImageErrorPredictorNetwork>(
          node,
          base_config_->sig_img_topic,
          base_config_->dpt_img_topic,
          base_config_->imu_topic,
          base_config_->reference_adjustment_model_path);
      CLOG(INFO, "cbit.control") << "Initialized image neural network for reference adjustment.";
    } else {
      CLOG(WARNING, "cbit.control") << "Reference adjustment model path is empty. Image neural network will not be initialized.";
    }
  } else if (config->reference_adjustment_mode == ReferenceAdjustmentMode::NumericalNeuralNetwork) {
    if (!config->reference_adjustment_model_path.empty()) {
      error_predictor_ = std::make_shared<NumericalErrorPredictorNetwork>(
          node,
          base_config_->imu_topic,
          base_config_->reference_adjustment_model_path);
      CLOG(INFO, "cbit.control") << "Initialized numerical neural network for reference adjustment.";
    } else {
      CLOG(WARNING, "cbit.control") << "Reference adjustment model path is empty. Numerical neural network will not be initialized.";
    }
  } else if (config->reference_adjustment_mode == ReferenceAdjustmentMode::HistoryBased) {
    error_predictor_ = std::make_shared<HistoryLookupErrorPredictor>();
    CLOG(INFO, "cbit.control") << "Initialized history-based reference adjustment.";
  } else {
    CLOG(INFO, "cbit.control") << "No reference adjustment will be applied.";
  }
}

BaseReferenceAdjustmentMPCPathTracker::~BaseReferenceAdjustmentMPCPathTracker() {
  if (log_path_.empty() || prediction_log_.empty()) return;
  std::ofstream f(log_path_);
  if (!f) {
    CLOG(WARNING, "cbit.control") << "Could not open prediction log: " << log_path_;
    return;
  }
  f << std::fixed << std::setprecision(6);
  f << "timestamp_ns,sid,actual_dx,actual_dy,actual_dyaw,pred_dx,pred_dy,pred_dyaw\n";
  for (const auto& e : prediction_log_) {
    f << e.timestamp_ns << ',' << e.sid << ','
      << e.actual_dx  << ',' << e.actual_dy  << ',' << e.actual_dyaw  << ','
      << e.pred_dx    << ',' << e.pred_dy    << ',' << e.pred_dyaw    << '\n';
  }
  CLOG(INFO, "cbit.control") << "Wrote " << prediction_log_.size()
                             << " prediction log entries to " << log_path_;
}

void BaseReferenceAdjustmentMPCPathTracker::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& curr_time) {

  auto& chain = robot_state.chain.ptr();
  std::vector<double> p_rollout;
  for (int j = 1; j < mpcConfig->N + 1; j++) {
    p_rollout.push_back(state_p + j * mpcConfig->VF * mpcConfig->DT);
  }

  mpcConfig->reference_poses.clear();
  auto referenceInfo = generateHomotopyReference(p_rollout, chain, T_w_p*T_p_r_extp);
  std::vector<lgmath::se3::Transformation> local_reference_poses;
  
  for (const auto& Tf : referenceInfo.poses) {
    mpcConfig->reference_poses.push_back(tf_to_global(T_w_p.inverse() * Tf));
    local_reference_poses.push_back(T_w_p.inverse() * Tf);

    CLOG(DEBUG, "cbit.control")
        << "Adding reference pose: " << tf_to_global(T_w_p.inverse() * Tf);
  }

  if (error_predictor_) {
    std::vector<lgmath::se3::Transformation> original_local_poses = local_reference_poses;

    error_predictor_->predictError(robot_state, curr_time, local_reference_poses);
    mpcConfig->reference_poses.clear();
    std::vector<lgmath::se3::Transformation> corrected_world_poses;
    std::vector<lgmath::se3::Transformation> predicted_robot_world_poses;
    for (size_t i = 0; i < local_reference_poses.size(); ++i) {
      mpcConfig->reference_poses.push_back(tf_to_global(local_reference_poses[i]));
      corrected_world_poses.push_back(T_w_p * local_reference_poses[i]);
      // target_error = ref_xyz - actual_xyz in path frame (additive translation offset).
      // xi_translation = original.translation - corrected.translation (both in path frame).
      // predicted robot position = original.translation + xi_translation
      //                          = original.translation + (original - corrected).translation
      // Keep the reference orientation (robot faces same direction as the reference waypoint).
      const Eigen::Vector4d orig_t = original_local_poses[i].matrix().col(3);
      const Eigen::Vector4d corr_t = local_reference_poses[i].matrix().col(3);
      const Eigen::Vector4d xi_t   = orig_t - corr_t;  // path-frame translation offset

      Eigen::Matrix4d pred_M = original_local_poses[i].matrix();
      pred_M.col(3) = orig_t + xi_t;  // shift translation by +xi in path frame
      predicted_robot_world_poses.push_back(T_w_p * lgmath::se3::Transformation(pred_M));
    }
    vis_->publishCorrectedReferencePoses(corrected_world_poses, curr_time);
    vis_->publishPredictedRobotPath(predicted_robot_world_poses, curr_time);

    // Logging: record prediction now; match against actual at the next step.
    // actual_dx/dy/dyaw = T_r_p (robot→path) = loc_res in dataset convention.
    // pred_dx/dy/dyaw   = xi predicted for pose[0], extracted via the same
    //                     axis-angle formula as dataset _reduce_se3_to_vec.
    if (!log_path_.empty() && !original_local_poses.empty()) {
      const auto [stamp, w_p_r_in_r, T_p_r, T_w_p_chain, T_w_v_odo, T_r_v_odo, curr_sid] =
          getChainInfo(*robot_state.chain.ptr());

      // Axis-angle extraction matching dataset _reduce_se3_to_vec:
      //   translation = matrix column 3; rotation = phi/(2 sin phi) * skew-sym terms
      auto matToAxisAngle = [](const lgmath::se3::Transformation& T)
          -> std::array<double, 3> {
        const auto M = T.matrix();
        const double tx  = M(0,3), ty = M(1,3);
        const double r00 = M(0,0), r11 = M(1,1), r22 = M(2,2);
        const double r21 = M(2,1), r12 = M(1,2);
        const double r02 = M(0,2), r20 = M(2,0);
        const double r10 = M(1,0), r01 = M(0,1);
        const double cos_phi = std::clamp((r00+r11+r22-1.0)/2.0, -1.0, 1.0);
        const double phi = std::acos(cos_phi);
        const double sinc = (std::abs(phi) < 1e-8) ? 1.0 : std::sin(phi)/phi;
        const double scale = 0.5 / sinc;
        return {tx, ty, scale*(r10 - r01)};  // [x, y, yaw]
      };

      // actual: loc_res = ne.T = T_{teach,repeat} = T_p_r (path→robot). Pass directly.
      const auto act = matToAxisAngle(T_p_r);

      // Flush previous step's prediction paired with this step's actual
      if (pending_pred_.valid) {
        prediction_log_.push_back({
            pending_pred_.timestamp_ns, pending_pred_.sid,
            act[0], act[1], act[2],
            pending_pred_.pred_dx, pending_pred_.pred_dy, pending_pred_.pred_dyaw
        });
      }

      // predicted xi for pose[0]: corrected[0] = original[0] * T(-xi)
      // => T(xi) = corrected[0]^-1 * original[0]
      const auto xi_T = local_reference_poses[0].inverse() * original_local_poses[0];
      const auto pred = matToAxisAngle(xi_T);

      pending_pred_ = {true, static_cast<int64_t>(curr_time), curr_sid,
                       pred[0], pred[1], pred[2]};
    }
  }

  // Detect end of path and set the corresponding cost weight vector element to zero
  mpcConfig->cost_weights.clear();
  mpcConfig->cost_weights.push_back(1.0);
  auto last_pose = (T_w_p.inverse()*referenceInfo.poses[0]).vec();
  
  int end_ind = -1;
  auto weighting = 1.0;
  for (int i = 1; i < mpcConfig->N; i++) {
    auto curr_pose = (T_w_p.inverse() * referenceInfo.poses[i]).vec();
    auto dist = (curr_pose - last_pose).norm();
    if (end_ind < 0 && dist < base_config_->end_of_path_distance_threshold) {
      end_ind = i-1;
      weighting = (float)  1.0 / (mpcConfig->N - end_ind);
      CLOG(DEBUG, "cbit.control") << "Detected end of path. Setting cost of EoP poses to: " << weighting;
    }
    else if (end_ind >= 0 && dist > base_config_->end_of_path_distance_threshold) {
      weighting = 1.0;
      for (int j = 0; j < i; j++) {
        mpcConfig->cost_weights[j] = weighting;
      }
      end_ind = -1;
      CLOG(DEBUG, "cbit.control") << "False end of path. Setting cost of EoP poses to: " << weighting;
    }

    mpcConfig->cost_weights.push_back(weighting);
    last_pose = curr_pose;
  }

  vis_->publishReferencePoses(referenceInfo.poses, curr_time);
  vis_->publishLocalReferencePoses(local_reference_poses, curr_time);

  if (end_ind == 0)
    end_ind = 1;
  mpcConfig->eop_index = end_ind;

  mpcConfig->up_barrier_q  = referenceInfo.barrier_q_max;
  mpcConfig->low_barrier_q = referenceInfo.barrier_q_min;
}


}  // namespace vtr::path_planning
