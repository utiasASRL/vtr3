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
#include <sstream>

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
  config->history_lookup_mode = static_cast<HistoryLookupMode>(
      node->declare_parameter<int>(prefix + ".error_predictor.history_lookup_mode", static_cast<int>(config->history_lookup_mode)));
  config->history_lookup_invert_correction = node->declare_parameter<bool>(
      prefix + ".error_predictor.history_lookup_invert_correction", config->history_lookup_invert_correction);
  config->nn_invert_correction = node->declare_parameter<bool>(
      prefix + ".error_predictor.nn_invert_correction", config->nn_invert_correction);
  config->prediction_log_path = node->declare_parameter<std::string>(prefix + ".error_predictor.prediction_log_path", config->prediction_log_path);
  config->use_gpu = node->declare_parameter<bool>(prefix + ".error_predictor.use_gpu", config->use_gpu);
  config->apply_corrections = node->declare_parameter<bool>(prefix + ".error_predictor.apply_corrections", config->apply_corrections);
  config->lateral_only_correction = node->declare_parameter<bool>(
      prefix + ".error_predictor.lateral_only_correction", config->lateral_only_correction);
  config->nn_smoothing_alpha = node->declare_parameter<double>(
      prefix + ".error_predictor.nn_smoothing_alpha", config->nn_smoothing_alpha);
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
      graph_(graph),
      log_path_(config->prediction_log_path) {


  auto node = robot_state_->node.ptr();
  if (config->reference_adjustment_mode == ReferenceAdjustmentMode::ImageNeuralNetwork) {
    if (!config->reference_adjustment_model_path.empty()) {
      error_predictor_ = std::make_shared<ImageErrorPredictorNetwork>(
          node,
          base_config_->sig_img_topic,
          base_config_->dpt_img_topic,
          base_config_->imu_topic,
          base_config_->reference_adjustment_model_path,
	  base_config_->use_gpu,
          base_config_->nn_invert_correction,
          base_config_->lateral_only_correction);
      CLOG(INFO, "cbit.control") << "Initialized image neural network for reference adjustment.";
    } else {
      CLOG(WARNING, "cbit.control") << "Reference adjustment model path is empty. Image neural network will not be initialized.";
    }
  } else if (config->reference_adjustment_mode == ReferenceAdjustmentMode::NumericalNeuralNetwork) {
    if (!config->reference_adjustment_model_path.empty()) {
      error_predictor_ = std::make_shared<NumericalErrorPredictorNetwork>(
          node,
          base_config_->imu_topic,
          base_config_->reference_adjustment_model_path,
	  base_config_->use_gpu,
          base_config_->nn_invert_correction,
          base_config_->lateral_only_correction,
          base_config_->nn_smoothing_alpha);
      CLOG(INFO, "cbit.control") << "Initialized numerical neural network for reference adjustment.";
    } else {
      CLOG(WARNING, "cbit.control") << "Reference adjustment model path is empty. Numerical neural network will not be initialized.";
    }
  } else if (config->reference_adjustment_mode == ReferenceAdjustmentMode::HistoryBased) {
    error_predictor_ = std::make_shared<HistoryLookupErrorPredictor>(
        config->history_lookup_mode, graph_, config->history_lookup_invert_correction,
        config->lateral_only_correction);
    CLOG(INFO, "cbit.control") << "Initialized history-based reference adjustment.";
  } else {
    CLOG(INFO, "cbit.control") << "No reference adjustment will be applied.";
  }
}

BaseReferenceAdjustmentMPCPathTracker::~BaseReferenceAdjustmentMPCPathTracker() {
  if (log_path_.empty()) return;
  if (pose_log_.empty() && pred_log_.empty() && input_log_.empty()) return;
  std::ofstream f(log_path_);
  if (!f) {
    CLOG(WARNING, "cbit.control") << "Could not open prediction log: " << log_path_;
    return;
  }
  f << std::fixed << std::setprecision(9);

  auto write_mat = [&](const lgmath::se3::Transformation& T) {
    const auto M = T.matrix();
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        f << ',' << M(r, c);
  };


  f << "type,t0_ns,t1_ns,sid,step,v0,v1,v2";
  for (int i = 0; i < 16; ++i) f << ",m0_" << i;
  for (int i = 0; i < 16; ++i) f << ",m1_" << i;
  f << '\n';

  for (const auto& e : pose_log_) {
    f << "pose," << e.timestamp_ns << ",,,,,," ;
    write_mat(e.T_w_r);
    write_mat(e.T_w_r_extp);
    f << '\n';
  }
  for (const auto& e : pred_log_) {
    f << "pred," << e.pred_timestamp_ns << ',' << e.target_timestamp_ns << ','
      << e.sid << ',' << e.step << ','
      << e.pred_dx << ',' << e.pred_dy << ',' << e.pred_dyaw;
    write_mat(e.T_w_ref);
    write_mat(e.T_w_p_pred);
    f << '\n';
  }
  CLOG(INFO, "cbit.control") << "Wrote " << pose_log_.size() << " pose + "
                             << pred_log_.size() << " pred entries to " << log_path_;

  if (!input_log_.empty()) {
    const std::string input_log_path = log_path_ + ".inputs.csv";
    std::ofstream fi(input_log_path);
    if (!fi) {
      CLOG(WARNING, "cbit.control") << "Could not open input log: " << input_log_path;
      return;
    }
    fi << std::fixed << std::setprecision(9);

    size_t max_horizon = 0;
    for (const auto& e : input_log_) max_horizon = std::max(max_horizon, e.sequence.size());

    fi << "t0_ns,sid";
    for (int i = 0; i < 6; ++i) fi << ",loc_res_" << i;
    for (int i = 0; i < 6; ++i) fi << ",odom_vel_" << i;
    for (int i = 0; i < 3; ++i) fi << ",grav_vec_" << i;
    for (size_t s = 0; s < max_horizon; ++s)
      for (int i = 0; i < 6; ++i) fi << ",seq" << s << '_' << i;
    fi << '\n';

    for (const auto& e : input_log_) {
      fi << e.timestamp_ns << ',' << e.sid;
      for (int i = 0; i < 6; ++i) fi << ',' << e.loc_res[i];
      for (int i = 0; i < 6; ++i) fi << ',' << e.odom_vel[i];
      for (int i = 0; i < 3; ++i) fi << ',' << e.grav_vec[i];
      for (size_t s = 0; s < max_horizon; ++s) {
        for (int i = 0; i < 6; ++i) {
          if (s < e.sequence.size()) fi << ',' << e.sequence[s][i];
          else fi << ',';
        }
      }
      fi << '\n';
    }
    CLOG(INFO, "cbit.control") << "Wrote " << input_log_.size()
                               << " input snapshot entries to " << input_log_path;
  }
}

void BaseReferenceAdjustmentMPCPathTracker::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& curr_time) {

  auto& chain = robot_state.chain.ptr();

  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p_chain, T_w_v_odo, T_r_v_odo,
              curr_sid] = getChainInfo(*robot_state.chain.ptr());

  std::vector<double> p_rollout;
  for (int j = 1; j < mpcConfig->N + 1; j++) {
    p_rollout.push_back(state_p + j * mpcConfig->VF * mpcConfig->DT);
  }

  mpcConfig->reference_poses.clear();
  auto referenceInfo = generateHomotopyReference(p_rollout, chain, T_w_p*T_p_r_extp);
  std::vector<lgmath::se3::Transformation> local_reference_poses;

  for (size_t j = 0; j < referenceInfo.poses.size(); ++j) {
    const auto& Tf = referenceInfo.poses[j];

    const auto local = T_w_p.inverse() * Tf;
    mpcConfig->reference_poses.push_back(tf_to_global(local));
    local_reference_poses.push_back(local);

    CLOG(DEBUG, "cbit.control")
        << "Adding reference pose: " << tf_to_global(local);
  }

  // Save pre-correction reference poses for logging
  std::vector<lgmath::se3::Transformation> original_local_poses = local_reference_poses;

  if (error_predictor_) {
    PredictorInputSnapshot input_snapshot;
    const auto predicted_errors = error_predictor_->predictError(
        robot_state, curr_time, local_reference_poses,
        base_config_->apply_corrections, &input_snapshot);
    vis_->publishPredictedErrors(predicted_errors, stamp);
    vis_->publishNNInputsOutputs(input_snapshot, stamp);

    mpcConfig->reference_poses.clear();
    std::vector<lgmath::se3::Transformation> corrected_world_poses;
    for (size_t i = 0; i < local_reference_poses.size(); ++i) {
      mpcConfig->reference_poses.push_back(tf_to_global(local_reference_poses[i]));
      corrected_world_poses.push_back(T_w_p * local_reference_poses[i]);
    }

    vis_->publishPredictedRobotPath(corrected_world_poses, stamp);

    if (!log_path_.empty() && !original_local_poses.empty()) {
      const int64_t stamp_ns = static_cast<int64_t>(stamp);
      const lgmath::se3::Transformation T_w_r_now = T_w_p * T_p_r;
      const lgmath::se3::Transformation T_w_r_extp = T_w_p * T_p_r_extp;

      // Record robot pose every cycle for post-hoc interpolation.
      pose_log_.push_back({stamp_ns, T_w_r_now, T_w_r_extp});

      // Record the exact live numeric inputs predictError() built this cycle.
      if (input_snapshot.valid) {
        input_log_.push_back({
            stamp_ns,
            curr_sid,
            input_snapshot.loc_res,
            input_snapshot.odom_vel,
            input_snapshot.grav_vec,
            input_snapshot.sequence,
        });
      }

      const int N = static_cast<int>(original_local_poses.size());
      for (int j = 0; j < N && j < static_cast<int>(predicted_errors.size()); ++j) {
        const auto& pred = predicted_errors[j];
        pred_log_.push_back({
            stamp_ns,
            stamp_ns + static_cast<int64_t>(j + 1) * kPredictionStepNs,
            curr_sid,
            j + 1,
            pred[0], pred[1], pred[2],
            // Log pre-correction application poses for comparison post-hoc
            T_w_p * original_local_poses[j],
            T_w_p
        });
      }
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

  vis_->publishReferencePoses(referenceInfo.poses, stamp);
  vis_->publishLocalReferencePoses(original_local_poses, stamp);

  if (end_ind == 0)
    end_ind = 1;
  mpcConfig->eop_index = end_ind;

  mpcConfig->up_barrier_q  = referenceInfo.barrier_q_max;
  mpcConfig->low_barrier_q = referenceInfo.barrier_q_min;
}


}  // namespace vtr::path_planning
