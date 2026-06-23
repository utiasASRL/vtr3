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
      robot_state_{robot_state} {

  if(config->reference_adjustment_mode == ReferenceAdjustmentMode::ImageNeuralNetwork)
  {
    if(!config->reference_adjustment_model_path.empty())
    {
      // Initialize the image neural network
      image_error_predictor_ = std::make_shared<ImageErrorPredictorNetwork>(base_config_->reference_adjustment_model_path);
      // Sub to dependencies
      sig_img_sub_ = robot_state_->node->create_subscription<ImageMsg>(
          base_config_->sig_img_topic, rclcpp::QoS(1),
          [this](const ImageMsg::SharedPtr msg) { signalCallback(msg); });
      dpt_img_sub_ = robot_state_->node->create_subscription<ImageMsg>(
          base_config_->dpt_img_topic, rclcpp::QoS(1),
          [this](const ImageMsg::SharedPtr msg) { depthCallback(msg); });
      imu_sub_ = robot_state_->node->create_subscription<ImuMsg>(
          base_config_->imu_topic, rclcpp::QoS(1),
          [this](const ImuMsg::SharedPtr msg) { imuCallback(msg); });
      CLOG(INFO, "cbit.control") << "Initialized image neural network for reference adjustment.";
    }
    else
    {
      CLOG(WARNING, "cbit.control") << "Reference adjustment model path is empty. Image neural network will not be initialized.";
    }
  }
  else if (config->reference_adjustment_mode == ReferenceAdjustmentMode::NumericalNeuralNetwork)
  {
    if(!config->reference_adjustment_model_path.empty())
    {
      // Initialize the numerical error predictor
      //numerical_error_predictor_ = std::make_shared<NumericalErrorPredictorNetwork>();
      imu_sub_ = robot_state_->node->create_subscription<ImuMsg>(
          base_config_->imu_topic, rclcpp::QoS(1),
          [this](const ImuMsg::SharedPtr msg) { imuCallback(msg); });
      CLOG(INFO, "cbit.control") << "Initialized numerical neural network for reference adjustment.";
    }
    else
    {
      CLOG(WARNING, "cbit.control") << "Reference adjustment model path is empty. Numerical neural network will not be initialized.";
    }
  }
  else if (config->reference_adjustment_mode == ReferenceAdjustmentMode::HistoryBased)
  {
    CLOG(INFO, "cbit.control") << "Using history-based reference adjustment.";
  }
  else
  {
    CLOG(INFO, "cbit.control") << "No reference adjustment will be applied.";
  }
}

BaseReferenceAdjustmentMPCPathTracker::~BaseReferenceAdjustmentMPCPathTracker() {}

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

  if (base_config_->reference_adjustment_mode == ReferenceAdjustmentMode::HistoryBased) {
    CLOG(INFO, "cbit.control") << "Using history-based reference adjustment.";
  }
  else if (base_config_->reference_adjustment_mode == ReferenceAdjustmentMode::NumericalNeuralNetwork) {
    CLOG(INFO, "cbit.control") << "Using numerical neural network for reference adjustment.";
  }
  else if (base_config_->reference_adjustment_mode == ReferenceAdjustmentMode::ImageNeuralNetwork) {
    CLOG(INFO, "cbit.control") << "Using image neural network for reference adjustment.";
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

void BaseReferenceAdjustmentMPCPathTracker::signalCallback(const ImageMsg::SharedPtr msg) {
  sig_img_ = msg;
  CLOG(INFO, "cbit.control") << "Received signal image.";
}

void BaseReferenceAdjustmentMPCPathTracker::depthCallback(const ImageMsg::SharedPtr msg) {
  dpt_img_ = msg;
  CLOG(INFO, "cbit.control") << "Received depth image.";
}

void BaseReferenceAdjustmentMPCPathTracker::imuCallback(const ImuMsg::SharedPtr msg) {
  imu_msg_ = msg;
  CLOG(INFO, "cbit.control") << "Received IMU message.";
}

}  // namespace vtr::path_planning
