// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \author Luka Antonyshyn, Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/bicycle_mpc_path_tracker_joint.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

namespace vtr::path_planning {

void BicycleMPCJointPathTracker::Config::loadConfig(BicycleMPCJointPathTracker::Config::Ptr config, 
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix) {

  // MPC Configs:

  // Follower params
  config->leader_namespace = node->declare_parameter<std::string>(prefix + ".leader_namespace", config->leader_namespace);
  config->following_offset = node->declare_parameter<double>(prefix + ".follow_distance", config->following_offset);
  config->distance_margin = node->declare_parameter<double>(prefix + ".distance_margin", config->distance_margin);

  config->f_q_dist = node->declare_parameter<double>(prefix + ".mpc.forward.q_dist", config->f_q_dist);
  config->r_q_dist = node->declare_parameter<double>(prefix + ".mpc.reverse.q_dist", config->r_q_dist);

  // Waypoint selection
  config->waypoint_selection = node->declare_parameter<std::string>(prefix + ".waypoint_selection", config->waypoint_selection);
}

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto BicycleMPCJointPathTracker::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  auto base_config = std::static_pointer_cast<BicycleMPCPathTracker::Config>(config);
  *base_config =  *BicycleMPCPathTracker::Config::fromROS(node, prefix);
  loadConfig(config, node, prefix);

  CLOG(DEBUG, "cbit.control") << "Bicycle Tracker MPC forward costs: "
      << ", q_dist: " << config->f_q_dist;

  CLOG(DEBUG, "cbit.control") << "Bicycle Tracker MPC reverse costs: "
      << ", q_dist: " << config->r_q_dist;

  return config;
}

// Declare class as inherited from the BasePathPlanner
BicycleMPCJointPathTracker::BicycleMPCJointPathTracker(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph,
                               const Callback::Ptr& callback)
    : BicycleMPCPathTracker(config, robot_state, graph, callback), config_(config), solver_{config_->mpc_verbosity}, graph_{graph}, robot_state_{robot_state} {

  vis_ = std::make_shared<VisualizationUtils>(robot_state->node.ptr());
  lastLeaderCommand_ << 0, 0;

  CLOG(DEBUG, "mpc.follower") << "Choosing " << config_->waypoint_selection << " option for waypoint selection!";

  using std::placeholders::_1;
  const auto leader_cmd_topic = config_->leader_namespace + "/cmd_vel";
  const auto leader_odom_topic = config_->leader_namespace + "/vtr/odometry";
  const auto leader_graph_topic = config_->leader_namespace + "/vtr/graph_state_srv";
  const auto leader_route_service = config_->leader_namespace + "/vtr/following_route_srv";
  CLOG(INFO, "mpc.follower") << "Requesting graph info from " << leader_graph_topic;
  CLOG(INFO, "mpc.follower") << "Listening for route on " << leader_route_service;
  CLOG(INFO, "mpc.follower") << "Listening for pose on " << leader_odom_topic;
  CLOG(INFO, "mpc.follower") << "Sending leader velocities to " << leader_cmd_topic;
  CLOG(INFO, "mpc.follower") << "Target separation: " << config->following_offset;
  CLOG(INFO, "mpc.follower") << "Robot's wheelbase: " << config->wheelbase << "m";

  leaderRouteSrv_ = robot_state->node->create_client<FollowingRouteSrv>(leader_route_service);
  leaderOdomSub_ = robot_state->node->create_subscription<OdomMsg>(leader_odom_topic, rclcpp::SystemDefaultsQoS(), std::bind(&BicycleMPCJointPathTracker::onLeaderOdom, this, _1));
  leaderCommandPub_ = robot_state->node->create_publisher<Command>(leader_cmd_topic, 10);
  estimatedDistancePub_ = robot_state->node->create_publisher<FloatMsg>("estimated_leader_distance", 10);
 
}

BicycleMPCJointPathTracker::~BicycleMPCJointPathTracker() {}

void BicycleMPCJointPathTracker::loadMPCConfig(
    CasadiBicycleMPCJoint::Config::Ptr mpc_config, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel, const bool isReversing) { 
  BicycleMPCPathTracker::loadMPCConfig(mpc_config, w_p_r_in_r, applied_vel, isReversing);
  mpc_config->Q_dist = isReversing ? config_->r_q_dist : config_->f_q_dist;
  mpc_config->distance = config_->following_offset;
  mpc_config->distance_margin = config_->distance_margin;
  mpc_config->recovery = false;
}

CasadiMPC::Config::Ptr BicycleMPCJointPathTracker::getMPCConfig(Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel, const bool isReversing) {
  auto mpcConfig = std::make_shared<CasadiBicycleMPCJoint::Config>();
  loadMPCConfig(mpcConfig, w_p_r_in_r, applied_vel, isReversing);
  return mpcConfig;
}

bool BicycleMPCJointPathTracker::isMPCStateValid(CasadiMPC::Config::Ptr, const tactic::Timestamp& curr_time){
  if (leader_root_ == tactic::VertexId::Invalid()){
    if (!hasRequestedLeaderRoute_ || robot_state_->node->now() - requestTime_ > rclcpp::Duration(1, 0)){
      CLOG(INFO, "cbit.control") << "Leader root not yet set. Calling service";
      auto request = std::make_shared<FollowingRouteSrv::Request>();
      leaderRouteSrv_->async_send_request(request, std::bind(&BicycleMPCJointPathTracker::leaderRouteCallback, this, std::placeholders::_1));
      hasRequestedLeaderRoute_ = true;
      requestTime_ = robot_state_->node->now();
    }
    return false;
  }
  
  if (leader_stamp_ == 0) {
    CLOG_EVERY_N(1, WARNING, "cbit.control") << "Leader has received no odom info from the leader yet. Stopping";
    return false;
  }

  const auto delta_t = curr_time - leader_stamp_;
  if (delta_t > 1e9) {
    CLOG_EVERY_N(1, WARNING,"cbit.control") << "Follower has received no odom from the leader in more than 1 second. Stopping\n Delay: "
           << delta_t / 1e9;
    return false;
  }
  return true;
}

void BicycleMPCJointPathTracker::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double follower_state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& curr_time) {

  auto joint_mpc_config = std::dynamic_pointer_cast<CasadiBicycleMPCJoint::Config>(mpcConfig);
  
  auto& chain = robot_state.chain.ptr();

  auto T_w_l_extp = T_fw_lw_ * T_lw_l_;
  if (config_->extrapolate_robot_pose) {
    auto dt = static_cast<double>(curr_time - leader_stamp_) * 1e-9;
    Eigen::Vector<double, 6> w_p_r_in_r;
    w_p_r_in_r << leader_vel_(0), leader_vel_(1), leader_vel_(2), leader_vel_(3), leader_vel_(4), leader_vel_(5);
    CLOG(DEBUG, "mpc.follower")
        << "Robot velocity Used for leader Extrapolation: " << -w_p_r_in_r
        << " dt: " << dt << std::endl;
    Eigen::Vector<double, 6> xi_p_r_in_r(-dt * w_p_r_in_r);
    T_w_l_extp = T_w_l_extp * tactic::EdgeTransform(xi_p_r_in_r);
  }

  joint_mpc_config->T0_leader = tf_to_global(T_w_p.inverse() * T_w_l_extp);
  joint_mpc_config->previous_vel_leader = {-leader_vel_(0), lastLeaderCommand_(1)};

  CLOG(DEBUG, "mpc.follower") << "Leader location: " << joint_mpc_config->T0_leader;
  CLOG(DEBUG, "mpc.follower") << "Follower location: " << joint_mpc_config->T0;

  const auto [_, leader_state_p] = findRobotP(T_w_l_extp, chain);
  auto referenceInfoLeader = [&](){
    std::vector<double> p_rollout;
    for(int j = 1; j < mpcConfig->N+1; j++){
      p_rollout.push_back(leader_state_p + j*mpcConfig->VF*mpcConfig->DT);
    }
    return generateHomotopyReference(p_rollout, chain);
  }();

  for(const auto& Tf : referenceInfoLeader.poses) {
    joint_mpc_config->leader_reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
    CLOG(DEBUG, "mpc.follower.target") << "Leader target " << tf_to_global(T_w_p.inverse() *  Tf);
  }

  const auto T_f_l = T_w_l_extp.inverse() * T_w_p * T_p_r_extp;
  CLOG(DEBUG, "mpc.follower") << "TF to leader:\n" <<  T_f_l;
  const Eigen::Vector<double, 3> dist_vec = T_f_l.r_ab_inb();
  FloatMsg internal_dist;
  internal_dist.data = dist_vec.head<2>().norm();
  CLOG(DEBUG, "mpc.follower") << "Displacement to leader:\n" << dist_vec;
  CLOG(DEBUG, "mpc.follower") << "Dist to leader: " << internal_dist.data << " at stamp " << curr_time;
  estimatedDistancePub_->publish(internal_dist);  

  joint_mpc_config->reference_poses.clear();
  auto referenceInfo = [&](){
    if(config_->waypoint_selection == "euclidean") {
      const auto final_leader_p_value = leader_state_p + mpcConfig->N * mpcConfig->VF * mpcConfig->DT;
      const auto initial_leader_p_value = leader_state_p;
      std::vector<double> max_p_vals;
      for (float i = 0; i < mpcConfig->N; i++) {
        max_p_vals.push_back(initial_leader_p_value + i / (mpcConfig->N - 1) * (final_leader_p_value - initial_leader_p_value));
      }
      return generateFollowerReferencePosesEuclidean(referenceInfoLeader.poses, max_p_vals, chain, follower_state_p, joint_mpc_config->distance);
    } else {
      CLOG_IF(config_->waypoint_selection ==  "arclength", WARNING, "mpc.follower") << "Arclength not implemented yet for bicycle!";

      std::vector<double> p_rollout;
      for(int j = 1; j < mpcConfig->N+1; j++){
        p_rollout.push_back(follower_state_p + j*mpcConfig->VF*mpcConfig->DT);
      }
      return generateHomotopyReference(p_rollout, chain);
    }
  }();
  
  for(const auto& Tf : referenceInfo.poses) {
    joint_mpc_config->reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
    CLOG(DEBUG, "mpc.follower.target") << "Follower target " << tf_to_global(T_w_p.inverse() *  Tf);
  }
  vis_->publishLeaderRollout(referenceInfoLeader.poses, curr_time, mpcConfig->DT);
}

std::map<std::string, casadi::DM> BicycleMPCJointPathTracker::callSolver(CasadiMPC::Config::Ptr config) {
  std::map<std::string, casadi::DM> result;

  try {
    CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
    result = solver_.solve(*config);
  } catch (std::exception& e) {
      CLOG(WARNING, "cbit.control")
          << "casadi failed! " << e.what();

      throw e;
  }

  // for (int i = 0; i < result["pose_leader"].columns(); i++) {
  //   const auto& pose_i = result["pose_leader"](casadi::Slice(), i).get_elements();
  //   mpc_poses.push_back(std::make_pair(curr_time + i*mpcConfig->DT*1e9, T_w_p * tf_from_global(pose_i[0], pose_i[1], pose_i[2])));
  // }

  const auto& mpc_vel_vec = result["vel_leader"](casadi::Slice(), 0).get_elements();

  Command leaderCommand;
  leaderCommand.linear.x = mpc_vel_vec[0];
  leaderCommand.angular.z = mpc_vel_vec[1];
  lastLeaderCommand_ << mpc_vel_vec[0], mpc_vel_vec[1];

  // // Get all the mpc velocities
  // for (int i = 0; i < result["vel"].columns(); i++) {
  //   const auto& vel_i = result["vel"](casadi::Slice(), i).get_elements();
  //   mpc_velocities.emplace_back(vel_i[0], vel_i[1]);
  //   CLOG(DEBUG, "cbit.control")
  //       << "MPC velocity at step " << i << ": " << mpc_velocities.back().transpose();
  // }

  leaderCommandPub_->publish(leaderCommand);

  return result;
}

void BicycleMPCJointPathTracker::setRunning(const bool running) {
  if (!running) {
    leader_root_ = tactic::VertexId::Invalid();
  }
  BicycleMPCPathTracker::setRunning(running);
}

void BicycleMPCJointPathTracker::leaderRouteCallback(const rclcpp::Client<FollowingRouteSrv>::SharedFuture future) {
  auto result = future.get();
  auto route = result->following_route;
  if (robot_state_->chain.valid() && robot_state_->chain->sequence().size() > 0 && route.ids.size() > 0) { 

    const auto follower_root = robot_state_->chain->sequence().front();
    CLOG(INFO, "mpc.follower") << "Follower's root is: " << follower_root;
    leader_root_ = tactic::VertexId(0, 0); 
    CLOG(INFO, "mpc.follower") << "Leader's root is: " << leader_root_;

    auto connected = graph_->dijkstraSearch(follower_root, leader_root_);
    
    T_fw_lw_ = pose_graph::eval::ComposeTfAccumulator(connected->beginDfs(follower_root), connected->end(), tactic::EdgeTransform(true));    
    CLOG(INFO, "mpc.follower") << "Set relative transform to : " << T_fw_lw_;
  } else {
    CLOG(WARNING, "mpc.follower") << "Leader route received but robot state chain is not valid or empty. Cannot update leader root.";
  }
}

void BicycleMPCJointPathTracker::onLeaderOdom(const OdomMsg::SharedPtr leader_pose) {
  using namespace vtr::common::conversions;
  T_lw_l_ = tfFromPoseMessage(leader_pose->pose.pose);
  leader_vel_ << -leader_pose->twist.twist.linear.x,  -leader_pose->twist.twist.linear.y,  -leader_pose->twist.twist.linear.x,
                   -leader_pose->twist.twist.angular.x, -leader_pose->twist.twist.angular.y, -leader_pose->twist.twist.angular.z;
  leader_stamp_ = rclcpp::Time(leader_pose->header.stamp).nanoseconds();
  CLOG(DEBUG, "mpc.follower") << "Received odom from leader at time " << rclcpp::Time(leader_pose->header.stamp).seconds();
}


}  // namespace vtr::path_planning
