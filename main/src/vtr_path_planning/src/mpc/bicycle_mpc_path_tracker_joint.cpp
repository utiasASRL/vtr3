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
  config->follower_namespace = node->declare_parameter<std::string>(prefix + ".follower_namespace", config->follower_namespace);
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

  CLOG(DEBUG, "mpc.follower") << "Choosing " << config_->waypoint_selection << " option for waypoint selection!";

  using std::placeholders::_1;
  const auto follower_cmd_topic = config_->follower_namespace + "/cmd_vel";
  const auto follower_odom_topic = config_->follower_namespace + "/vtr/odometry";
  const auto follower_graph_topic = config_->follower_namespace + "/vtr/graph_state_srv";
  const auto follower_route_topic = config_->follower_namespace + "/vtr/following_route";
  CLOG(INFO, "mpc.follower") << "Requesting graph info from " << follower_graph_topic;
  CLOG(INFO, "mpc.follower") << "Listening for route on " << follower_route_topic;
  CLOG(INFO, "mpc.follower") << "Listening for pose on " << follower_odom_topic;
  CLOG(INFO, "mpc.follower") << "Sending leader velocities to " << follower_cmd_topic;
  CLOG(INFO, "mpc.follower") << "Target separation: " << config->following_offset;
  CLOG(INFO, "mpc.follower") << "Robot's wheelbase: " << config->wheelbase << "m";

  followerRouteSub_ = robot_state->node->create_subscription<RouteMsg>(follower_route_topic, rclcpp::SystemDefaultsQoS(), std::bind(&BicycleMPCJointPathTracker::onFollowerRoute, this, _1));
  followerOdomSub_ = robot_state->node->create_subscription<OdomMsg>(follower_odom_topic, rclcpp::SystemDefaultsQoS(), std::bind(&BicycleMPCJointPathTracker::onFollowerOdom, this, _1));
  followerCommandPub_ = robot_state->node->create_publisher<Command>(follower_cmd_topic, 10);
  leaderGraphSrv_ = robot_state->node->create_client<GraphStateSrv>("vtr/graph_state_srv");
  followerGraphSrv_ = robot_state->node->create_client<GraphStateSrv>(follower_graph_topic);

}

BicycleMPCJointPathTracker::~BicycleMPCJointPathTracker() {}

void BicycleMPCJointPathTracker::loadMPCConfig(
    CasadiBicycleMPCJoint::Config::Ptr mpc_config, const bool isReversing, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) { 
  BicycleMPCPathTracker::loadMPCConfig(mpc_config, isReversing, w_p_r_in_r, applied_vel);
  mpc_config->Q_dist = isReversing ? config_->r_q_dist : config_->f_q_dist;
  mpc_config->distance = config_->following_offset;
  mpc_config->distance_margin = config_->distance_margin;
  mpc_config->recovery = false;
}

CasadiMPC::Config::Ptr BicycleMPCJointPathTracker::getMPCConfig(const bool isReversing, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) {
  auto mpcConfig = std::make_shared<CasadiBicycleMPCJoint::Config>();
  loadMPCConfig(mpcConfig, isReversing, w_p_r_in_r, applied_vel);
  return mpcConfig;
}

bool BicycleMPCJointPathTracker::isMPCStateValid(CasadiMPC::Config::Ptr, const tactic::Timestamp& curr_time){
  return true;
}

void BicycleMPCJointPathTracker::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& curr_time) {

  // Leader is handled as normal
  BaseMPCPathTracker::loadMPCPath(mpcConfig, T_w_p, T_p_r_extp, state_p, robot_state, curr_time);
  std::vector<lgmath::se3::Transformation> leader_world_poses;
  for (const auto& Tf : mpcConfig->reference_poses) {
    const auto Tf_g = Tf.get_elements();
    leader_world_poses.push_back(T_w_p * tf_from_global(Tf_g[0], Tf_g[1], Tf_g[2]));
  }

  auto joint_mpc_config = std::dynamic_pointer_cast<CasadiBicycleMPCJoint::Config>(mpcConfig);
  
  auto& chain = robot_state.chain.ptr();

  auto T_w_f_extp = T_lw_fw_ * T_fw_f_;
  if (config_->extrapolate_robot_pose) {
    auto dt = static_cast<double>(curr_time - follower_stamp_) * 1e-9;
    Eigen::Vector<double, 6> w_p_r_in_r;
    w_p_r_in_r << -follower_vel_(0), 0, 0, 0, 0, -follower_vel_(1);
    CLOG(DEBUG, "mpc.follower")
        << "Robot velocity Used for Follower Extrapolation: " << -w_p_r_in_r.transpose()
        << " dt: " << dt << std::endl;
    Eigen::Vector<double, 6> xi_p_r_in_r(-dt * w_p_r_in_r);
    T_w_f_extp = T_w_f_extp * tactic::EdgeTransform(xi_p_r_in_r);
  }

  const auto T_f_l = (T_w_p * T_p_r_extp).inverse() * T_w_f_extp;
  CLOG(DEBUG, "mpc.follower") << "TF to leader:\n" <<  T_f_l;
  const Eigen::Vector<double, 3> dist = T_f_l.r_ab_inb();
  CLOG(DEBUG, "mpc.follower") << "Dist to leader:\n" << dist.head<2>().norm();
  

  joint_mpc_config->follower_reference_poses.clear();
  const auto [_, follower_state_p] = findRobotP(T_w_f_extp, chain);
  auto referenceInfo = [&](){
    if(config_->waypoint_selection == "euclidean") {
      const auto final_leader_p_value = state_p + mpcConfig->N * mpcConfig->VF * mpcConfig->DT;
      return generateFollowerReferencePosesEuclidean(leader_world_poses, final_leader_p_value, chain, follower_state_p, joint_mpc_config->distance);
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
    joint_mpc_config->follower_reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
    CLOG(DEBUG, "mpc.follower.target") << "Follower target " << tf_to_global(T_w_p.inverse() *  Tf);
  }
  vis_->publishLeaderRollout(leader_world_poses, curr_time, mpcConfig->DT);
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

  // for (int i = 0; i < result["pose_follower"].columns(); i++) {
  //   const auto& pose_i = result["pose_follower"](casadi::Slice(), i).get_elements();
  //   mpc_poses.push_back(std::make_pair(curr_time + i*mpcConfig->DT*1e9, T_w_p * tf_from_global(pose_i[0], pose_i[1], pose_i[2])));
  // }

  const auto& mpc_vel_vec = result["vel_follower"](casadi::Slice(), 0).get_elements();

  Command followerCommand;
  followerCommand.linear.x = mpc_vel_vec[0];
  followerCommand.angular.z = mpc_vel_vec[1];

  // // Get all the mpc velocities
  // for (int i = 0; i < result["vel"].columns(); i++) {
  //   const auto& vel_i = result["vel"](casadi::Slice(), i).get_elements();
  //   mpc_velocities.emplace_back(vel_i[0], vel_i[1]);
  //   CLOG(DEBUG, "cbit.control")
  //       << "MPC velocity at step " << i << ": " << mpc_velocities.back().transpose();
  // }

  followerCommandPub_->publish(followerCommand);

  return result;
}


void BicycleMPCJointPathTracker::onFollowerRoute(const RouteMsg::SharedPtr route) {
  if (robot_state_->chain.valid() && robot_state_->chain->sequence().size() > 0 && route->ids.size() > 0 && route->ids.front() != follower_root_) { 

    //TODO Figure out the best time to check if we are using the same graph for leader and follower. 
    // leaderGraphSrv_->async_send_request()
    follower_root_ = route->ids.front();
    CLOG(INFO, "mpc.follower") << "Updated follower's root to: " << follower_root_;
    const auto leader_root = robot_state_->chain->sequence().front();
    CLOG(INFO, "mpc.follower") << "Leader's root is: " << leader_root;

    auto connected = graph_->dijkstraSearch(leader_root, follower_root_);
    
    T_lw_fw_ = pose_graph::eval::ComposeTfAccumulator(connected->beginDfs(leader_root), connected->end(), tactic::EdgeTransform(true));    
    CLOG(INFO, "mpc.follower") << "Set relative transform to : " << T_lw_fw_;
  } else {
    CLOG(WARNING, "mpc.follower") << "Follower route received but robot state chain is not valid or empty. Cannot update follower root.";
  }
}

void BicycleMPCJointPathTracker::onFollowerOdom(const OdomMsg::SharedPtr leader_pose) {
  using namespace vtr::common::conversions;
  T_fw_f_ = tfFromPoseMessage(leader_pose->pose.pose);
  follower_vel_ << leader_pose->twist.twist.linear.x, leader_pose->twist.twist.angular.z;
  follower_stamp_ = rclcpp::Time(leader_pose->header.stamp).nanoseconds();
}


}  // namespace vtr::path_planning
