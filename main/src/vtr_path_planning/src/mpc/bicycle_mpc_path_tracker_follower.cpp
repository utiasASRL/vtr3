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

#include "vtr_path_planning/mpc/bicycle_mpc_path_tracker_follower.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vtr_path_planning/cbit/utils.hpp>

namespace vtr::path_planning {

void BicycleMPCPathTrackerFollower::Config::loadConfig(BicycleMPCPathTrackerFollower::Config::Ptr config, 
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix) {

  // MPC Configs:
  // PID PARAMETERS
  config->kp = node->declare_parameter<double>(prefix + ".longitudinal_control.kp", config->kp);
  config->ki = node->declare_parameter<double>(prefix + ".longitudinal_control.ki", config->ki);
  config->kd = node->declare_parameter<double>(prefix + ".longitudinal_control.kd", config->kd);

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
auto BicycleMPCPathTrackerFollower::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
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
BicycleMPCPathTrackerFollower::BicycleMPCPathTrackerFollower(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph,
                               const Callback::Ptr& callback)
    : BicycleMPCPathTracker(config, robot_state, graph, callback), config_(config), solver_{config_->mpc_verbosity}, graph_{graph}, robot_state_{robot_state} {

  vis_ = std::make_shared<VisualizationUtils>(robot_state->node.ptr());

  CLOG(DEBUG, "mpc.follower") << "Choosing " << config_->waypoint_selection << " option for waypoint selection!";

  using std::placeholders::_1;
  const auto leader_path_topic = config_->leader_namespace + "/vtr/mpc_prediction";
  const auto leader_graph_topic = config_->leader_namespace + "/vtr/graph_state_srv";
  const auto leader_route_topic = config_->leader_namespace + "/vtr/following_route";
  CLOG(INFO, "mpc.follower") << "Listening for MPC rollouts on " << leader_path_topic;
  CLOG(INFO, "mpc.follower") << "Requesting graph info from " << leader_graph_topic;
  CLOG(INFO, "mpc.follower") << "Listening for route on " << leader_route_topic;
  CLOG(INFO, "mpc.follower") << "Target separation: " << config->following_offset;
  CLOG(INFO, "mpc.follower") << "Robot's wheelbase: " << config->wheelbase << "m";

  leaderRolloutSub_ = robot_state->node->create_subscription<PathMsg>(leader_path_topic, rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&BicycleMPCPathTrackerFollower::onLeaderPath, this, _1));
  leaderRouteSub_ = robot_state->node->create_subscription<RouteMsg>(leader_route_topic, rclcpp::SystemDefaultsQoS(), std::bind(&BicycleMPCPathTrackerFollower::onLeaderRoute, this, _1));

  leaderGraphSrv_ = robot_state->node->create_client<GraphStateSrv>(leader_graph_topic);
  followerGraphSrv_ = robot_state->node->create_client<GraphStateSrv>("vtr/graph_state_srv");

  leaderDistanceSub_ = robot_state->node->create_subscription<FloatMsg>("leader_distance", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&BicycleMPCPathTrackerFollower::onLeaderDist, this, _1));
}


BicycleMPCPathTrackerFollower::~BicycleMPCPathTrackerFollower() {}
void BicycleMPCPathTrackerFollower::loadMPCConfig(
    CasadiBicycleMPCFollower::Config::Ptr mpc_config, const bool isReversing, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) { 
  BicycleMPCPathTracker::loadMPCConfig(mpc_config, isReversing, w_p_r_in_r, applied_vel);
  mpc_config->Q_dist = isReversing ? config_->r_q_dist : config_->f_q_dist;
  mpc_config->distance = config_->following_offset;
  mpc_config->distance_margin = config_->distance_margin;
  mpc_config->recovery = false;
}

CasadiMPC::Config::Ptr BicycleMPCPathTrackerFollower::getMPCConfig(const bool isReversing, Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) {
  auto mpcConfig = std::make_shared<CasadiBicycleMPCFollower::Config>();
  loadMPCConfig(mpcConfig, isReversing, w_p_r_in_r, applied_vel);
  return mpcConfig;
}

bool BicycleMPCPathTrackerFollower::isMPCStateValid(CasadiMPC::Config::Ptr, const tactic::Timestamp& curr_time){
  if (recentLeaderPath_ == nullptr) {
    CLOG_EVERY_N(1, WARNING, "cbit.control") << "Follower has received no path from the leader yet. Stopping";
    return false;
  }

  const auto leader_path_time = rclcpp::Time(recentLeaderPath_->header.stamp).nanoseconds();
  const auto delta_t = curr_time - leader_path_time;
  if (delta_t > 1e9) {
    CLOG_EVERY_N(1, WARNING,"cbit.control") << "Follower has received no path from the leader in more than 1 second. Stopping\n Delay: "
           << delta_t / 1e9;
    return false;
  }

  return true;
}

void BicycleMPCPathTrackerFollower::loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                         const lgmath::se3::Transformation& T_p_r_extp,
                         const double state_p,
                         RobotState& robot_state,
                         const tactic::Timestamp& curr_time) {

  
  auto follower_mpc_config = std::dynamic_pointer_cast<CasadiBicycleMPCFollower::Config>(mpcConfig);
  
  auto& chain = robot_state.chain.ptr();
  follower_mpc_config->leader_reference_poses.clear();

  // mpcConfig->VF = abs(leader_vel_(0));
  if (follower_mpc_config->reversing) {
    CLOG(DEBUG, "mpc.follower") << "Reversing!";
  }
  if (config_->waypoint_selection == "external_dist") {
    const float distance = recentLeaderDist_->data;
    const double error = distance - config_->following_offset;
    errorIntegrator += error;
    mpcConfig->VF = config_->kp * error + config_->ki * errorIntegrator + config_->kd * (error - lastError_);
    lastError_ = error;
    CLOG(DEBUG, "mpc.follower.pid") << "Requested forward speed " << mpcConfig->VF;

    //TODO revert to max and min
    // mpcConfig->vel_max = {mpcConfig.VF, config_->max_ang_vel};
    // mpcConfig->vel_min = {mpcConfig.VF, -config_->max_ang_vel};
    follower_mpc_config->lin_acc_max = 1000;
  }

  std::vector<lgmath::se3::Transformation> leader_world_poses;
  std::vector<double> leader_p_values;
  const auto leaderPath_copy = *leaderPathInterp_;
  const auto T_f_l = (T_w_p * T_p_r_extp).inverse() * T_fw_lw_ * leaderPath_copy.at(curr_time);
  CLOG(DEBUG, "mpc.follower") << "TF to leader:\n" <<  T_f_l;
  const Eigen::Vector<double, 3> dist = T_f_l.r_ab_inb();
  CLOG(DEBUG, "mpc.follower") << "Displacement to leader:\n" << dist;
  CLOG(DEBUG, "mpc.follower") << "Dist to leader:\n" << dist.head<2>().norm();

  if (dist.norm() > 1.0) {
    CLOG(DEBUG, "mpc.follower") << "Current Leader Pose\n" << leaderPath_copy.at(curr_time);
    CLOG(DEBUG, "mpc.follower") << "Current Follower Path Loc\n" << T_w_p;
    CLOG(DEBUG, "mpc.follower") << "New extrapolated pose:\n"  << T_p_r_extp;
  }
  
  for (int i = 0; i < mpcConfig->N; i++){
    const auto T_w_lp = T_fw_lw_ * leaderPath_copy.at(curr_time + (1+i) * mpcConfig->DT * 1e9);
    follower_mpc_config->leader_reference_poses.push_back(tf_to_global(T_w_p.inverse() *  T_w_lp));
    leader_world_poses.push_back(T_w_lp);

    if (config_->waypoint_selection == "euclidean") {
      CLOG(DEBUG, "mpc.follower") << "Finding p value for leader";
      leader_p_values.push_back(findRobotP(T_w_lp, chain).second);
    }
    CLOG(DEBUG, "mpc.follower.target") << "Leader Target " << tf_to_global(T_w_p.inverse() *  T_w_lp);
  }

  mpcConfig->reference_poses.clear();
  auto referenceInfo = [&](){
    if(config_->waypoint_selection == "euclidean") {
      return generateFollowerReferencePosesEuclidean(leader_world_poses, leader_p_values, chain, state_p, follower_mpc_config->distance);
    } else {
      CLOG_IF(config_->waypoint_selection ==  "arclength", WARNING, "mpc.follower") << "Arclength not implemented yet for bicycle!";

      std::vector<double> p_rollout;
      for(int j = 1; j < mpcConfig->N+1; j++){
        p_rollout.push_back(state_p + j*mpcConfig->VF*mpcConfig->DT);
      }
      return generateHomotopyReference(p_rollout, chain);
    }
  }();
  
  for(const auto& Tf : referenceInfo.poses) {
    mpcConfig->reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
    CLOG(DEBUG, "mpc.follower") << "Target " << tf_to_global(T_w_p.inverse() *  Tf);
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
    if (end_ind < 0 && dist < config_->end_of_path_distance_threshold) {
      end_ind = i-1;
      weighting = (float) end_ind / mpcConfig->N;
      CLOG(DEBUG, "cbit.control") << "Detected end of path. Setting cost of EoP poses to: " << weighting;
    }
      
    mpcConfig->cost_weights.push_back(weighting);
    last_pose = curr_pose;
  }
  vis_->publishReferencePoses(referenceInfo.poses);
  vis_->publishLeaderRollout(leader_world_poses, curr_time, mpcConfig->DT);

  mpcConfig->eop_index = end_ind;
  mpcConfig->up_barrier_q  = referenceInfo.barrier_q_max;
  mpcConfig->low_barrier_q = referenceInfo.barrier_q_min;
}

void BicycleMPCPathTrackerFollower::onLeaderPath(const PathMsg::SharedPtr path) {
  using namespace vtr::common::conversions;
  
  recentLeaderPath_ = path;

  //reconstruct velocity
  if (path->poses.size() > 1) {
    const Transformation T_w_p0 = tfFromPoseMessage(path->poses[0].pose);
    const Transformation T_w_p1 =  tfFromPoseMessage(path->poses[1].pose);
    const auto dt = rclcpp::Time(path->poses[1].header.stamp) - rclcpp::Time(path->poses[0].header.stamp);
    auto vel = (T_w_p0.inverse() * T_w_p1).vec() / dt.seconds();
    if (vel(0, 0) > 1.0) {
      CLOG(WARNING, "mpc.follower") << "Erroneous velocity " << vel << " capped to nominal forward speed. DT=" << dt.seconds();
      leader_vel_ << config_->forward_vel, 0.0;
    } else {
      leader_vel_ << vel(0, 0), vel(5, 0);
    }
    CLOG(DEBUG, "mpc.follower") << "Estimated leader velo: " << leader_vel_;
  } 

  leaderPathInterp_ = std::make_shared<const PathInterpolator>(path);
}

void BicycleMPCPathTrackerFollower::onLeaderRoute(const RouteMsg::SharedPtr route) {
  if (robot_state_->chain.valid() && robot_state_->chain->sequence().size() > 0 && route->ids.size() > 0 && route->ids.front() != leader_root_) { 

    //TODO Figure out the best time to check if we are using the same graph for leader and follower. 
    // leaderGraphSrv_->async_send_request()
    leader_root_ = route->ids.front();
    CLOG(INFO, "mpc.follower") << "Updated leader's root to: " << leader_root_;
    const auto follower_root = robot_state_->chain->sequence().front();
    CLOG(INFO, "mpc.follower") << "Follower's root to: " << follower_root;

    auto connected = graph_->dijkstraSearch(follower_root, leader_root_);
    
    T_fw_lw_ = pose_graph::eval::ComposeTfAccumulator(connected->beginDfs(follower_root), connected->end(), tactic::EdgeTransform(true));    
    CLOG(INFO, "mpc.follower") << "Set relative transform to : " << T_fw_lw_;

  }
  else{
    CLOG(WARNING, "mpc.follower") << "Leader route received but robot state chain is not valid or empty. Cannot update leader root.";
  }
}

std::map<std::string, casadi::DM> BicycleMPCPathTrackerFollower::callSolver(CasadiMPC::Config::Ptr config) {
  std::map<std::string, casadi::DM> result;

  try {
    CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
    result = solver_.solve(*config);
    CLOG(INFO, "cbit.control") << "Solver called";
  } catch (std::exception& e) {
      CLOG(WARNING, "cbit.control")
          << "casadi failed! " << e.what();
      throw e;
  }
  return result;
}

void BicycleMPCPathTrackerFollower::onLeaderDist(const FloatMsg::SharedPtr distance) {
  recentLeaderDist_ = distance;
}

}  // namespace vtr::path_planning
