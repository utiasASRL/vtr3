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

namespace {
// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Vector2d saturateVel(const Eigen::Vector2d& applied_vel, double v_lim, double w_lim) {
  return {std::clamp(applied_vel(0), -v_lim, v_lim), std::clamp(applied_vel(1), -w_lim, w_lim)};
}
}

// Configure the class as a ROS2 node, get configurations from the ros parameter server
auto BicycleMPCPathTrackerFollower::Config::fromROS(const rclcpp::Node::SharedPtr& node, const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();

  auto base_config = std::static_pointer_cast<BasePathPlanner::Config>(config);
  *base_config =  *BasePathPlanner::Config::fromROS(node, prefix);

  // MPC Configs:
  // SPEED SCHEDULER PARAMETERS
  config->planar_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.planar_curv_weight", config->planar_curv_weight);
  config->profile_curv_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.profile_curv_weight", config->profile_curv_weight);
  config->eop_weight = node->declare_parameter<double>(prefix + ".speed_scheduler.eop_weight", config->eop_weight);
  config->min_vel = node->declare_parameter<double>(prefix + ".speed_scheduler.min_vel", config->min_vel);

  // Follower params
  config->leader_namespace = node->declare_parameter<std::string>(prefix + ".leader_namespace", config->leader_namespace);
  config->following_offset = node->declare_parameter<double>(prefix + ".follow_distance", config->following_offset);
  config->distance_margin = node->declare_parameter<double>(prefix + ".distance_margin", config->distance_margin);

  // Waypoint selection
  config->waypoint_selection = node->declare_parameter<std::string>(prefix + ".waypoint_selection", config->waypoint_selection);

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
  config->turning_radius = node->declare_parameter<double>(prefix + ".mpc.turning_radius", config->turning_radius);
  config->wheelbase = node->declare_parameter<double>(prefix + ".mpc.wheelbase", config->wheelbase);
  
  // MPC COST PARAMETERS
  config->q_x = node->declare_parameter<double>(prefix + ".mpc.q_x", config->q_x);
  config->q_y = node->declare_parameter<double>(prefix + ".mpc.q_y", config->q_y);
  config->q_th = node->declare_parameter<double>(prefix + ".mpc.q_th", config->q_th);
  config->r1 = node->declare_parameter<double>(prefix + ".mpc.r1", config->r1);
  config->r2 = node->declare_parameter<double>(prefix + ".mpc.r2", config->r2);
  config->racc1 = node->declare_parameter<double>(prefix + ".mpc.racc1", config->racc1);
  config->racc2 = node->declare_parameter<double>(prefix + ".mpc.racc2", config->racc2);
  config->q_f = node->declare_parameter<double>(prefix + ".mpc.q_f", config->q_f);
  config->q_dist = node->declare_parameter<double>(prefix + ".mpc.q_dist", config->q_dist);
  CLOG(INFO, "cbit.control") << "The config is: Q_x " << config->q_x << " q_y: " << config->q_y<< " q_th: " << config->q_th<< " r1: " << config->r1<< " r2: " << config->r2<< "acc_r1: " << config->racc1<< " acc_r2: " << config->racc2;

  // MISC
  config->command_history_length = node->declare_parameter<int>(prefix + ".mpc.command_history_length", config->command_history_length);

  return config;
}


// Declare class as inherited from the BasePathPlanner
BicycleMPCPathTrackerFollower::BicycleMPCPathTrackerFollower(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const tactic::GraphBase::Ptr& graph,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, graph, callback), config_(config), solver_{config_->mpc_verbosity}, graph_{graph}, robot_state_{robot_state} {
  applied_vel_ << 0,
                  0;
  vel_history.reserve(config_->command_history_length);
  for (int i = 0; i < config_->command_history_length; i++)
  {
    vel_history.push_back(applied_vel_);
  }

  vis_ = std::make_shared<VisualizationUtils>(robot_state->node.ptr());

  CLOG(DEBUG, "mpc.follower") << "Choosing " << config_->waypoint_selection << " option for waypoint selection!";


  using std::placeholders::_1;
  const auto leader_path_topic = config_->leader_namespace + "/vtr/mpc_prediction";
  const auto leader_graph_topic = config_->leader_namespace + "/vtr/graph_state_srv";
  const auto leader_route_topic = config_->leader_namespace + "/vtr/following_route";
  CLOG(INFO, "mpc.follower") << "Listening for MPC rollouts on " << leader_path_topic;
  CLOG(INFO, "mpc.follower") << "Requesting graph info from " << leader_graph_topic;
  CLOG(INFO, "mpc.follower") << "Listening for route on " << leader_route_topic;
  CLOG(INFO, "mpc.follower") << "Target separation: " << config->distance_margin;
  CLOG(INFO, "mpc.follower") << "Robot's wheelbase: " << config->wheelbase << "m";

  leaderRolloutSub_ = robot_state->node->create_subscription<PathMsg>(leader_path_topic, rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&BicycleMPCPathTrackerFollower::onLeaderPath, this, _1));
  leaderRouteSub_ = robot_state->node->create_subscription<RouteMsg>(leader_route_topic, rclcpp::SystemDefaultsQoS(), std::bind(&BicycleMPCPathTrackerFollower::onLeaderRoute, this, _1));

  leaderGraphSrv_ = robot_state->node->create_client<GraphStateSrv>(leader_graph_topic);
  followerGraphSrv_ = robot_state->node->create_client<GraphStateSrv>("vtr/graph_state_srv");

}


BicycleMPCPathTrackerFollower::~BicycleMPCPathTrackerFollower() {}

auto BicycleMPCPathTrackerFollower::computeCommand(RobotState& robot_state) -> Command {
  auto raw_command = computeCommand_(robot_state);
  
  Eigen::Vector2d output_vel = {raw_command.linear.x, raw_command.angular.z};

  // Apply robot motor controller calibration scaling factors if applicable
  output_vel(0) = output_vel(0) * config_->robot_linear_velocity_scale;
  output_vel(1) = output_vel(1) * config_->robot_angular_velocity_scale;

  // If required, saturate the output velocity commands based on the configuration limits
  CLOG(DEBUG, "cbit.control") << "Saturating the velocity command if required";
  Eigen::Vector2d saturated_vel = saturateVel(output_vel, config_->max_lin_vel, config_->max_ang_vel);
  CLOG(INFO, "cbit.control") << "The Saturated linear velocity is:  " << saturated_vel(0) << " The angular vel is: " << saturated_vel(1);
  
  Command command;
  command.linear.x = saturated_vel(0);
  command.angular.z = saturated_vel(1);
  applied_vel_ = saturated_vel;

  // Store the result in memory so we can use previous state values to re-initialize and extrapolate the robot pose in subsequent iterations
  vel_history.erase(vel_history.begin());
  vel_history.push_back(applied_vel_);

  CLOG(INFO, "cbit.control")
    << "Final control command: [" << command.linear.x << ", "
    << command.linear.y << ", " << command.linear.z << ", "
    << command.angular.x << ", " << command.angular.y << ", "
    << command.angular.z << "]";
  
  return command;
}


// Generate twist commands to track the planned local path (function is called at the control rate)
auto BicycleMPCPathTrackerFollower::computeCommand_(RobotState& robot_state) -> Command {
  auto& chain = robot_state.chain.ptr();
  if (!chain->isLocalized()) {
    CLOG(WARNING, "cbit.control") << "Robot is not localized, commanding the robot to stop";
    return Command();
  }
  
  if (recentLeaderPath_ == nullptr) {
    CLOG_EVERY_N(1, WARNING, "cbit.control") << "Follower has received no path from the leader yet. Stopping";
    return Command();
  }

  // retrieve the transform info from the localization chain for the current robot state
  const auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_w_v_odo, T_r_v_odo, curr_sid] = getChainInfo(*chain);

  // Store the current robot state in the robot state path so it can be visualized
  auto T_w_r = T_w_p * T_p_r;

  CasadiBicycleMPCFollower::Config mpcConfig;
  mpcConfig.vel_max = {config_->max_lin_vel, config_->max_ang_vel};
  mpcConfig.wheelbase = config_->wheelbase;
  mpcConfig.Q_x     = config_->q_x;
  mpcConfig.Q_y     = config_->q_y;
  mpcConfig.Q_th    = config_->q_th;
  mpcConfig.R1      = config_->r1;
  mpcConfig.R2      = config_->r2;
  mpcConfig.Acc_R1  = config_->racc1;
  mpcConfig.Acc_R2  = config_->racc2;
  mpcConfig.lin_acc_max = config_->max_lin_acc;
  mpcConfig.ang_acc_max = config_->max_ang_acc;
  mpcConfig.Q_f = config_->q_f;
  mpcConfig.Q_dist = config_->q_dist;
  mpcConfig.distance = config_->following_offset;
  mpcConfig.distance_margin = config_->distance_margin;

  // Schedule speed based on path curvatures + other factors
  // TODO refactor to accept the chain and use the curvature of the links
  mpcConfig.VF = ScheduleSpeed(chain, {config_->forward_vel, config_->min_vel, config_->planar_curv_weight, config_->profile_curv_weight, config_->eop_weight, 7});


  // EXTRAPOLATING ROBOT POSE INTO THE FUTURE TO COMPENSATE FOR SYSTEM DELAYS
  auto T_p_r_extp = T_p_r;
  auto curr_time = stamp;  // always in nanoseconds

  if (config_->extrapolate_robot_pose) {
    curr_time = robot_state.node->now().nanoseconds();  // always in nanoseconds
    auto dt = static_cast<double>(curr_time - stamp) * 1e-9 - 0.05;
    if (fabs(dt) > 0.25) { 
      CLOG(WARNING, "cbit") << "Pose extrapolation was requested but the time delta is " << dt << "s.\n"
            << "Ignoring extrapolation requestion. Check your time sync!";
      dt = 0;
    }

    Eigen::Matrix<double, 6, 1> xi_p_r_in_r(-dt * w_p_r_in_r);
    T_p_r_extp = T_p_r * tactic::EdgeTransform(xi_p_r_in_r);

    CLOG(DEBUG, "cbit.debug") << "New extrapolated pose:"  << T_p_r_extp;
  }

  const auto leader_path_time = rclcpp::Time(recentLeaderPath_->header.stamp).nanoseconds();
  const auto delta_t = curr_time - leader_path_time;
  if (delta_t > 1e9) {
    CLOG_EVERY_N(1, WARNING,"cbit.control") << "Follower has received no path from the leader in more than 1 second. Stopping\n Delay: "
           << delta_t / 1e9;
    return Command();
  }

  lgmath::se3::Transformation T0 = T_p_r_extp;
  mpcConfig.T0 = tf_to_global(T0);

  CLOG(DEBUG, "cbit.control") << "Last velocity " << w_p_r_in_r << " with stamp " << stamp;


  // Define Leader Waypoints
  mpcConfig.leader_reference_poses.clear();
  std::vector<lgmath::se3::Transformation> leader_world_poses;
  std::vector<double> leader_p_values;
  const auto leaderPath_copy = *leaderPathInterp_;
  CLOG(DEBUG, "mpc.follower") << "TF to leader:\n" << T_fw_lw_ * leaderPath_copy.at(curr_time) * (T_w_p * T_p_r_extp).inverse();
  for (uint i = 0; i < mpcConfig.N; i++){
    const auto T_w_lp = T_fw_lw_ * leaderPath_copy.at(curr_time + (1+i) * mpcConfig.DT * 1e9);
    mpcConfig.leader_reference_poses.push_back(tf_to_global(T_w_p.inverse() *  T_w_lp));
    leader_world_poses.push_back(T_w_lp);
    leader_p_values.push_back(findRobotP(T_w_lp, chain));
    CLOG(DEBUG, "mpc.follower.target") << "Leader Target " << tf_to_global(T_w_p.inverse() *  T_w_lp);

  }

  double state_p = findRobotP(T_w_p * T_p_r_extp, chain);

  mpcConfig.reference_poses.clear();
  auto referenceInfo = [&](){
    if(config_->waypoint_selection == "euclidean") {
      return generateFollowerReferencePosesEuclidean(leader_world_poses, leader_p_values, chain, state_p, mpcConfig.distance);
    } else {
      CLOG_IF(config_->waypoint_selection ==  "arclength", WARNING, "mpc.follower") << "Arclength not implemented yet for bicycle!";

      std::vector<double> p_rollout;
      for(int j = 1; j < mpcConfig.N+1; j++){
        p_rollout.push_back(state_p + j*mpcConfig.VF*mpcConfig.DT);
      }
      return generateHomotopyReference(p_rollout, chain);
    }
  }();
  
  for(const auto& Tf : referenceInfo.poses) {
    mpcConfig.reference_poses.push_back(tf_to_global(T_w_p.inverse() *  Tf));
    CLOG(DEBUG, "test") << "Target " << tf_to_global(T_w_p.inverse() *  Tf);
  }
  // mpcConfig.up_barrier_q = referenceInfo.barrier_q_max;
  // mpcConfig.low_barrier_q = referenceInfo.barrier_q_min;
  mpcConfig.previous_vel = {-w_p_r_in_r(0, 0), -w_p_r_in_r(5, 0)};
  

  // Create and solve the casadi optimization problem
  std::vector<lgmath::se3::Transformation> mpc_poses;
  // return the computed velocity command for the first time step
  Command command;
  std::vector<Eigen::Vector2d> mpc_velocities;
  try {
    CLOG(INFO, "cbit.control") << "Attempting to solve the MPC problem";
    auto mpc_res = solver_.solve(mpcConfig);
    
    for(int i = 0; i < mpc_res["pose"].columns(); i++) {
      const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
      mpc_poses.push_back(T_w_p * tf_from_global(pose_i[0], pose_i[1], pose_i[2]));
    }

    CLOG(INFO, "cbit.control") << "Successfully solved MPC problem";
    const auto& mpc_vel_vec = mpc_res["vel"](casadi::Slice(), 0).get_elements();

    command.linear.x = mpc_vel_vec[0];
    command.angular.z = mpc_vel_vec[1];

    // Get all the mpc velocities 
    for (int i = 0; i < mpc_res["vel"].columns(); i++) {
      const auto& vel_i = mpc_res["vel"](casadi::Slice(), i).get_elements();
      mpc_velocities.emplace_back(vel_i[0], vel_i[1]);
    }

  } catch(std::exception &e) {
    CLOG(WARNING, "cbit.control") << "casadi failed! " << e.what() << " Commanding to Stop the Vehicle";
    return Command();
  }

  vis_->publishMPCRollout(mpc_poses, curr_time, mpcConfig.DT);
  vis_->publishLeaderRollout(leader_world_poses, leaderPath_copy.start(), mpcConfig.DT);
  vis_->publishReferencePoses(referenceInfo.poses);


  CLOG(INFO, "cbit.control") << "The linear velocity is:  " << command.linear.x << " The angular vel is: " << command.angular.z;

  return command;
  
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
    leader_vel_ << vel(0, 0), vel(5, 0);
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
    auto connected = graph_->dijkstraSearch(follower_root, leader_root_);
    
    T_fw_lw_ = pose_graph::eval::ComposeTfAccumulator(connected->beginDfs(follower_root), connected->end(), tactic::EdgeTransform(true));    
    CLOG(INFO, "mpc.follower") << "Set relative transform to : " << T_fw_lw_;

  }
}

}  // namespace vtr::path_planning
