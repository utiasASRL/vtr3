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
 * \file unicycle_mpc_path_tracker.hpp
 * \author Alec Krawciw, Luka Antonyshyn Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_logging/logging.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_common/conversions/ros_lgmath.hpp>
#include <vtr_path_planning/mpc/bicycle_mpc_path_tracker.hpp>
#include <vtr_path_planning/mpc/casadi_path_planners.hpp>
#include <vtr_path_planning/mpc/speed_scheduler.hpp>
#include <vtr_path_planning/mpc/follower_common.hpp>

#include <vtr_path_planning/cbit/visualization_utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <vtr_navigation_msgs/msg/graph_route.hpp>
#include <vtr_navigation_msgs/srv/graph_state.hpp>
#include <vtr_navigation_msgs/srv/following_route.hpp>
#include "std_msgs/msg/float32.hpp"

namespace vtr {
namespace path_planning {

class BicycleMPCPathTrackerFollower : public BicycleMPCPathTracker {
 public:
  PTR_TYPEDEFS(BicycleMPCPathTrackerFollower);

  static constexpr auto static_name = "bicycle_mpc_follower";

  using PathMsg = nav_msgs::msg::Path;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using RouteMsg = vtr_navigation_msgs::msg::GraphRoute;
  using GraphStateSrv = vtr_navigation_msgs::srv::GraphState;
  using FollowingRouteSrv = vtr_navigation_msgs::srv::FollowingRoute;
  using Transformation = lgmath::se3::Transformation;
  using FloatMsg = std_msgs::msg::Float32;

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BicycleMPCPathTracker::Config {
    PTR_TYPEDEFS(Config);
    std::string leader_namespace = "leader";
    
    // Options: leader_vel euclidean external_dist
    std::string waypoint_selection = "leader_vel";

    double following_offset = 0.5; //m
    double distance_margin = 1.0;
    double f_q_dist = 1.0;
    double r_q_dist = 1.0;

    //Distance PID Gains
    double kp = 1.0;
    double kd = 0.0;
    double ki = 0.0;

    // Misc
    int command_history_length = 100;

    static void loadConfig(Config::Ptr config,  
		           const rclcpp::Node::SharedPtr& node,
                           const std::string& prefix = "path_planning");

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  BicycleMPCPathTrackerFollower(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);
  ~BicycleMPCPathTrackerFollower() override;

  void setRunning(const bool running) override;

 protected:
  void initializeRoute(RobotState& robot_state);

  void loadMPCConfig(
      CasadiBicycleMPCFollower::Config::Ptr mpc_config, const bool isReversing,   Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel);

  CasadiMPC::Config::Ptr getMPCConfig(
      const bool isReversing,  Eigen::Matrix<double, 6, 1> w_p_r_in_r, Eigen::Vector2d applied_vel) override;
  
  virtual bool isMPCStateValid(CasadiMPC::Config::Ptr mpcConfig, const tactic::Timestamp& curr_time) override;

  virtual void loadMPCPath(CasadiMPC::Config::Ptr mpcConfig, const lgmath::se3::Transformation& T_w_p,
                           const lgmath::se3::Transformation& T_p_r_extp,
                           const double state_p,
                           RobotState& robot_state,
                           const tactic::Timestamp& curr_time) override;

  std::map<std::string, casadi::DM> callSolver(CasadiMPC::Config::Ptr config) override;


 private: 
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(BicycleMPCPathTrackerFollower);

  Config::ConstPtr config_;
  CasadiBicycleMPCFollower solver_;
  tactic::GraphBase::Ptr graph_;

  RobotState::Ptr robot_state_;

  PathMsg::SharedPtr recentLeaderPath_;
  rclcpp::Subscription<PathMsg>::SharedPtr leaderRolloutSub_;
  void onLeaderPath(const PathMsg::SharedPtr path);
  Eigen::Vector2d leader_vel_;
  std::vector<Transformation> leaderRollout_;
  PathInterpolator::ConstPtr leaderPathInterp_; 
  PoseStampedMsg lastRobotPose_;

  tactic::EdgeTransform T_fw_lw_;
  tactic::VertexId leader_root_ = tactic::VertexId::Invalid();
  
  rclcpp::Client<FollowingRouteSrv>::SharedPtr leaderRouteSrv_;
  void leaderRouteCallback(const rclcpp::Client<FollowingRouteSrv>::SharedFuture Future);
  rclcpp::Client<GraphStateSrv>::SharedPtr leaderGraphSrv_;
  rclcpp::Client<GraphStateSrv>::SharedPtr followerGraphSrv_;

  FloatMsg::SharedPtr recentLeaderDist_;
  rclcpp::Subscription<FloatMsg>::SharedPtr leaderDistanceSub_;
  void onLeaderDist(const FloatMsg::SharedPtr distance);
  float lastError_ = 0;
  float errorIntegrator = 0;
  bool hasRequestedLeaderRoute_ = false;


};

}  // namespace path_planning
}  // namespace vtr
