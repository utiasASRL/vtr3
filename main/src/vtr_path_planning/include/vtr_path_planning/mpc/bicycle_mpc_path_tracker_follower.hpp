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
#include <vtr_path_planning/base_path_planner.hpp>
#include <vtr_path_planning/mpc/bicycle_mpc_path_tracker.hpp>
#include <vtr_path_planning/mpc/casadi_path_planners.hpp>
#include <vtr_path_planning/mpc/speed_scheduler.hpp>
#include <vtr_path_planning/mpc/follower_common.hpp>

#include <vtr_path_planning/cbit/visualization_utils.hpp>

#include <rclcpp/rclcpp.hpp>
#include <vtr_navigation_msgs/msg/graph_route.hpp>
#include <vtr_navigation_msgs/srv/graph_state.hpp>

namespace vtr {
namespace path_planning {

class BicycleMPCPathTrackerFollower : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(BicycleMPCPathTrackerFollower);

  static constexpr auto static_name = "bicycle_mpc_follower";

  using PathMsg = nav_msgs::msg::Path;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using RouteMsg = vtr_navigation_msgs::msg::GraphRoute;
  using GraphStateSrv = vtr_navigation_msgs::srv::GraphState;
  using Transformation = lgmath::se3::Transformation;

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BicycleMPCPathTracker::Config {
    PTR_TYPEDEFS(Config);
    double wheelbase = 0.5;
    
    std::string leader_namespace = "leader";
    std::string waypoint_selection = "leader_vel";

    double following_offset = 0.5; //m
    double distance_margin = 1.0;
    double f_q_dist = 1.0;
    double r_q_dist = 1.0;

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

 protected:
  void initializeRoute(RobotState& robot_state);
  Command computeCommand(RobotState& robot_state) override;

 private: 
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(BicycleMPCPathTrackerFollower);

  Config::ConstPtr config_;
  CasadiBicycleMPCFollower solver_;
  tactic::GraphBase::Ptr graph_;


  // Store the previously applied velocity and a sliding window history of MPC results
  Eigen::Vector2d applied_vel_;
  std::vector<Eigen::Vector2d> vel_history;
  tactic::Timestamp prev_vel_stamp_;
  RobotState::Ptr robot_state_;
  u_int32_t frame_delay_ = 0;
  Command computeCommand_(RobotState& robot_state);


  PathMsg::SharedPtr recentLeaderPath_;
  rclcpp::Subscription<PathMsg>::SharedPtr leaderRolloutSub_;
  void onLeaderPath(const PathMsg::SharedPtr path);
  Eigen::Vector2d leader_vel_;
  std::vector<Transformation> leaderRollout_;
  PathInterpolator::ConstPtr leaderPathInterp_; 
  PoseStampedMsg lastRobotPose_;

  rclcpp::Subscription<RouteMsg>::SharedPtr leaderRouteSub_;
  void onLeaderRoute(const RouteMsg::SharedPtr route);
  tactic::EdgeTransform T_fw_lw_;
  tactic::VertexId leader_root_ = tactic::VertexId::Invalid();

  rclcpp::Client<GraphStateSrv>::SharedPtr leaderGraphSrv_;
  rclcpp::Client<GraphStateSrv>::SharedPtr followerGraphSrv_;

  VisualizationUtils::Ptr vis_;  


};

}  // namespace path_planning
}  // namespace vtr
