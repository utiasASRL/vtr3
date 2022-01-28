// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
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
 * \file teb_path_planner.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"

/// \note Some library here overrides the "LOG" macro to google logging
/// which conflis with the "LOG" macro in the vtr logging library, i.e.
/// easylogging++.
// timed-elastic-band related classes
#include "teb_local_planner/homotopy_class_planner.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/visualization.h"

#include "vtr_path_planning/base_path_planner.hpp"

namespace vtr {
namespace path_planning {

class TEBPathPlanner : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(TEBPathPlanner);

  static constexpr auto static_name = "teb";

  struct Config : public BasePathPlanner::Config,
                  public teb_local_planner::TebConfig {
    PTR_TYPEDEFS(Config);
    //
    bool visualize = false;
    // whether or not to extrapolate using the current velocity estimate
    bool extrapolate = false;
    double extrapolation_timeout = 1.0;  // in seconds
    //
    double lookahead_distance = 8.0;
    // point, circular, line, two_circles, polygon
    std::string robot_model = "point";
    double robot_radius = 0.5;

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
    // Subscription for parameter change
    rclcpp::AsyncParametersClient::SharedPtr ros_parameters_client;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
        ros_parameter_event_sub;
  };

  TEBPathPlanner(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const Callback::Ptr& callback);
  ~TEBPathPlanner() override;

 protected:
  void initializeRoute(RobotState& robot_state) override;
  Command computeCommand(RobotState& robot_state) override;

 protected:
  struct ChainInfo {
    tactic::Timestamp stamp;
    Eigen::Matrix<double, 6, 1> w_p_r_in_r;
    // planning frame is the current localization frame
    tactic::EdgeTransform T_p_r;  // T_planning_robot
    tactic::EdgeTransform T_w_p;  // T_world_planning
    tactic::EdgeTransform T_p_g;  // T_planning_goal
    std::vector<tactic::EdgeTransform> T_p_i_vec;
    unsigned curr_sid;
  };
  /** \brief Retrieve information for planning from localization chain */
  ChainInfo getChainInfo(RobotState& robot_state);

  void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x,
                        double max_vel_y, double max_vel_theta) const;

  void visualize(const tactic::Timestamp& stamp,
                 const tactic::EdgeTransform& T_w_p,
                 const tactic::EdgeTransform& T_p_r,
                 const tactic::EdgeTransform& T_p_r_extp,
                 const tactic::EdgeTransform& T_p_g,
                 const std::vector<tactic::EdgeTransform>& T_p_i_vec) const;

 private:
  const Config::ConstPtr config_;

 protected:
  // Instance of the underlying optimal planner class
  teb_local_planner::PlannerInterfacePtr planner_;
  // Obstacle vector that should be considered during trajectory optimization
  teb_local_planner::ObstContainer obstacles_;
  // via-points that should be considered during trajectory optimization
  teb_local_planner::ViaPointContainer via_points_;
  //
  teb_local_planner::CostMapContainer costmaps_;
  //
  teb_local_planner::TebVisualizationPtr visualization_;

  // for rviz visualization
 private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(TEBPathPlanner);
};

}  // namespace path_planning
}  // namespace vtr
