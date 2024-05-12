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
 * \file cbit.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <cstdio>
#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include <algorithm>
#include <random>
#include <chrono> // For benchmarking
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "vtr_path_planning/cbit/cbit_config.hpp"
#include "vtr_path_planning/base_path_planner.hpp"
#include "vtr_path_planning/cbit/utils.hpp"
#include "vtr_path_planning/cbit/generate_pq.hpp"
#include "vtr_path_planning/cbit/cbit_path_planner.hpp"
#include "vtr_path_planning/cbit/cbit_costmap.hpp"
#include "vtr_path_planning/cbit/visualization_utils.hpp"
#include "vtr_path_planning/mpc/speed_scheduler.hpp"

#include "steam.hpp"


namespace vtr {
namespace path_planning {

class CBIT : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(CBIT);

  static constexpr auto static_name = "cbit";

  // Note all rosparams that are in the config yaml file need to be declared here first, though they can be then changes using the declareparam function for ros in the cpp file
  struct Config : public BasePathPlanner::Config {
    PTR_TYPEDEFS(Config);

    // point, circular, line, two_circles, polygon
    std::string robot_model = "point";
    double robot_radius = 0.5;

    // CBIT
    // Environment
    double obs_padding = 1.0;
    int curv_to_euclid_discretization = 20;
    double sliding_window_width = 15.0;
    double sliding_window_freespace_padding = 0.5;
    double corridor_resolution = 0.05;
    double state_update_freq = 1; // In Hz
    bool update_state = true;
    int rand_seed = 1;

    // ROC
    double roc_lookahead = 2.0;
    int roc_discretization = 40;
    double roc_q_tolerance = 0.001;

    // Planner Tuning Params
    int initial_samples = 250;
    int batch_samples = 100;
    double pre_seed_resolution = 0.5;
    double alpha = 0.5;
    double q_max = 2.5;
    int frame_interval = 50;
    int iter_max = 10000000;
    double eta = 1.1;
    double rad_m_exhange = 1.00;
    double initial_exp_rad = 1.00;
    bool extrapolation = true;

    // Misc
    bool incremental_plotting = false;
    bool plotting = true;

    // Speed Scheduler
    double planar_curv_weight = 2.50;
    double profile_curv_weight = 0.5; 
    double eop_weight = 1.0;
    double min_vel = 0.5;  

    // MPC Configs
    bool obstacle_avoidance = false;
    bool extrapolate_robot_pose = true;
    bool mpc_verbosity = false;
    bool homotopy_guided_mpc = false;
    int horizon_steps = 10;
    double horizon_step_size = 0.5;
    double forward_vel = 0.75;
    double max_lin_vel = 1.25;
    double max_ang_vel = 0.75;
    double robot_linear_velocity_scale = 1.0;
    double robot_angular_velocity_scale = 1.0;

    // Add unicycle model param

    // Covariance tuning weights
    Eigen::Matrix<double, 6, 6> pose_error_cov = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 2, 2> vel_error_cov = Eigen::Matrix<double, 2, 2>::Zero();
    Eigen::Matrix<double, 2, 2> acc_error_cov = Eigen::Matrix<double, 2, 2>::Zero();
    Eigen::Matrix<double, 6, 6> kin_error_cov = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 1, 1> lat_error_cov = Eigen::Matrix<double, 1, 1>::Zero();

    // MPC weight params:
    double pose_error_weight = 1.0;
    double vel_error_weight = 1.0;
    double acc_error_weight = 1.0;
    double kin_error_weight = 1.0;
    double lat_error_weight = 0.01;

    // Misc
    int command_history_length = 100;

    // COSTMAP PARAMS;
    double costmap_filter_value = 0.01;
    int costmap_history = 5;


    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
    // Subscription for parameter change
    rclcpp::AsyncParametersClient::SharedPtr ros_parameters_client;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
        ros_parameter_event_sub;
  };

  CBIT(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const Callback::Ptr& callback);
  ~CBIT() override;

  void setRunning(const bool running) override;

 protected:
  void initializeRoute(RobotState& robot_state);
  Command computeCommand(RobotState& robot_state) override;


  
 protected:
  struct ChainInfo {
    tactic::Timestamp stamp;
    Eigen::Matrix<double, 6, 1> w_p_r_in_r;
    // planning frame is the current localization frame
    tactic::EdgeTransform T_p_r;  // T_planning_robot
    tactic::EdgeTransform T_w_p;  // T_world_planning
    tactic::EdgeTransform T_w_v_odo;  // T_planning_robot
    tactic::EdgeTransform T_r_v_odo;  // T_world_planning
    unsigned curr_sid;
  };
  /** \brief Retrieve information for planning from localization chain */
  ChainInfo getChainInfo(RobotState& robot_state);

 private: 
  const Config::ConstPtr config_;
  CBITConfig cbit_config;
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(CBIT);

  std::shared_ptr<CBITPlanner> planner_ptr_;

  // Pointers to the output path
  std::vector<Pose> cbit_path;
  std::shared_ptr<std::vector<Pose>> cbit_path_ptr;

  // Pointers to the flag for there being a valid solution or not
  std::shared_ptr<bool> valid_solution_ptr;

  // Pointer that sets the maximum lateral deviation the planner is allowed to use in planning
  std::shared_ptr<double> q_max_ptr;

  // Pointers to the corridor
  std::shared_ptr<CBITCorridor> corridor_ptr;

  // Pointer to the global path
  std::shared_ptr<CBITPath> global_path_ptr;

  // Pointer to visualizer
  std::shared_ptr<VisualizationUtils> visualization_ptr;

  unsigned int prev_costmap_sid = 0;
  tactic::Timestamp prev_stamp;

  // Store the previously applied velocity and a sliding window history of MPC results
  Eigen::Vector2d applied_vel;
  std::vector<Eigen::Vector2d> vel_history;

  //create vector to store the robots path for visualization purposes
  std::vector<lgmath::se3::Transformation> robot_poses;


  // Create costmap pointer object
  std::shared_ptr<CBITCostmap> costmap_ptr = std::make_shared<CBITCostmap> ();

 private:
  void process_cbit();
  std::thread process_thread_cbit_;
  void stop_cbit();

 private:
  /** \brief shared memory that stores the current robot state */
  RobotState::Ptr robot_state_;


};

}  // namespace path_planning
}  // namespace vtr