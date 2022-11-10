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
 * \file teb_path_planner.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_lidar/path_planning/teb_path_planner.hpp"

#include "vtr_lidar/cache.hpp"

using namespace teb_local_planner;

namespace vtr {
namespace lidar {

namespace {

inline std::tuple<double, double, double> T2xyth(
    const tactic::EdgeTransform& T) {
  const auto Tm = T.matrix();
  return std::make_tuple(Tm(0, 3), Tm(1, 3), std::atan2(Tm(1, 0), Tm(0, 0)));
}

inline tactic::EdgeTransform xyth2T(
    const std::tuple<double, double, double>& xyth) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = std::get<0>(xyth);
  T(1, 3) = std::get<1>(xyth);
  const auto theta = std::get<2>(xyth);
  T.block<2, 2>(0, 0) << std::cos(theta), -std::sin(theta), std::sin(theta),
      std::cos(theta);
  return tactic::EdgeTransform(T);
}

}  // namespace

auto LidarTEBPathPlanner::Config::fromROS(const rclcpp::Node::SharedPtr& node,
                                          const std::string& prefix) -> Ptr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // basics
  config->control_period = (unsigned int)node->declare_parameter<int>(prefix + ".control_period", config->control_period);
  config->visualize = node->declare_parameter<bool>(prefix + ".teb.visualize", config->visualize);
  // extrapolation
  config->extrapolate = node->declare_parameter<bool>(prefix + ".teb.extrapolate", config->extrapolate);
  config->extrapolation_timeout = node->declare_parameter<double>(prefix + ".teb.extrapolation_timeout", config->extrapolation_timeout);
  // lookahead
  config->lookahead_distance = node->declare_parameter<double>(prefix + ".teb.lookahead_distance", config->lookahead_distance);
  // robot configuration
  config->robot_model = node->declare_parameter<std::string>(prefix + ".teb.robot_model", config->robot_model);
  config->robot_radius = node->declare_parameter<double>(prefix + ".teb.robot_radius", config->robot_radius);
  // clang-format on
  config->declareParameters(node, prefix + ".teb");
  config->loadRosParamFromNodeHandle(node, prefix + ".teb");
  // set up callback for changes to parameters
  config->ros_parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(
          node->get_node_base_interface(), node->get_node_topics_interface(),
          node->get_node_graph_interface(),
          node->get_node_services_interface());
  config->ros_parameter_event_sub =
      config->ros_parameters_client->on_parameter_event(
          std::bind(&teb_local_planner::TebConfig::on_parameter_event_callback,
                    std::ref(*config), std::placeholders::_1));
  return config;
}

LidarTEBPathPlanner::LidarTEBPathPlanner(const Config::ConstPtr& config,
                                         const RobotState::Ptr& robot_state,
                                         const Callback::Ptr& callback)
    : Parent(config, robot_state, callback), config_(config) {}

LidarTEBPathPlanner::~LidarTEBPathPlanner() { stop(); }

auto LidarTEBPathPlanner::computeCommand(RobotState& robot_state0) -> Command {
  auto& robot_state = dynamic_cast<LidarOutputCache&>(robot_state0);

  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "path_planning.teb")
        << "Robot is not localized, command to stop the robot";
    return Command();
  }
    else {
    CLOG(INFO, "path_planning.teb") << "Robot is now localized and we can start doing things";
  }

  // retrieve info from the localization chain
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_p_g, T_p_i_vec, curr_sid] =
      chain_info;

  //CLOG(INFO, "path_planning.teb") << "stamp " << stamp;
  //CLOG(INFO, "path_planning.teb") << "w_p_r_in_r: " << w_p_r_in_r;
  //CLOG(INFO, "path_planning.teb") << "T_p_r: " << T_p_r;
  //CLOG(INFO, "path_planning.teb") << "T_w_p: " << T_w_p;
  //CLOG(INFO, "path_planning.teb") << "T_p_g: " << T_p_g;
  //CLOG(INFO, "path_planning.teb") << "T_p_i_vec: " << T_p_i_vec;
  //Testing trying to grab all of the teach key frame transforms
  //CLOG(INFO, "path_planning.teb") << "Key Frame 1 " << chain.pose(0);
  //CLOG(INFO, "path_planning.teb") << "Key Frame 2 " << chain.pose(1);


  // get robot pose project onto 2D plane w.r.t. the planning frame
  const auto T_p_r_2d = xyth2T(T2xyth(T_p_r));

  // keep current time
  const auto curr_time = now();  // always in nanoseconds
  const auto dt = static_cast<double>(curr_time - stamp) * 1e-9;
  if (config_->extrapolate && std::abs(dt) > config_->extrapolation_timeout) {
    CLOG(WARNING, "path_planning.teb")
        << "Time difference b/t estimation and planning: " << dt
        << ", extrapolation timeout reached, stopping the robot";
    return Command();
  }

  // extrapolate robot pose if required
  const auto T_p_r_extp = [&]() {
    if (!config_->extrapolate) return T_p_r;
    // extrapolate based on current time
    Eigen::Matrix<double, 6, 1> xi_p_r_in_r(dt * w_p_r_in_r);
    return T_p_r * tactic::EdgeTransform(xi_p_r_in_r).inverse();
  }();
  // get robot pose project onto 2D plane w.r.t. the planning frame
  const auto T_p_r_extp_2d = xyth2T(T2xyth(T_p_r_extp));
  const auto [rx, ry, rt] = T2xyth(T_p_r_extp);
  const auto robot_pose = PoseSE2(rx, ry, rt);

  // get robot velocity
  Command robot_velocity;
  robot_velocity.linear.x = config_->extrapolate ? -w_p_r_in_r(0) : 0.0;
  robot_velocity.linear.y = config_->extrapolate ? -w_p_r_in_r(1) : 0.0;
  robot_velocity.linear.z = config_->extrapolate ? -w_p_r_in_r(2) : 0.0;
  robot_velocity.angular.x = config_->extrapolate ? -w_p_r_in_r(3) : 0.0;
  robot_velocity.angular.y = config_->extrapolate ? -w_p_r_in_r(4) : 0.0;
  robot_velocity.angular.z = config_->extrapolate ? -w_p_r_in_r(5) : 0.0;

  // get intermediate poses project onto 2D plane w.r.t. the planning frame
  decltype(T_p_i_vec) T_p_i_2d_vec;
  via_points_.clear();
  for (const auto& T_p_i : T_p_i_vec) {
    const auto [ix, iy, it] = T2xyth(T_p_i);
    (void)it;
    via_points_.emplace_back(Eigen::Vector2d(ix, iy));
    T_p_i_2d_vec.emplace_back(xyth2T(T2xyth(T_p_i)));
  }

  // get goal pose project onto 2D plane w.r.t. the planning frame
  const auto [gx, gy, gt] = T2xyth(T_p_g);
  const auto goal_pose = PoseSE2(gx, gy, gt);
  const auto T_p_g_2d = xyth2T(T2xyth(T_p_g));

  // vtr visualization
  if (config_->visualize)
    visualize(0, T_w_p, T_p_r_2d, T_p_r_extp_2d, T_p_g_2d, T_p_i_2d_vec);

  /// \todo clear planner when we switch to a different planning frame

  CLOG(DEBUG, "path_planning.teb")
      << "Current time: " << curr_time << ", leaf time: " << stamp
      << "Time difference in seconds: " << dt << std::endl
      << "Robot velocity: " << -w_p_r_in_r.transpose() << std::endl
      << "Planning a trajectory from: [" << rx << ", " << ry << ", " << rt
      << "] to: [" << gx << ", " << gy << ", " << gt << "]";

  obstacles_.clear();
#if false
  // get current change detection result
  {
    auto change_detection_costmap_ref =
        robot_state.change_detection_costmap.locked();
    auto& change_detection_costmap = change_detection_costmap_ref.get();
    if (change_detection_costmap.valid()) {
      // update change detection result
      /// \todo hardcoded threshold
      const auto occupied = change_detection_costmap->filter(0.3);
      const auto costmap_sid = change_detection_costmap->vertex_sid();
      const auto costmap_T_vertex_this =
          change_detection_costmap->T_vertex_this();
      const auto ogm_dl = change_detection_costmap->dl();
      auto& chain = *robot_state.chain;
      const auto T_start_vertex = chain.pose(costmap_sid);
      const auto T_start_trunk = chain.pose(curr_sid);
      const auto T_trunk_vertex = T_start_trunk.inverse() * T_start_vertex;
      const auto T_trunk_this = T_trunk_vertex * costmap_T_vertex_this;
      change_detection_costmap->T_this_plan() = T_trunk_this.inverse();
      CLOG(DEBUG, "path_planning.teb")
          << "change detection OGM information: " << std::endl
          << "T_trunk_vertex: " << T_trunk_vertex.vec().transpose() << std::endl
          << "T_trunk_this: " << T_trunk_this.vec().transpose();
      for (const auto& p : occupied) {
        Eigen::Vector4d p_in_ogm(p.first.first, p.first.second, 0.0, 1.0);
        const auto p_in_trunk = T_trunk_this * p_in_ogm;
        obstacles_.emplace_back(std::make_shared<CircularObstacle>(
            p_in_trunk.head(2), ogm_dl / 2.0));
      }
    }
  }
#endif

  costmaps_.clear();
  // get current change detection result
  {
    auto change_detection_costmap_ref =
        robot_state.change_detection_costmap.locked();
    auto& change_detection_costmap = change_detection_costmap_ref.get();
    if (change_detection_costmap.valid()) {
      // update change detection result
      const auto costmap_sid = change_detection_costmap->vertex_sid();
      const auto costmap_T_vertex_this =
          change_detection_costmap->T_vertex_this();
      auto& chain = *robot_state.chain;
      const auto T_start_vertex = chain.pose(costmap_sid);
      const auto T_start_trunk = chain.pose(curr_sid);
      const auto T_trunk_vertex = T_start_trunk.inverse() * T_start_vertex;
      const auto T_trunk_this = T_trunk_vertex * costmap_T_vertex_this;
      change_detection_costmap->T_this_plan() = T_trunk_this.inverse();
      CLOG(DEBUG, "path_planning.teb")
          << "change detection costmap information: " << std::endl
          << "T_trunk_vertex: " << T_trunk_vertex.vec().transpose() << std::endl
          << "T_trunk_this: " << T_trunk_this.vec().transpose();
      /// it's ok to use the shared ptr here directly, because the costmap won't
      /// be changed from outside this thread, change detection module from
      /// pipline will assign a new pointer to output cache when new costmap is
      /// computed
      costmaps_.push_back(change_detection_costmap.ptr());
    }
  }

  /// Do not allow config changes during the following optimization step
  std::lock_guard<std::mutex> config_lock(config_->configMutex());

  //
  if (!planner_->plan(robot_pose, goal_pose, &robot_velocity,
                      config_->goal_tolerance.free_goal_vel)) {
    CLOG(WARNING, "path_planning.teb") << "Planning failed!";
    planner_->clearPlanner();  // force reinitialization for next time
    /// \todo add infeasible plan counter
  }

  // check for convergence
  if (planner_->hasDiverged()) {
    CLOG(WARNING, "path_planning.teb") << "Planner has diverged!";
    // Reset everything to start again with the initialization of new
    // trajectories.
    planner_->clearPlanner();
  }

  /// \todo check if the generated trajectory is feasible

  // get the velocity command for this sampling interval
  Command command;
  if (!planner_->getVelocityCommand(
          command.linear.x, command.linear.y, command.angular.z,
          config_->trajectory.control_look_ahead_poses)) {
    CLOG(WARNING, "path_planning.teb") << "Computing velocity command failed!";
    planner_->clearPlanner();
  }

  // saturate velocity command
  saturateVelocity(command.linear.x, command.linear.y, command.angular.z,
                   config_->robot.max_vel_x, config_->robot.max_vel_y,
                   config_->robot.max_vel_theta);

  CLOG(DEBUG, "path_planning.teb")
      << "Final control command: [" << command.linear.x << ", "
      << command.linear.y << ", " << command.linear.z << ", "
      << command.angular.x << ", " << command.angular.y << ", "
      << command.angular.z << "]";

  // visualize the current trajectory
  if (config_->visualize) planner_->visualize();

  return command;
}

}  // namespace lidar
}  // namespace vtr