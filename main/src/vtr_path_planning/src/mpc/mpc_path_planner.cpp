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
 * \file mpc_path_planner.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/mpc/mpc_path_planner.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

namespace vtr {
namespace path_planning {

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

MPCPathPlanner::MPCPathPlanner(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback) {
  const auto node = robot_state->node.ptr();
  // visualization
  tf_bc_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

MPCPathPlanner::~MPCPathPlanner() { stop(); }

void MPCPathPlanner::initializeRoute(RobotState& robot_state) {
  /// \todo reset any internal state
}

auto MPCPathPlanner::computeCommand(RobotState& robot_state) -> Command {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(INFO, "path_planning") << "Robot is not localized, stop the robot";
    return Command();
  }

  // T_r_p = T_robot_planning, T_world_planning, T_plannning_goal
  // planning frame is the current localization frame
  auto [T_p_r, T_w_p, T_p_g, curr_sid, target_sid] = [&]() {
    auto lock = chain.guard();
    const auto T_p_r = chain.T_leaf_trunk().inverse();
    const auto T_w_p = chain.T_start_trunk();
    const auto curr_sid = chain.trunkSequenceId();
    const auto target_sid = std::min((size_t)(curr_sid + 2), chain.size() - 1);
    const auto T_w_g = chain.pose(target_sid);
    const auto T_p_g = T_w_p.inverse() * T_w_g;
    return std::make_tuple(T_p_r, T_w_p, T_p_g, curr_sid, target_sid);
  }();

  // project onto 2D plane w.r.t. the planning frame
  const auto T_p_r_2d = xyth2T(T2xyth(T_p_r));
  const auto T_p_g_2d = xyth2T(T2xyth(T_p_g));

  CLOG(INFO, "path_planning") << "compute command: current robot to world "
                              << (T_w_p * T_p_r).vec().transpose();
  // visualization
  visualize(0, T_w_p, T_p_r_2d, T_p_g_2d);

  return Command();
}

void MPCPathPlanner::visualize(const tactic::Timestamp& stamp,
                               const tactic::EdgeTransform& T_w_p,
                               const tactic::EdgeTransform& T_p_r,
                               const tactic::EdgeTransform& T_p_g) const {
  /// Publish the current frame for planning
  {
    Eigen::Affine3d T(T_w_p.matrix());
    auto msg = tf2::eigenToTransform(T);
    msg.header.stamp = rclcpp::Time(stamp);
    msg.header.frame_id = "world";
    msg.child_frame_id = "planning frame";
    tf_bc_->sendTransform(msg);
  }

  /// Publish the current robot in the planning frame
  {
    Eigen::Affine3d T2(T_p_r.matrix());
    auto msg2 = tf2::eigenToTransform(T2);
    msg2.header.frame_id = "planning frame";
    msg2.header.stamp = rclcpp::Time(stamp);
    msg2.child_frame_id = "robot planning";
    tf_bc_->sendTransform(msg2);
  }

  /// Publish the current target in the planning frame
  {
    Eigen::Affine3d T3(T_p_g.matrix());
    auto msg3 = tf2::eigenToTransform(T3);
    msg3.header.frame_id = "planning frame";
    msg3.header.stamp = rclcpp::Time(stamp);
    msg3.child_frame_id = "goal planning";
    tf_bc_->sendTransform(msg3);
  }
}

}  // namespace path_planning
}  // namespace vtr