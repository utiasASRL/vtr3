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
 * \file mpc_path_planner.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include "vtr_path_planning/base_path_planner.hpp"

namespace vtr {
namespace path_planning {

class MPCPathPlanner : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(MPCPathPlanner);

  static constexpr auto static_name = "mpc";

  MPCPathPlanner(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const Callback::Ptr& callback);
  ~MPCPathPlanner() override;

 private:
  void initializeRoute(RobotState& robot_state) override;
  void initializeRouteTest(RobotState& robot_state) override;
  Command computeCommand(RobotState& robot_state) override;

 private:
  void visualize(const tactic::Timestamp& stamp,
                 const tactic::EdgeTransform& T_w_p,
                 const tactic::EdgeTransform& T_p_r,
                 const tactic::EdgeTransform& T_p_g) const;

  // for rviz visualization
 private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;

  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(MPCPathPlanner);
};

}  // namespace path_planning
}  // namespace vtr
