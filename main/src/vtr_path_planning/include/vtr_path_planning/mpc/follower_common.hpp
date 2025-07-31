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
 * \file follower_common.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#pragma once

#include <vtr_tactic/types.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vtr_common/conversions/ros_lgmath.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mpc_common.hpp"

namespace vtr::path_planning
{
  using TransformList = std::vector<lgmath::se3::Transformation>;

class PathInterpolator {
public:
  PTR_TYPEDEFS(PathInterpolator);
  using Transformation = lgmath::se3::Transformation;


  PathInterpolator(const nav_msgs::msg::Path::SharedPtr& path);

  Transformation at(tactic::Timestamp time) const;
  tactic::Timestamp start() const;

private:
  std::map<tactic::Timestamp, Transformation> path_info_;
};

PoseResultHomotopy generateFollowerReferencePosesEuclidean(const TransformList& leader_world_poses, const double final_leader_p_value, const tactic::LocalizationChain::Ptr chain, double robot_p, double distance_target);

} // namespace vtr::path_planning
