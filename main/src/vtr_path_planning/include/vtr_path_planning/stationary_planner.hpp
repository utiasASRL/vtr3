// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file stationary_planner.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_path_planning/base_path_planner.hpp>

namespace vtr {
namespace path_planning {

class StationaryPlanner : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(StationaryPlanner);
  using Command = geometry_msgs::msg::Twist;

  static constexpr auto static_name = "stationary";

  struct Config : public BasePathPlanner::Config {
    PTR_TYPEDEFS(Config);

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  StationaryPlanner(const Config::ConstPtr& config,
                 const RobotState::Ptr& robot_state,
                 const tactic::GraphBase::Ptr& graph,
                 const Callback::Ptr& callback);
  ~StationaryPlanner() override;

 protected:
  Command computeCommand(RobotState& robot_state) override;
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(StationaryPlanner);
  Config::ConstPtr config_;


};

}  // namespace path_planning
}  // namespace vtr
