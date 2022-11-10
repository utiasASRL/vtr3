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

#include "vtr_path_planning/cbit/cbit.hpp"


namespace vtr {
namespace lidar {

class LidarCBIT : public vtr::path_planning::CBIT {
 public:
  PTR_TYPEDEFS(LidarCBIT);

  static constexpr auto static_name = "cbit.lidar";

  struct Config : public path_planning::CBIT::Config {
    PTR_TYPEDEFS(Config);
    //
    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  LidarCBIT(const Config::ConstPtr& config,
                      const RobotState::Ptr& robot_state,
                      const Callback::Ptr& callback);
  ~LidarCBIT() override;

 private:
  Command computeCommand(RobotState& robot_state) override;

 private:
  const Config::ConstPtr config_;

  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(LidarCBIT);


  unsigned int prev_costmap_sid = 0;
  tactic::Timestamp prev_stamp;


  // Store the previously applied velocity and a sliding window history of MPC results
  Eigen::Matrix<double, 2, 1> applied_vel;
  std::vector<Eigen::Matrix<double, 2, 1>> vel_history;

  //create vector to store the robots path for visualization purposes
  std::vector<lgmath::se3::Transformation> robot_poses;
};

}  // namespace lidar
}  // namespace vtr