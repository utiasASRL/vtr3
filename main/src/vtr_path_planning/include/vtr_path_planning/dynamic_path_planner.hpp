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
 * \file dynamic_path_planner.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once


#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/base_path_planner.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_path_planning/factory.hpp"

#include "geometry_msgs/msg/twist.hpp"

namespace vtr {
namespace path_planning {


class DynamicPathPlanner : public BasePathPlanner {
 public:
  PTR_TYPEDEFS(DynamicPathPlanner);

  using RobotState = tactic::OutputCache;
  using Command = geometry_msgs::msg::Twist;
  using Callback = PathPlannerCallbackInterface;

  /** \brief An unique identifier. Subclass should overwrite this. */
  static constexpr auto static_name = "dynamic_path_planner";

  struct Config : public BasePathPlanner::Config {
    PTR_TYPEDEFS(Config);

    virtual ~Config() = default;
    std::vector<std::string> planner_whitelist;
    std::string default_planner = "stationary";

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  DynamicPathPlanner(const Config::ConstPtr& config,
                  const RobotState::Ptr& robot_state,
                  const tactic::GraphBase::Ptr& graph, 
                  const Callback::Ptr& callback);
  ~DynamicPathPlanner() override;

  /// for state machine use
 public:
  /** \brief Sets whether control command should be computed */
  void setRunning(const bool running) override;

  void setController(const std::string& new_controller);

 protected:
  /** \brief Subclass override this method to compute a control command */
  Command computeCommand(RobotState& robot_state) override;

  /** \brief Derived class must call this upon destruction */
  void stop();

 private:
  const Config::ConstPtr config_;
  /** \brief shared memory that stores the current robot state */
  const RobotState::Ptr robot_state_;

  const tactic::GraphBase::Ptr graph_;
  /** \brief callback functions on control finished */
  const Callback::Ptr callback_;

  BasePathPlanner::Ptr active_planner_;
  ROSPathPlannerFactory::Ptr factory_;
  std::unordered_set<std::string> controllers_used_;
  VTR_REGISTER_PATH_PLANNER_DEC_TYPE(DynamicPathPlanner);

};

} //path_planning
} //vtr