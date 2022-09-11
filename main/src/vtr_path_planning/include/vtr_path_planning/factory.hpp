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
 * \file factory.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/base_path_planner.hpp"

namespace vtr {
namespace path_planning {

/** \brief constructs a path planner based on a type_str trait */
class PathPlannerFactory
    : public std::enable_shared_from_this<PathPlannerFactory> {
 public:
  PTR_TYPEDEFS(PathPlannerFactory);

  /**
   * \brief constructs a new or gets a cached path planner
   * \param token the token used to get the type_str trait (static name) of the
   * path planner to construct
   * \return a shared_ptr to the constructed path planner
   */
  virtual BasePathPlanner::Ptr get(
      const std::string& token,
      const BasePathPlanner::RobotState::Ptr& robot_state,
      const BasePathPlanner::Callback::Ptr& callback,
      const BasePathPlanner::Config::ConstPtr& config = nullptr) {
    CLOG(DEBUG, "path_planning")
        << "Getting path planner with token: " << token;
    auto iter = cached_planners_.find(token);
    if (iter != cached_planners_.end()) {
      return iter->second;
    } else {
      auto planner = make(token, robot_state, callback, config);
      cached_planners_.emplace(std::make_pair(token, planner));
      return planner;
    }
  }

  /**
   * \brief makes the requested path planner matching the type_str trait
   * \return a base path planner pointer to the derived class, nullptr if not
   * found
   * \throw invalid_argument if the derived path planner couldn't be found
   */
  virtual BasePathPlanner::Ptr make(
      const std::string& token,
      const BasePathPlanner::RobotState::Ptr& robot_state,
      const BasePathPlanner::Callback::Ptr& callback,
      const BasePathPlanner::Config::ConstPtr& config = nullptr) {
    const auto type_str = getTypeStr(token);
    CLOG(DEBUG, "path_planning")
        << "Constructing path planner with static name: " << type_str;
    if (!BasePathPlanner::name2Ctor().count(type_str))
      throw std::invalid_argument(
          "PathPlannerFactory::make: path planner type_str not found: " +
          type_str);

    return BasePathPlanner::name2Ctor().at(type_str)(config, robot_state,
                                                     callback);
  }

 private:
  virtual std::string getTypeStr(const std::string& token) const {
    return token;
  }

 private:
  /** \brief a map from type_str trait to a path planner */
  std::unordered_map<std::string, BasePathPlanner::Ptr> cached_planners_;
};

/** \brief make a path planner based on ros configuration */
class ROSPathPlannerFactory : public PathPlannerFactory {
 public:
  /** \brief constructed with ros param info */
  ROSPathPlannerFactory(const rclcpp::Node::SharedPtr& node) : node_(node) {}

  BasePathPlanner::Ptr make(
      const std::string& param_prefix,
      const BasePathPlanner::RobotState::Ptr& robot_state,
      const BasePathPlanner::Callback::Ptr& callback,
      const BasePathPlanner::Config::ConstPtr& config = nullptr) override {
    const auto& type_str = getTypeStr(param_prefix);
    if (!BasePathPlanner::name2Ctor().count(type_str))
      throw std::invalid_argument(
          "PathPlanner::make: path planner type_str not found: " + type_str);

    const auto& config_typed =
        config == nullptr
            ? BasePathPlanner::name2Cfros().at(type_str)(node_, param_prefix)
            : config;
    return PathPlannerFactory::make(param_prefix, robot_state, callback,
                                    config_typed);
  }

 private:
  std::string getTypeStr(const std::string& param_prefix) const override {
    std::string param_name{param_prefix + "." + type_field_};
    auto type_str =
        node_->has_parameter(param_name)
            ? node_->get_parameter(param_name).get_value<std::string>()
            : node_->declare_parameter<std::string>(param_name, "");
    if (type_str.empty()) {
      auto msg = "No field: '" + param_name + "'";
      CLOG(ERROR, "path_planning") << msg;
      throw std::runtime_error(msg);
    }
    return type_str;
  }

 private:
  static constexpr auto type_field_ = "type";
  const rclcpp::Node::SharedPtr node_;
};

}  // namespace path_planning
}  // namespace vtr
