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
 * \file base_path_planner.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "vtr_logging/logging.hpp"
#include "vtr_path_planning/path_planner_interface.hpp"
#include "vtr_tactic/cache.hpp"

#include "geometry_msgs/msg/twist.hpp"

namespace vtr {
namespace path_planning {

class PathPlannerCallbackInterface {
 public:
  PTR_TYPEDEFS(PathPlannerCallbackInterface);

  using Command = geometry_msgs::msg::Twist;

  virtual ~PathPlannerCallbackInterface() = default;

  virtual tactic::Timestamp getCurrentTime() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  /** \brief Callback when a new route is set */
  virtual void stateChanged(const bool& /* running */) {}

  /**
   * \brief Callback when the route is finished
   * \note command is not a class member of BasePathPlanner, so this function is
   * called without BasePathPlanner locked
   */
  virtual void commandReceived(const Command& /* command */) {}
};

class BasePathPlanner : public PathPlannerInterface {
 public:
  PTR_TYPEDEFS(BasePathPlanner);

  using Mutex = std::mutex;
  using CondVar = std::condition_variable;
  using LockGuard = std::lock_guard<Mutex>;
  using UniqueLock = std::unique_lock<Mutex>;

  using RobotState = tactic::OutputCache;
  using Command = geometry_msgs::msg::Twist;
  using Callback = PathPlannerCallbackInterface;

  /** \brief An unique identifier. Subclass should overwrite this. */
  static constexpr auto static_name = "base_path_planner";

  struct Config {
    PTR_TYPEDEFS(Config);

    unsigned int control_period = 0;

    virtual ~Config() = default;

    static Ptr fromROS(const rclcpp::Node::SharedPtr& node,
                       const std::string& prefix = "path_planning");
  };

  BasePathPlanner(const Config::ConstPtr& config,
                  const RobotState::Ptr& robot_state,
                  const tactic::GraphBase::Ptr& graph, 
                  const Callback::Ptr& callback);
  ~BasePathPlanner() override;

  /// for state machine use
 public:
  /** \brief Sets whether control command should be computed */
  void setRunning(const bool running) override;

 private:
  /** \brief Subclass override this method to compute a control command */
  virtual Command computeCommand(RobotState& robot_state) = 0;

 protected:
  /** \brief Subclass use this function to get current time */
  tactic::Timestamp now() const { return callback_->getCurrentTime(); }
  /** \brief Derived class must call this upon destruction */
  void stop();

 private:
  void process();

 private:
  const Config::ConstPtr config_;
  /** \brief shared memory that stores the current robot state */
  const RobotState::Ptr robot_state_;

  const tactic::GraphBase::Ptr graph_;
  /** \brief callback functions on control finished */
  const Callback::Ptr callback_;

 protected:
  /** \brief Protects all class members */
  mutable Mutex mutex_;
  /** \brief wait until stop or controller state has changed */
  mutable CondVar cv_terminate_or_state_changed_;
  mutable CondVar cv_waiting_;
  mutable CondVar cv_thread_finish_;

 protected:
  /** \brief Whether the processing thread should be computing command */
  bool running_ = false;
  /** \brief Whether the processing thread is waiting to run */
  bool waiting_ = true;

  /** \brief signal the process thread to stop */
  bool terminate_ = false;
  /** \brief used to wait until all threads finish */
  size_t thread_count_ = 0;
  /** \brief the event processing thread */
 private:
  std::thread process_thread_;

  /// factory handlers (note: local static variable constructed on first use)
 private:
  /** \brief a map from type_str trait to a constructor function */
  using CtorFunc = std::function<Ptr(
      const Config::ConstPtr&, const RobotState::Ptr&, const tactic::GraphBase::Ptr&, const Callback::Ptr&)>;
  using Name2Ctor = std::unordered_map<std::string, CtorFunc>;
  static Name2Ctor& name2Ctor() {
    static Name2Ctor name2ctor;
    return name2ctor;
  }

  /** \brief a map from type_str trait to a config from ROS function */
  using CfROSFunc = std::function<Config::ConstPtr(
      const rclcpp::Node::SharedPtr&, const std::string&)>;
  using Name2CfROS = std::unordered_map<std::string, CfROSFunc>;
  static Name2CfROS& name2Cfros() {
    static Name2CfROS name2cfros;
    return name2cfros;
  }

  template <typename T>
  friend class PathPlannerRegister;
  friend class PathPlannerFactory;
  friend class ROSPathPlannerFactory;
};

template <typename T>
struct PathPlannerRegister {
  PathPlannerRegister() {
    bool success = true;
    success &=
        BasePathPlanner::name2Ctor()
            .try_emplace(
                T::static_name,
                BasePathPlanner::CtorFunc(
                    [](const BasePathPlanner::Config::ConstPtr& config,
                       const BasePathPlanner::RobotState::Ptr& robot_state,
                       const tactic::GraphBase::Ptr& graph,
                       const BasePathPlanner::Callback::Ptr& callback) {
                      const auto& config_typed =
                          (config == nullptr
                               ? std::make_shared<const typename T::Config>()
                               : std::dynamic_pointer_cast<
                                     const typename T::Config>(config));
                      return std::make_shared<T>(config_typed, robot_state, graph,
                                                 callback);
                    }))
            .second;
    success &= BasePathPlanner::name2Cfros()
                   .try_emplace(T::static_name,
                                BasePathPlanner::CfROSFunc(
                                    [](const rclcpp::Node::SharedPtr& node,
                                       const std::string& prefix) {
                                      return T::Config::fromROS(node, prefix);
                                    }))
                   .second;
    if (!success)
      throw std::runtime_error{"PathPlannerRegister failed - duplicated name"};
  }
};

/// \brief Register a path planner
/// \todo probably need to add a dummy use of this variable for initialization
#define VTR_REGISTER_PATH_PLANNER_DEC_TYPE(NAME) \
  inline static vtr::path_planning::PathPlannerRegister<NAME> reg_
}  // namespace path_planning
}  // namespace vtr
