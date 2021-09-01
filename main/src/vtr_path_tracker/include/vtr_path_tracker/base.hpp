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
 * \file base.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <atomic>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <lgmath.hpp>
#include <steam/trajectory/SteamTrajInterface.hpp>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>
#include <vtr_pose_graph/path/localization_chain.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace vtr::pose_graph {
class RCGraph;
}

namespace vtr {
namespace path_tracker {

// common type shortcuts
using Chain = pose_graph::LocalizationChain;
using Graph = pose_graph::RCGraph;
using Tf = lgmath::se3::Transformation;
using TfCov = lgmath::se3::TransformationWithCovariance;
using Vid = pose_graph::VertexId;
using Vertex = pose_graph::RCVertex;
using RunId = pose_graph::RCRun::IdType;
using Stamp = common::timing::time_point;
using Clock = common::timing::clock;

/** \brief Path tracking state for external control of the path tracker */
enum class State {
  STOP,   ///< thread exits, path aborted
  PAUSE,  ///< skip control loop, path kept
  RUN,    ///< control output, follow path
};

// Output types
class SafetyStatus;
using TwistMsg = geometry_msgs::msg::Twist;
using Command = geometry_msgs::msg::TwistStamped;

/**
 * \brief Base path tracker class that implements a generic path tracker
 * interface
 */
class Base {
 public:
  /**
   * \brief Construct the base path tracker
   * \param graph The pose graph
   * \param node ROS2 node
   * \param param_prefix ROS2 parameter prefix for parameter loading
   */
  Base(const std::shared_ptr<Graph> &graph,
       const std::shared_ptr<rclcpp::Node> &node,
       const std::string &param_prefix);

  /** \brief Destruct the path tracker, stopping any active paths */
  virtual ~Base();

  /**
   * \brief Start a new thread that will follow the path
   * \param state Initial following state
   * \param chain The path to follow
   */
  void followPathAsync(const State &state, const Chain::Ptr &chain);

  /**
   * \brief Notify the path tracker about a new leaf (vision-based path
   * localization) Called by the Navigator's tactic
   * \param chain Ref to the loc. chain with the most recent pose estimate
   * \param leaf_stamp The timestamp for the pose
   * \param live_vid Vertex ID of the current vertex in the live run
   */
  virtual void notifyNewLeaf(const Chain::ConstPtr &chain, Stamp leaf_stamp,
                             Vid live_vid) = 0;
  // todo (Ben): the parameter list of the above and below should be more
  // similar

  /** \brief Notify the path tracker about a new leaf including a STEAM
   * trajectory. */
  virtual void notifyNewLeaf(const Chain::ConstPtr &chain,
                             const steam::se3::SteamTrajInterface &trajectory,
                             const Vid live_vid,
                             const uint64_t image_stamp) = 0;

  /**
   * \brief Determine if we are currently following a path
   * \return True if following path
   */
  bool isRunning() {
    return control_loop_.valid() &&
           control_loop_.wait_for(std::chrono::milliseconds(0)) !=
               std::future_status::ready;
  }

  /**
   * \brief Set the path-following state for the active control loop
   * \note Not thread-safe for multiple writers, only control the path tracker
   * from a single thread.
   */
  void setState(const State &state) {
    // don't change the state if we've already commanded a stop
    if (state_ != State::STOP) {
      std::lock_guard<std::mutex> lock(state_mtx_);
      state_ = state;
    }
  }

  /** \brief Get the path-following state */
  State getState() { return state_; }

  /** \brief Stop the path tracker and wait for the thread to finish */
  void stopAndJoin() {
    CLOG(INFO, "path_tracker") << "Path tracker stopping and joining";
    if (control_loop_.valid()) {
      setState(State::STOP);
      control_loop_.wait();
    }
  }

  /** \brief Pause the path tracker but keep the current path */
  void pause() {
    if (control_loop_.valid()) {
      CLOG(INFO, "path_tracker") << "Pausing the path tracker thread";
      setState(State::PAUSE);
      CLOG(INFO, "path_tracker") << "Path tracker paused";
    }
  }

  /** \brief Resume the goal after pause() called */
  void resume() {
    if (control_loop_.valid()) {
      CLOG(INFO, "path_tracker") << "Resuming the path tracker thread";
      setState(State::RUN);
      CLOG(INFO, "path_tracker") << "Path tracker thread resumed";
    }
  }

  /** \brief Constructor method for the factory */
  static std::shared_ptr<Base> Create();

  /** \brief The future for the async control loop task */
  std::future<void> control_loop_;

 protected:
  /**
   * \brief Follow the path specified by the chain
   * This is the synchronous implementation of the control loop
   */
  void controlLoop();

  /** \brief Load configuration parameters and do any pre-processing */
  virtual void loadConfigs() = 0;

  /** \brief This is the main control step that is left to derived classes */
  virtual Command controlStep() = 0;

  /**
   * \brief Sleep the remaining time in the control loop if using a fixed
   * control rate
   */
  void controlLoopSleep();

  /** \brief  */
  virtual void finishControlLoop();

  /** \brief Publish a drive command to the vehicle */
  virtual void publishCommand(Command &command);

  /** \brief Reset before a new run */
  virtual void reset() = 0;

  /** \brief A pointer to the Navigator node */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief ROS2 publisher for velocity command */
  rclcpp::Publisher<TwistMsg>::SharedPtr publisher_;

  /** \brief Namespace for parameters. e.g. node namespace + "/path_tracker" */
  std::string param_prefix_;

  /**
   * \brief The path tracker state (RUN/PAUSE/STOP) that guides the control
   * loop
   */
  std::atomic<State> state_;

  /** \brief Mutex to lock path-tracker state while we calculate control step */
  std::mutex state_mtx_;

  /** \brief Whether to use a fixed control rate */
  bool use_fixed_ctrl_rate_;

  /** \brief The control loop period in ms  */
  double control_period_ms_;

  /** \brief Timer to track how long control step takes */
  common::timing::SimpleTimer step_timer_;

  /** \brief The latest command produces by the controlStep  */
  Command latest_command_;

  /** \brief The localization chain  */
  Chain::Ptr chain_;

  /** \brief The pose graph  */
  std::shared_ptr<const Graph> graph_;
};

}  // namespace path_tracker
}  // namespace vtr
