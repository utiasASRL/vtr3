#pragma once

#include <memory>
#include <atomic>
#include <future>

#include <geometry_msgs/TwistStamped.h>

#include <asrl/pose_graph/id/VertexId.hpp>
#include <asrl/common/timing/TimeUtils.hpp>
#include <asrl/pose_graph/path/LocalizationChain.hpp>

#include <lgmath.hpp>
#include <steam/trajectory/SteamTrajInterface.hpp>

namespace vtr {

namespace asrl::pose_graph {
  class RCGraph;
}

namespace path_tracker {

// common type shortcuts
typedef ::asrl::pose_graph::LocalizationChain Chain;
typedef ::asrl::pose_graph::RCGraph Graph;
typedef lgmath::se3::Transformation Tf;
typedef lgmath::se3::TransformationWithCovariance TfCov;
typedef ::asrl::pose_graph::VertexId Vid;
typedef ::asrl::pose_graph::RCVertex Vertex;
typedef ::asrl::pose_graph::RCRun::IdType RunId;
typedef ::asrl::common::timing::time_point Stamp;
typedef ::asrl::common::timing::clock Clock;

/// Path tracking state for external control of the path tracker
enum class State {
  STOP,   ///< thread exits, path aborted
  PAUSE,  ///< skip control loop, path kept
  RUN,    ///< control output, follow path
};

// Output types
class SafetyStatus;
typedef geometry_msgs::TwistStamped Command;

/// Base path tracker class that implements a generic path tracker interface
class Base {
 public:

  /// Construct the base path tracker
  Base(const std::shared_ptr<Graph> & graph, ///< pose graph
       double control_period_ms); ///< the period for the control loop

  /// Destruct the path tracker, stopping any active paths
  virtual ~Base();

  /// Start a new thread that will follow the path
  void followPathAsync(const State & state, ///< initial following state
                       Chain& chain); ///< path to follow

  virtual void finishControlLoop();
  /// load configuration parameters and do any pre-processing.
  virtual void loadConfigs() = 0;

  /// Notify the path tracker about a new leaf (vision-based path localization)
  virtual void notifyNewLeaf(const Chain & chain, ///< ref to the loc. chain with the most recent pose estimate
                             Stamp leaf_stamp,///< The timestamp for the pose
                             Vid live_vid ///< Vid of the current vertex in the live run
                             ) = 0;

  /// Notify the path tracker about a new leaf including a STEAM trajectory.
  virtual void notifyNewLeaf(const Chain & chain,
                             const steam::se3::SteamTrajInterface & trajectory,
                             const Vid live_vid,
                             const uint64_t image_stamp) = 0;

  /// Pass a new IMU state to the path tracker
  virtual void updateImuState(const Eigen::Quaterniond & global_orientation,
                              const Eigen::Vector3d & global_accel,
                              const Stamp & stamp) { return; }

  /// Are we currently following a path?
  bool isRunning() {
    return control_loop_.valid() &&
        control_loop_.wait_for(std::chrono::milliseconds(0))
        != std::future_status::ready;
  }

  /// Set the path-following state for the active control loop
  ///
  /// Note: not thread-safe for multiple writers,
  /// only control the path tracker from a single thread.
  void setState(const State & state) {
    // don't change the state if we've already commanded a stop
    if (state_ != State::STOP) {
      std::lock_guard<std::mutex> lock(state_mtx_);
      state_ = state;
    }
  }

  /// Get the path-following state
  ///
  State getState() {
    return state_;
  }

  /// Stop the path tracker and wait for the thread to finish
  void stopAndJoin() {
    LOG(INFO) << "Path tracker stopping and joining";
    if (control_loop_.valid()) {
      setState(State::STOP);
      control_loop_.wait();
    }
  }

  /// Pause the path tracker but keep the current path
   void pause() {
     if (control_loop_.valid()) {
       LOG(INFO) << "Pausing the path tracker thread";
       setState(State::PAUSE);
       LOG(INFO) << "Path tracker paused";
     }
   }

   /// Resume the goal after a pause()
   void resume() {
     if (control_loop_.valid()) {
       LOG(INFO) << "Resuming the path tracker thread";
       setState(State::RUN);
       LOG(INFO) << "Path tracker thread resumed";
     }
   }

   /// Constructor method for the factory
   static std::shared_ptr<Base> Create();

   /// The future for the async control loop task
   std::future<void> control_loop_;

 protected:

  /// Follow the path specified by the chain.
  ///
  /// This is the synchronous implementation of the control loop
  void controlLoop();

  /// Sleep the remaining time in the control loop (currently fixed sleep)
  virtual void controlLoopSleep();

  /// This is the main control step that is left to derived classes
  virtual Command controlStep() = 0;

  /// Publish a drive command to the vehicle
  virtual void publishCommand(Command &command);

  // Reset before a new run.
  virtual void reset() = 0;

  /// The path tracker state (RUN/PAUSE/STOP) that guides the control loop
  std::atomic<State> state_;
  std::mutex state_mtx_;

  /// The control loop period in ms
  double control_period_ms_;
  ::asrl::common::timing::SimpleTimer step_timer_;
  /// The latest command produces by the controlStep
  Command latest_command_;

  /// The last time an update was received from the safety monitor
  Stamp t_last_safety_monitor_update_;

  /// The publisher for control messages (TODO)
  // std::shared_ptr<asrl__common__babelfish::Publisher<geometry_msgs::Twist>> publisher_;

  // Graph and localization chain
  std::shared_ptr<Chain> chain_;
  std::shared_ptr<const Graph> graph_;

};

}
}
