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
 * \file navigator.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "rclcpp/rclcpp.hpp"

#include "vtr_navigation/graph_map_server.hpp"
#include "vtr_navigation/ros_mission_server.hpp"
#include "vtr_path_planning/path_planner_interface.hpp"
#include "vtr_route_planning/route_planner_interface.hpp"
#include "vtr_route_planning/bfs_planner.hpp" // Hshmat: for mapping following route ids to BFS edge blacklist
#include "vtr_tactic/tactic.hpp"
#include "vtr_navigation_msgs/msg/graph_route.hpp" //Hshmat: for mapping following route ids to BFS edge blacklist
#include "vtr_navigation_msgs/msg/server_state.hpp" // HSHMAT: for resetting obstacle state on Repeat start
#include "std_msgs/msg/bool.hpp" // Hshmat: for mapping following route ids to BFS edge blacklist
#include "std_msgs/msg/float64.hpp" // Hshmat: for obstacle distance subscription
#include "std_msgs/msg/string.hpp" // Hshmat: for ChatGPT decision subscription
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Core>
#include "vtr_navigation/wait_strategy.hpp"  // HSHMAT: Strategy pattern for wait time decisions
#include "vtr_navigation/real_world_logger.hpp"  // HSHMAT: Real-world episode/encounter logging
#include <unordered_map>
#include <limits>
#include <chrono> // Hshmat: for debouncing obstacle detection
#include <optional> // Hshmat: for debouncing obstacle detection
#include <mutex>
#include <atomic>  // HSHMAT: for speech_done_ flag

#ifdef VTR_ENABLE_VISION
#include "message_filters/subscriber.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/msg/image.hpp"
#endif

#include "vtr_common/conversions/tf2_ros_eigen.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#ifdef VTR_ENABLE_LIDAR
#include "sensor_msgs/msg/point_cloud2.hpp"
#endif

#ifdef VTR_ENABLE_RADAR
#include "navtech_msgs/msg/radar_b_scan_msg.hpp"
#include "sensor_msgs/msg/image.hpp"
#endif

#if defined(VTR_ENABLE_RADAR ) || defined(VTR_ENABLE_LIDAR)
#include "sensor_msgs/msg/imu.hpp"
#endif

namespace vtr {
namespace navigation {

/** \brief Remembered blockage interval for TDSP (seconds in ROS clock domain). */
struct EdgeBlockageInterval {
  double start_sec = 0.0;
  double end_sec = 0.0;
};

/**
 * \brief HSHMAT: Obstacle handling state machine.
 *
 * Provides a clearer view of the obstacle episode lifecycle, replacing the
 * previous "boolean soup" of flags.
 *
 * States:
 *   Idle      - No obstacle episode active. Robot may be moving normally.
 *   Waiting   - Obstacle detected, robot paused, waiting for obstacle to clear
 *               or timer to expire (W* seconds). Timer runs countdown.
 *   Rerouting - Timer expired (or immediate reroute), computing alternate route.
 *               Transitions to Idle after route change completes.
 */
enum class ObstacleState {
  Idle,
  AwaitingClassification,  // HSHMAT: Waiting for VLM to classify obstacle (rule_based/learned)
  Waiting,
  Rerouting
};

// HSHMAT: Helper to convert ObstacleState to string for logging
inline const char* obstacleStateToString(ObstacleState s) {
  switch (s) {
    case ObstacleState::Idle: return "Idle";
    case ObstacleState::AwaitingClassification: return "AwaitingClassification";
    case ObstacleState::Waiting: return "Waiting";
    case ObstacleState::Rerouting: return "Rerouting";
  }
  return "Unknown";
}

class Navigator {
 public:
  using Mutex = std::mutex;
  using LockGuard = std::lock_guard<Mutex>;
  using UniqueLock = std::unique_lock<Mutex>;

  Navigator(const rclcpp::Node::SharedPtr &node);
  ~Navigator();

  void process();

 private:
  /** \brief ROS-handle for communication */
  const rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_sbc_;

  /// TF buffer/listener for coordinate transforms (e.g., map -> lidar)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// robot and sensor specific stuff, these are set in constructor, so no need
  /// to lock
  rclcpp::CallbackGroup::SharedPtr callback_group_;           // HSHMAT: For obstacle FSM logic (mutually exclusive)
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;    // HSHMAT: For sensors (reentrant, never blocked)
  rclcpp::CallbackGroup::SharedPtr obstacle_callback_group_;  // HSHMAT: For non-critical obstacle subs
  // robot
  const std::string &robot_frame() const { return robot_frame_; }
  std::string robot_frame_;
  // environment info
  void envInfoCallback(const tactic::EnvInfo::SharedPtr msg);
  rclcpp::Subscription<tactic::EnvInfo>::SharedPtr env_info_sub_;
#ifdef VTR_ENABLE_LIDAR
  // lidar
  const std::string &lidar_frame() const { return lidar_frame_; }
  const tactic::EdgeTransform &T_lidar_robot() const { return T_lidar_robot_; }
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  std::string lidar_frame_;
  tactic::EdgeTransform T_lidar_robot_;
#endif

#ifdef VTR_ENABLE_RADAR 
  // radar
  const std::string &radar_frame() const { return radar_frame_; }
  const tactic::EdgeTransform &T_radar_robot() const { return T_radar_robot_; }
  void radarCallback(const navtech_msgs::msg::RadarBScanMsg::SharedPtr msg);
  rclcpp::Subscription<navtech_msgs::msg::RadarBScanMsg>::SharedPtr radar_sub_;
  std::string radar_frame_;
  tactic::EdgeTransform T_radar_robot_;
#endif
 
#if defined(VTR_ENABLE_RADAR ) || defined(VTR_ENABLE_LIDAR)
  // gyro
  using ImuMsg = sensor_msgs::msg::Imu;
  const std::string &gyro_frame() const { return gyro_frame_; }
  const tactic::EdgeTransform &T_gyro_robot() const { return T_gyro_robot_; }
  void gyroCallback(const ImuMsg::SharedPtr msg);
  rclcpp::Subscription<ImuMsg>::SharedPtr gyro_sub_;
  std::string gyro_frame_;
  tactic::EdgeTransform T_gyro_robot_;
  std::vector<ImuMsg> gyro_msgs_;
  std::array<double, 3> gyro_bias_ = {0.0, 0.0, 0.0};
#endif

#ifdef VTR_ENABLE_VISION
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image
  > ApproximateImageSync;


  const std::string &camera_frame() const { return camera_frame_;}
  const tactic::EdgeTransform &T_camera_robot() const { return T_camera_robot_; }
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg_r, const sensor_msgs::msg::Image::SharedPtr msg_l);
  message_filters::Subscriber<sensor_msgs::msg::Image> right_camera_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_camera_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateImageSync>> sync_;
  std::string camera_frame_;
  tactic::EdgeTransform T_camera_robot_;
#endif

 private:
  /** \brief protects: event_, goals_, stop_, trigger_success_ */
  mutable Mutex mutex_;
  /** \brief wait until the queue is full or stop */
  mutable std::condition_variable cv_set_or_stop_;
  /** \brief wait until the process thread has finished */
  mutable std::condition_variable cv_thread_finish_;

  std::queue<tactic::QueryCache::Ptr> queue_;
  int max_queue_size_ = 5;
  tactic::EnvInfo env_info_;
  
  /// VTR building blocks
  GraphMapServer::Ptr graph_map_server_;
  tactic::Graph::Ptr graph_;
  tactic::Tactic::Ptr tactic_;
  path_planning::PathPlannerInterface::Ptr path_planner_;
  route_planning::RoutePlannerInterface::Ptr route_planner_;
  ROSMissionServer::Ptr mission_server_;
  mission_planning::StateMachine::Ptr state_machine_;

  // HSHMAT: Obstacle status subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_status_sub_;
  // Hshmat: Obstacle distance subscriber (distance along path to nearest obstacle)
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_distance_sub_;
  double last_obstacle_distance_;  // meters along path where obstacle detected
  // Hshmat: Following route subscriber (for mapping path to vertex ids)
  rclcpp::Subscription<vtr_navigation_msgs::msg::GraphRoute>::SharedPtr following_route_sub_;
  std::vector<uint64_t> following_route_ids_;
  // Hshmat: Occupancy grid from path obstacle detector (red cells = on-path obstacles)
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_grid_sub_;
  nav_msgs::msg::OccupancyGrid last_obstacle_grid_;
  // Hshmat: Obstacle type (e.g., "person", "chair") from VLM/decision node
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_type_sub_;
  std::string last_obstacle_type_ = "unknown";
  
  // HSHMAT: FSM state - single source of truth for obstacle episode lifecycle
  Mutex obstacle_mutex_;
  ObstacleState obstacle_state_ = ObstacleState::Idle;
  bool last_obstacle_status_msg_ = false;  // Raw sensor value
  rclcpp::Time obstacle_start_time_;  // When current episode started
  
  // HSHMAT: Wait strategy - determines W* (how long to wait before rerouting)
  std::unique_ptr<WaitStrategy> wait_strategy_;
  WaitStrategyConfig wait_strategy_config_;
  double current_W_star_ = 0.0;  // Current wait time limit (seconds)
  
  // HSHMAT: Wait timer for countdown
  rclcpp::TimerBase::SharedPtr wait_timer_;

  std::vector<int> countdown_intervals_;  // Announce at these seconds remaining
  int next_countdown_idx_ = 0;  // Index into countdown_intervals_
  void onWaitTimerTick();  // Called every second during waiting
  
  // HSHMAT: Reroute detection - save route snapshot when reroute requested
  std::vector<uint64_t> reroute_snapshot_route_;  // Full route when reroute was requested
  bool awaiting_new_route_ = false;               // True after triggerReroute until new route arrives
  bool no_alternate_exists_ = false;              // True when planner determined no alternate route
  bool announcing_no_alternate_ = false;          // True while speaking "No alternate path" (blocks handleObstacleCleared)
  rclcpp::Time reroute_complete_time_;            // When last successful reroute completed (for 500ms cooldown)
  
  void handleObstacleCleared(ObstacleState previous_state);   // Handle obstacle cleared event (FSM dispatch)
  void onRerouteComplete(bool alternate_found);   // Handle successful reroute 
  void onNoAlternateRoute();                      // Handle when no alternate route exists
  
  // HSHMAT: Reroute status subscription (for detecting planner failures)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reroute_status_sub_;
  
  // HSHMAT: Server state subscriber - reset obstacle state on Repeat start
  rclcpp::Subscription<vtr_navigation_msgs::msg::ServerState>::SharedPtr server_state_sub_;
  uint8_t last_goal_state_ = 0;
  void resetObstacleState();
  
  // HSHMAT: Publisher for pausing robot (zero velocity)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pause_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_state_pub_;
  bool robot_paused_ = false;
  
  // HSHMAT: Speech publisher (announces obstacle events)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr speech_done_sub_;
  std::atomic<bool> speech_done_{true};
  void speak(const std::string& text);
  void speakAndWait(const std::string& text, double timeout_sec = 10.0);
  
  // HSHMAT: Publisher to tell Python decision node whether to use ChatGPT.
  // false for always_wait, always_detour, greedy_ctp (Navigator handles all logic)
  // true for rule_based, learned (need obstacle type from ChatGPT)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr use_chatgpt_pub_;
  
  // HSHMAT: Episode management
  // Idle -> Waiting/Rerouting. `precomputed` executes an already-planned
  // decision (SPARROW junction arrivals: one epoch = one plan; re-planning
  // here would break committed learning macros and sim parity).
  void startObstacleEpisode(const WaitDecision* precomputed = nullptr);
  void onObstacleCleared();     // Obstacle cleared during Waiting
  void onWaitTimeout();         // W* expired, Waiting -> Rerouting
  void completeEpisode();       // Episode ends -> Idle
  
  // HSHMAT: Graph access for Learned strategy
  void setupLearnedStrategyGraphAccess();  // Set up TDSP callbacks
  EdgeIdSet computeBlockedEdges() const;   // Get edges blocked by current obstacle

  // Cached privileged (teach) subgraph, shared by all graph queries
  // (adjacent-edge checks, corridors, junction scans). Created lazily,
  // invalidated on obstacle-state reset (new repeat).
  tactic::GraphBase::Ptr privilegedGraph() const;
  mutable tactic::GraphBase::Ptr cached_priv_graph_;

  // HSHMAT SPARROW: observed status of edges incident to a vertex.
  // 1 = blocked, 0 = free, -1 = unknown (out of sensor range / no TF).
  // Combines the current detection (current_blocked_edges_) with a corridor
  // check of each adjacent teach edge against the obstacle costmap (which is
  // built from the lidar pointcloud by vtr_path_obstacle_detector).
  std::map<tactic::EdgeId, int> computeAdjacentEdgeStatuses(
      const tactic::VertexId& v) const;
  // Check the corridor leaving `v` towards `n` against the obstacle costmap.
  // Returns 1 blocked / 0 free / -1 unknown.
  int checkEdgeCorridorInCostmap(const tactic::VertexId& v,
                                 const tactic::VertexId& n) const;
  // Sample teach-corridor points along edge (va,vb) in the loc-vertex frame,
  // starting from the endpoint closer to the robot and continuing down the
  // degree-2 chain up to length_m. Shared by the costmap edge check and the
  // observe-corridor publisher.
  bool computeEdgeCorridorPoints(uint64_t va, uint64_t vb, double length_m,
                                 std::vector<Eigen::Vector3d>& pts) const;
  // HSHMAT SPARROW: tell the detector which corridor the Observe action
  // targets (it builds a mask restricted to that corridor for the VLM).
  void publishObserveCorridor(const std::pair<uint64_t, uint64_t>& edge) const;
  void clearObserveCorridor() const;
  // Strategy accessors that work for both LEARNED and SPARROW.
  GlobalObstacleStats* strategyObstacleStats() const;
  double strategyFreshEdgeExpectedWait() const;
  bool strategyLearnsStats() const;  // LEARNED or SPARROW
  tactic::VertexId getCurrentVertex() const;  // Get current robot position
  tactic::VertexId getGoalVertex() const;     // Get goal vertex

  /** Index of \p v in following_route_ids_ (first match), or -1. Caller must hold obstacle_mutex_ if concurrent. */
  int followingRouteIndexOf(const tactic::VertexId& v) const;
  
  /** \brief True if remaining path from current vertex to goal is the same in both routes.
   * E.g. snapshot [1,2,3,4], new_route [3,4], current at 3 -> same. No 3s timeout. */
  bool routesSameRemaining(
      const std::vector<uint64_t>& snapshot,
      const std::vector<uint64_t>& new_route,
      const tactic::VertexId& current_vid) const;
  
  // Current blocked edges (computed in startObstacleEpisode)
  EdgeIdSet current_blocked_edges_;

  // HSHMAT SPARROW: adjacent-edge costmap check knobs
  bool sparrow_use_costmap_edge_check_ = true;
  double sparrow_edge_check_length_m_ = 2.5;
  double sparrow_edge_check_radius_m_ = 0.4;

  // HSHMAT SPARROW: on-demand VLM classification (Observe action).
  // The POMCP chose Observe: we published /vtr/request_classification, entered
  // AwaitingClassification, and when /vtr/obstacle_type arrives
  // startObstacleEpisode() re-plans WITHOUT redoing episode bookkeeping.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_classification_pub_;
  // Corridor of the edge being observed, for the detector's observe mask.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr observe_corridor_pub_;
  bool sparrow_observe_pending_ = false;
  // Canonical (min,max) vertex pair of the edge the Observe action targeted,
  // and when the request went out (to log the measured VLM latency - the
  // real-world analogue of delta_obs_s in the search).
  std::pair<uint64_t, uint64_t> sparrow_observe_edge_{0, 0};
  double sparrow_observe_request_sec_ = 0.0;
  // Extra micro-edges to ban in the next SPARROW reroute so the executed
  // route starts with the corridor the POMCP's Traverse action chose.
  std::vector<std::pair<uint64_t, uint64_t>> sparrow_detour_bans_;

  // HSHMAT SPARROW: continuous edge monitoring + every-decision replanning
  // (paper Table VII: replanning at every decision epoch, not only when the
  // route is blocked). Monitoring is driven by the detector's occupancy
  // (costmap) updates - throttled to monitor_period_s - and keeps the
  // strategy's per-edge sighting streaks fresh. As the robot approaches a
  // decision vertex (junction), it plans WHILE STILL MOVING: "continue"
  // costs no stop; a different corridor becomes an on-the-fly reroute;
  // Wait/Observe pause the robot and start a standard episode. Re-plans are
  // re-armed by the strategy's belief revision counter (edge transitions).
  void onSparrowOccupancyUpdate();
  double sparrow_last_monitor_sec_ = 0.0;
  // Junction the last en-route plan was made for, and the belief revision it
  // used (a revision bump re-arms the same junction within the lookahead).
  uint64_t sparrow_junction_handled_ = 0;
  uint64_t sparrow_junction_planned_revision_ = 0;
  // Belief revision when the current SPARROW wait started; a bump while
  // waiting means new information arrived (an edge transitioned) -> the wait
  // is cut short so the planner can reconsider with the new belief.
  uint64_t sparrow_wait_revision_baseline_ = std::numeric_limits<uint64_t>::max();
  // True while the current episode was triggered at a junction (adjacent
  // blocked edges, no detector front obstacle). Guards: startObstacleEpisode
  // keeps the injected blocked set, the detector's continuous CLEARED stream
  // is ignored (it cannot see adjacent edges), and clearance comes from the
  // costmap monitor instead.
  bool sparrow_junction_encounter_ = false;
  
  // Learned p_block: index along stored following_route_ids_. On obstacle or mission end: add (idx - last_path_index_), then last_path_index_=idx.
  // New repeat -> 0. Each following_route that replaces the path -> re-anchor to current vertex index (reroute included).
  int last_path_index_ = 0;
  // Total vertices in following_route when first received (edges = this - 1). Preserved even if route clears at mission end.
  int episode_route_size_ = 0;
  // Buffered edge traversals to flush at episode end (keeps p_block constant within episode)
  int pending_edge_traversals_ = 0;

  /** Track edges traversed (buffer only, don't update stats until episode end). */
  void recordLearnedEdgeProgressUpToIndex(int idx_in_route);
  /** At goal finish: flush all buffered updates (edges, KM, type counts). */
  void flushLearnedEdgeTraversalsForEpisode();
  
  void setRobotPaused(bool paused);
  void triggerReroute();  // Centralized reroute flow
  // Estimate obstacle extent (meters) from the latest occupancy grid.
  double estimateObstacleExtentFromGrid() const;

  struct RoutePlanningConfig {
    bool enable_reroute = false;
    double nominal_speed_mps = 0.5;
    // Radius (m) for grid-based screening of edges when building obstacle delays
    double screening_lookahead_m = 0.0;
    // Radius (m) for pruning remembered blockages when grid shows edge is clear
    double verify_blockage_lookahead_m = 0.0;
  };
  RoutePlanningConfig route_cfg_;
  // Newest-overrides semantics: on a new detection for an edge, we overwrite the interval.
  // Used only for non-LEARNED strategies (LEARNED uses EWTDSPPlanner with survival model).
  std::unordered_map<vtr::tactic::EdgeId, EdgeBlockageInterval> edge_blockages_;

  // Wait episode tracking (used for survival model updates)
  double wait_episode_start_sec_ = -1.0;  // <0 means inactive
  std::string wait_episode_type_ = "unknown";

  // Lookahead radii are configured via route_planning.* parameters (YAML).

  // HSHMAT: Debouncing for obstacle detection
  std::optional<std::chrono::steady_clock::time_point> last_obstacle_time_;

  // =========================================================================
  // HSHMAT: Real-world episode logging
  // =========================================================================
  std::unique_ptr<RealWorldLogger> real_world_logger_;
  int current_episode_ = 0;                         // Episode counter (per-run)
  rclcpp::Time mission_start_time_;                 // When current mission started
  double mission_time_limit_ = 2000.0;              // Time limit for success/failure
  int episode_reroute_count_ = 0;                   // Reroutes in current episode
  double episode_wait_time_ = 0.0;                  // Total wait time in current episode
  std::map<std::string, int> episode_encounter_counts_;  // Per-type encounter counts
  std::string graph_name_;                          // Graph name from config
  int run_idx_ = 1;                                 // Run index from config
  std::string logging_output_dir_;                  // Output directory for logging
  bool episode_finalized_ = false;                  // Prevent double-finalization on goal finish
  
  // Per-encounter tracking (for computing duration)
  double current_encounter_start_sec_ = -1.0;       // When current encounter started
  std::string current_encounter_blocked_edges_;     // Blocked edges for current encounter
  
  // HSHMAT: Deadman-based timing (time_to_goal = deadman release - deadman press)
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  bool deadman_pressed_ = false;                    // Current deadman state
  rclcpp::Time deadman_press_time_;                 // When deadman was first pressed this episode
  rclcpp::Time deadman_release_time_;               // When deadman was last released
  bool deadman_timing_valid_ = false;               // True if we have valid press time for this episode
  
  // Helper to format EdgeIdSet as string for logging
  std::string formatBlockedEdges(const EdgeIdSet& edges) const;
  
  // Called when goal reaches FINISHING state
  void onGoalFinishing();

  /// Threading
  bool stop_ = false;
  size_t thread_count_ = 0;
  /** \brief the event processing thread */
  std::thread process_thread_;
};

}  // namespace navigation
}  // namespace vtr
