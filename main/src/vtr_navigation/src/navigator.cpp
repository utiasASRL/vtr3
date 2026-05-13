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
 * \file navigator.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_navigation/navigator.hpp"
#include "vtr_navigation/survival_model.hpp"
#include "vtr_navigation/wait_strategy.hpp"
#include "vtr_navigation/obstacle_memory.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_set>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <unistd.h>
#include <thread>  // HSHMAT: for speakAndWait sleep

#include "vtr_common/utils/filesystem.hpp"
#include "yaml-cpp/yaml.h"
#include "vtr_navigation/command_publisher.hpp"
#include "vtr_navigation/task_queue_server.hpp"
#include "vtr_path_planning/factory.hpp"
#include "vtr_tactic/pipelines/factory.hpp"

#include "vtr_path_planning/path_planning.hpp"
#include "vtr_route_planning/route_planning.hpp"

#ifdef VTR_ENABLE_LIDAR
#include "vtr_lidar/pipeline.hpp"
#endif

#ifdef VTR_ENABLE_RADAR
#include "vtr_radar/pipeline.hpp"
#endif
#ifdef VTR_ENABLE_VISION
#include "vtr_vision/pipeline.hpp"
#endif

namespace vtr {
namespace navigation {

using namespace vtr::tactic;
using namespace vtr::path_planning;
using namespace vtr::route_planning;
using namespace vtr::mission_planning;

namespace {

EdgeTransform loadTransform(const std::string& source_frame,
                            const std::string& target_frame) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf_buffer{clock};
  tf2_ros::TransformListener tf_listener{tf_buffer};
  if (tf_buffer.canTransform(source_frame, target_frame, tf2::TimePoint(),
                             tf2::durationFromSec(2))) {
    auto tf_source_target = tf_buffer.lookupTransform(
        source_frame, target_frame, tf2::TimePoint(), tf2::durationFromSec(2));
    tf2::Stamped<tf2::Transform> tf2_source_target;
    tf2::fromMsg(tf_source_target, tf2_source_target);
    EdgeTransform T_source_target(
        common::conversions::fromStampedTransformation(tf2_source_target));
    T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
    CLOG(DEBUG, "navigation")
        << "Transform from " << target_frame << " to " << source_frame
        << " has been set to" << T_source_target;
    return T_source_target;
  }
  CLOG(WARNING, "navigation")
      << "Transform not found - source: " << source_frame
      << " target: " << target_frame << ". Default to identity.";
  EdgeTransform T_source_target(Eigen::Matrix4d(Eigen::Matrix4d::Identity()));
  T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
  return T_source_target;
}

// Hshmat: Helper functions for obstacle priors
std::string nowStampForFilename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return ss.str();
}

// Hshmat: Helper functions for obstacle priors
std::string makeRuntimePriorsPathNextToBase(const std::string &base_path) {
  namespace fs = std::filesystem;
  const fs::path base(base_path);
  const fs::path dir = base.has_parent_path() ? base.parent_path() : fs::path(".");
  const int pid = static_cast<int>(::getpid());
  const std::string stamp = nowStampForFilename();
  const std::string fname = "obstacles_runtime_" + stamp + "_" + std::to_string(pid) + ".yaml";
  return (dir / fname).string();
}

// Hshmat: Helper functions for obstacle priors
bool writeObstaclePriorsYamlAtomic(const std::string &path,
                                  const double default_sec,
                                  const std::unordered_map<std::string, double> &priors_sec,
                                  std::string *err_msg = nullptr) {
  try {
    namespace fs = std::filesystem;
    const fs::path out_path(path);
    const fs::path tmp_path = out_path.string() + ".tmp";

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "default" << YAML::Value << default_sec;
    out << YAML::Key << "obstacles" << YAML::Value << YAML::BeginMap;
    for (const auto &kv : priors_sec) {
      out << YAML::Key << kv.first << YAML::Value << kv.second;
    }
    out << YAML::EndMap;  // obstacles
    out << YAML::EndMap;  // root

    std::ofstream f(tmp_path, std::ios::out | std::ios::trunc);
    if (!f.is_open()) {
      if (err_msg) *err_msg = "failed to open tmp file for write";
      return false;
    }
    f << out.c_str() << "\n";
    f.close();

    std::error_code ec;
    fs::rename(tmp_path, out_path, ec);
    if (ec) {
      fs::remove(tmp_path, ec);
      if (err_msg) *err_msg = "rename failed: " + ec.message();
      return false;
    }
    return true;
  } catch (const std::exception &e) {
    if (err_msg) *err_msg = e.what();
    return false;
  }
}

}  // namespace

Navigator::Navigator(const rclcpp::Node::SharedPtr& node) : node_(node) {
  el::Helpers::setThreadName("navigator");
  CLOG(INFO, "navigation") << "Starting VT&R3 system - hello!";

  /// TF buffer and listener for coordinate transforms
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  /// data storage directory (must have been set at this moment)
  auto data_dir = node_->get_parameter("data_dir").get_value<std::string>();
  data_dir = common::utils::expand_user(common::utils::expand_env(data_dir));
  CLOG(INFO, "navigation") << "Data directory set to: " << data_dir;

  /// graph map server (pose graph callback, tactic callback)
  // graph_map_server_ = std::make_shared<GraphMapServer>();
  graph_map_server_ = std::make_shared<RvizGraphMapServer>(node_);

  /// pose graph
  auto new_graph = node_->declare_parameter<bool>("start_new_graph", false);
  graph_ = tactic::Graph::MakeShared(data_dir + "/graph", !new_graph,
                                     graph_map_server_);
  graph_map_server_->start(node_, graph_);

  /// tactic
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node_);
  auto pipeline = pipeline_factory->get("pipeline");
  auto pipeline_output = pipeline->createOutputCache();
  pipeline_output->node = node_;  // some planners need node for visualization
  tactic_ = std::make_shared<Tactic>(Tactic::Config::fromROS(node_), pipeline,
                                     pipeline_output, graph_, graph_map_server_,
                                     std::make_shared<TaskQueueServer>(node_));
  if (graph_->contains(VertexId(0, 0))) tactic_->setTrunk(VertexId(0, 0));

  /// path planner
  auto planner_factory = std::make_shared<ROSPathPlannerFactory>(node_);
  path_planner_ =
      planner_factory->get("path_planning", pipeline_output,
                           std::make_shared<CommandPublisher>(node_));

  /// route planner
  // -------------------------------------------------------------------------
  // Route planning configuration (new schema).
  //
  // The key "if/else" lives here:
  // - If memory=false -> we construct a Dijkstra-like planner (BFSPlanner with time cost).
  // - If memory=true  -> we construct a FIFO time-dependent shortest path (TDSPPlanner).
  //
  // The state machine is planner-agnostic and always calls route_planner_->path(...).
  // -------------------------------------------------------------------------
  route_cfg_.enable_reroute =
        node_->declare_parameter<bool>("route_planning.enable_reroute", false);
  route_cfg_.memory =
      node_->declare_parameter<bool>("route_planning.memory", false);
  route_cfg_.update_obstacle_priors =
      node_->declare_parameter<bool>("route_planning.update_obstacle_priors", false);
  route_cfg_.obstacle_prior_update_alpha =
      node_->declare_parameter<double>("route_planning.obstacle_costs.update_alpha", 0.2);
  route_cfg_.obstacle_prior_update_min_sec =
      node_->declare_parameter<double>("route_planning.obstacle_costs.update_min_sec", 0.0);
  route_cfg_.obstacle_prior_update_max_sec =
      node_->declare_parameter<double>("route_planning.obstacle_costs.update_max_sec", 600.0);
  route_cfg_.planner_with_memory = node_->declare_parameter<std::string>(
      "route_planning.planner.with_memory", std::string("time_dependent_shortest_path"));
  route_cfg_.planner_without_memory = node_->declare_parameter<std::string>(
      "route_planning.planner.without_memory", std::string("dijkstra"));
  route_cfg_.nominal_speed_mps =
      node_->declare_parameter<double>("route_planning.nominal_speed_mps", 0.5);
  route_cfg_.verify_blockage_lookahead_m =
      node_->declare_parameter<double>("route_planning.verify_blockage_lookahead_m", 5.0);
  route_cfg_.screening_lookahead_m =
      node_->declare_parameter<double>("route_planning.screening_lookahead_m", 5.0);
  route_cfg_.obstacle_cost_source = node_->declare_parameter<std::string>(
      "route_planning.obstacle_costs.source", std::string("file"));
  route_cfg_.obstacle_cost_file = node_->declare_parameter<std::string>(
      "route_planning.obstacle_costs.file", std::string("obstacles.yaml"));

  CLOG(INFO, "navigation")
      << "Route planning config: enable_reroute=" << (route_cfg_.enable_reroute ? "true" : "false")
      << ", memory=" << (route_cfg_.memory ? "true" : "false")
      << ", planner.with_memory='" << route_cfg_.planner_with_memory << "'"
      << ", planner.without_memory='" << route_cfg_.planner_without_memory << "'"
      << ", nominal_speed_mps=" << route_cfg_.nominal_speed_mps
      << ", verify_blockage_lookahead_m=" << route_cfg_.verify_blockage_lookahead_m
      << ", screening_lookahead_m=" << route_cfg_.screening_lookahead_m
      << ", obstacle_costs.source='" << route_cfg_.obstacle_cost_source << "'"
      << ", obstacle_costs.file='" << route_cfg_.obstacle_cost_file << "'"
      << ", update_obstacle_priors=" << (route_cfg_.update_obstacle_priors ? "true" : "false")
      << ", obstacle_costs.update_alpha=" << route_cfg_.obstacle_prior_update_alpha
      << ", obstacle_costs.update_min_sec=" << route_cfg_.obstacle_prior_update_min_sec
      << ", obstacle_costs.update_max_sec=" << route_cfg_.obstacle_prior_update_max_sec;

  // HSHMAT: Always use TDSP planner - it supports both modes:
  // - Memoryless mode: Only static_delays, no time intervals (like Dijkstra)
  // - Memory mode: Time-dependent blockages that expire (for LEARNED strategy only)
  // The strategy type determines which mode to use at runtime in triggerReroute().
  auto tdsp = std::make_shared<vtr::route_planning::TDSPPlanner>(graph_);
  tdsp->setNominalSpeed(route_cfg_.nominal_speed_mps);
  tdsp->setNowSecCallback([this]() { return node_->get_clock()->now().seconds(); });
  route_planner_ = tdsp;
  
  CLOG(INFO, "navigation")
      << "HSHMAT: Using TDSP planner (memoryless/memory mode selected by strategy at runtime)";

  if (route_cfg_.obstacle_cost_source == "file") {
    loadObstaclePriorsFromFile();
    // If enabled, create a per-launch runtime priors file next to the base YAML.
    // We will write updated priors to this runtime file (base file is never modified).
    if (route_cfg_.update_obstacle_priors) {
      obstacle_priors_runtime_path_.clear();
      const std::string runtime_path = makeRuntimePriorsPathNextToBase(obstacle_priors_base_path_);
      std::string err;
      if (writeObstaclePriorsYamlAtomic(runtime_path, obstacle_default_prior_sec_, obstacle_priors_sec_, &err)) {
        obstacle_priors_runtime_path_ = runtime_path;
    CLOG(INFO, "navigation")
            << "Obstacle priors updater enabled: base='" << obstacle_priors_base_path_
            << "', runtime='" << obstacle_priors_runtime_path_ << "'";
      } else {
        CLOG(WARNING, "navigation")
            << "Obstacle priors updater enabled but failed to create runtime priors file next to base='"
            << obstacle_priors_base_path_ << "': " << err
            << ". Updates will still be applied in-memory, but will not be persisted.";
      }
    }
  }

  /// mission server
  mission_server_ = std::make_shared<ROSMissionServer>();

  /// state machine
  state_machine_ = std::make_shared<StateMachine>(
      tactic_, route_planner_, path_planner_, mission_server_);
  mission_server_->start(node_, state_machine_);

  /// robot and sensor transformation, subscription
  // clang-format off
  
  // HSHMAT: Callback group for obstacle FSM logic (obstacle_status, following_route, reroute_status)
  // MutuallyExclusive ensures these callbacks don't run concurrently, preventing race conditions
  // when speakAndWait blocks and CLEARED signals arrive.
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  
  // HSHMAT: Separate callback group for sensor callbacks (lidar, radar, gyro)
  // These must NEVER be blocked by obstacle handling / speech - critical for localization.
  // Reentrant allows multiple sensor callbacks to run concurrently if needed.
  sensor_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group_;
  
  // HSHMAT: Callback group for non-critical obstacle-related subscriptions (server_state, obstacle_grid)
  // MutuallyExclusive so only ONE runs at a time, but independent of FSM logic group.
  obstacle_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto obstacle_sub_opt = rclcpp::SubscriptionOptions();
  obstacle_sub_opt.callback_group = obstacle_callback_group_;
  // robot frame
  robot_frame_ = node_->declare_parameter<std::string>("robot_frame", "robot");
  // environment info
  const auto env_info_topic = node_->declare_parameter<std::string>("env_info_topic", "env_info");
  env_info_sub_ = node_->create_subscription<tactic::EnvInfo>(env_info_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::envInfoCallback, this, std::placeholders::_1), sub_opt);

  max_queue_size_ = node->declare_parameter<int>("queue_size", max_queue_size_);

  // HSHMAT: Initialize FSM state
  robot_paused_ = false;
  obstacle_state_ = ObstacleState::Idle;
  const auto clock_type = node_->get_clock()->get_clock_type();
  obstacle_start_time_ = rclcpp::Time(0, 0, clock_type);
  reroute_complete_time_ = rclcpp::Time(0, 0, clock_type);  // Initialize to epoch (cooldown won't trigger)
  current_W_star_ = 0.0;
  
  // HSHMAT: Load wait strategy configuration
  {
    std::string strategy_str = node_->declare_parameter<std::string>(
        "route_planning.obstacle_strategy.type", "learned");
    wait_strategy_config_.default_W_max = node_->declare_parameter<double>(
        "route_planning.obstacle_strategy.default_W_max", 120.0);
    
    // Load per-type W_max values
    const std::vector<std::string> wmax_types = {"person", "chair", "cart", "door", "unknown"};
    for (const auto& type : wmax_types) {
      try {
        double wmax = node_->declare_parameter<double>(
            "route_planning.obstacle_strategy.W_max_per_type." + type,
            wait_strategy_config_.default_W_max);
        wait_strategy_config_.W_max_per_type[type] = wmax;
      } catch (...) {
        // Use default
      }
    }
    
    wait_strategy_config_.W_grid_points = node_->declare_parameter<int>(
        "route_planning.obstacle_strategy.W_grid_points", 100);
    wait_strategy_config_.T_grid_points = node_->declare_parameter<int>(
        "route_planning.obstacle_strategy.T_grid_points", 50);
    wait_strategy_config_.learned_data_dir = node_->declare_parameter<std::string>(
        "route_planning.obstacle_strategy.learned_data_dir", "");
    
    // Expand environment variables in path
    if (!wait_strategy_config_.learned_data_dir.empty()) {
      wait_strategy_config_.learned_data_dir = 
          common::utils::expand_user(common::utils::expand_env(wait_strategy_config_.learned_data_dir));
    }
    
    // Load wait_types for rule_based strategy
    try {
      auto wait_types_vec = node_->declare_parameter<std::vector<std::string>>(
          "route_planning.obstacle_strategy.wait_types", std::vector<std::string>{"person"});
      wait_strategy_config_.wait_types = std::set<std::string>(wait_types_vec.begin(), wait_types_vec.end());
    } catch (...) {
      wait_strategy_config_.wait_types = {"person"};
    }
    
    // Load countdown intervals
    try {
      auto intervals = node_->declare_parameter<std::vector<int64_t>>(
          "route_planning.obstacle_strategy.countdown_intervals",
          std::vector<int64_t>{60, 30, 15, 10, 5});
      countdown_intervals_.clear();
      for (auto i : intervals) countdown_intervals_.push_back(static_cast<int>(i));
      std::sort(countdown_intervals_.begin(), countdown_intervals_.end(), std::greater<int>());
    } catch (...) {
      countdown_intervals_ = {60, 30, 15, 10, 5};
    }
    
    // Load debug plotting configuration
    wait_strategy_config_.debug_plot_policy = node_->declare_parameter<bool>(
        "route_planning.obstacle_strategy.debug_plot_policy", false);
    wait_strategy_config_.debug_plot_dir = node_->declare_parameter<std::string>(
        "route_planning.obstacle_strategy.debug_plot_dir", "");
    // Expand environment variables in debug_plot_dir
    if (!wait_strategy_config_.debug_plot_dir.empty()) {
      wait_strategy_config_.debug_plot_dir = 
          common::utils::expand_user(common::utils::expand_env(wait_strategy_config_.debug_plot_dir));
    }
    
    // Create the strategy
    StrategyType strategy_type;
    try {
      strategy_type = parseStrategyType(strategy_str);
    } catch (...) {
      CLOG(WARNING, "navigation") << "HSHMAT: Unknown strategy '" << strategy_str 
                                  << "', defaulting to learned";
      strategy_type = StrategyType::LEARNED;
    }
    
    // Load p_block and type_weights for learned strategy
    wait_strategy_config_.p_block = node_->declare_parameter<double>(
        "route_planning.obstacle_strategy.p_block", 0.05);
    wait_strategy_config_.robot_speed_mps = route_cfg_.nominal_speed_mps;
    
    // Type weights (probability distribution over obstacle types)
    // Default: assume "person" is most common
    wait_strategy_config_.type_weights["person"] = 0.6;
    wait_strategy_config_.type_weights["chair"] = 0.2;
    wait_strategy_config_.type_weights["cart"] = 0.1;
    wait_strategy_config_.type_weights["door"] = 0.1;
    
    // Load seed samples for survival model initialization
    // These provide initial estimates before we collect real data
    const std::vector<std::string> seed_types = {"person", "chair", "cart", "door", "unknown"};
    for (const auto& type : seed_types) {
      try {
        auto samples = node_->declare_parameter<std::vector<double>>(
            "route_planning.obstacle_strategy.seed_samples." + type,
            std::vector<double>{});
        if (!samples.empty()) {
          wait_strategy_config_.seed_samples[type] = samples;
          CLOG(DEBUG, "navigation") << "HSHMAT: Loaded " << samples.size() 
                                    << " seed samples for '" << type << "'";
        }
      } catch (...) {
        // Type not configured, skip
      }
    }
    
    wait_strategy_ = createWaitStrategy(strategy_type, wait_strategy_config_);
    CLOG(INFO, "navigation") << "HSHMAT: Initialized wait strategy: " 
                              << strategyTypeToString(strategy_type)
                              << ", W_max=" << wait_strategy_config_.default_W_max << "s"
                              << ", p_block=" << wait_strategy_config_.p_block;
    
    // For learned strategy, set up graph access (done after graph is available)
    // This will be called in setupLearnedStrategyGraphAccess()
  }

  // HSHMAT: Create publishers
  pause_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/j100_0365/platform/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS());
  auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
  pause_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/vtr/navigation_pause", latched_qos);
  speech_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/vtr/speech", rclcpp::SystemDefaultsQoS());
  
  // HSHMAT: Publisher to tell Python decision node whether to use ChatGPT.
  // Latched (transient_local) so the decision node gets the value immediately on startup.
  use_chatgpt_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/vtr/use_chatgpt", latched_qos);
  
  // HSHMAT: Publish use_chatgpt based on strategy type.
  // Only rule_based and learned need ChatGPT for obstacle type classification.
  // always_wait, always_detour, and greedy_ctp don't care about obstacle type.
  {
    std_msgs::msg::Bool chatgpt_msg;
    if (wait_strategy_) {
      StrategyType stype = wait_strategy_->type();
      chatgpt_msg.data = (stype == StrategyType::RULE_BASED || stype == StrategyType::LEARNED);
    } else {
      chatgpt_msg.data = false;  // Default to false if no strategy
    }
    use_chatgpt_pub_->publish(chatgpt_msg);
    CLOG(INFO, "navigation") << "HSHMAT: Published use_chatgpt=" << (chatgpt_msg.data ? "true" : "false")
                             << " based on strategy type.";
  }
  
  // HSHMAT: Subscribe to speech completion signal from decision node
  // NOTE: This must NOT use the MutuallyExclusive callback_group_ (sub_opt) because
  // speakAndWait blocks while waiting for speech_done_. If speech_done_sub_ is in the
  // same group, the callback can never run while speakAndWait is blocked.
  speech_done_ = true;
  speech_done_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/vtr/speech_done", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg && msg->data) {
          speech_done_ = true;
          CLOG(DEBUG, "mission.state_machine") << "HSHMAT: Speech completed signal received.";
        }
      });
  
  {
    std_msgs::msg::Bool msg;
    msg.data = false;
    pause_state_pub_->publish(msg);
  }
  
  // Hshmat: Subscribe to obstacle distance (meters along path where obstacle detected)
  last_obstacle_distance_ = 0.0;
  obstacle_distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/vtr/obstacle_distance", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!msg) return;
        last_obstacle_distance_ = msg->data;
      }, sub_opt);

  // Optional: expected duration published by decision system (for TDSP)
  obstacle_expected_duration_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/vtr/obstacle_expected_duration_sec", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!msg) return;
        last_obstacle_expected_duration_sec_ = msg->data;
      }, sub_opt);

  // HSHMAT: Subscribe to obstacle status - handles detection and clearing
  // Uses SAME callback group as following_route so they are mutually exclusive.
  // This ensures onNoAlternateRoute() completes (including speakAndWait) before any
  // obstacle callback can fire, preventing race conditions with CLEARED signals.
  obstacle_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/vtr/obstacle_status", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg) return;
        
        // SIMPLE LOGIC:
        // 1. DETECTED + Idle -> start episode (after cooldown)
        // 2. DETECTED + non-Idle -> IGNORE (already handling an episode)
        // 3. CLEARED + Waiting -> handle cleared (for always_wait)
        // 4. CLEARED + Rerouting with no_alternate_exists_ -> handle cleared (resume)
        // 5. CLEARED + Rerouting without no_alternate_exists_ -> IGNORE (let reroute finish)
        // 6. CLEARED + AwaitingClassification -> IGNORE (wait for VLM)
        
        bool should_start_episode = false;
        bool should_handle_cleared = false;
        bool should_speak_detected = false;  // For VLM strategies: say "Obstacle detected." immediately
        ObstacleState state_snapshot;
        
        {
          LockGuard lock(obstacle_mutex_);
          last_obstacle_status_msg_ = msg->data;
          state_snapshot = obstacle_state_;
          
          CLOG(DEBUG, "mission.state_machine")
              << "HSHMAT: obstacle_status=" << (msg->data ? "DETECTED" : "CLEARED")
              << ", FSM=" << obstacleStateToString(obstacle_state_);

          if (msg->data) {
            // === DETECTED ===
            if (obstacle_state_ == ObstacleState::Idle) {
              // Cooldown after episode completes so CBIT path planner can stabilize on new route.
              // CBIT often reports "no valid solution" briefly after route changes, causing the
              // path detector to publish stale/incorrect paths and trigger false DETECTED signals.
              // 1.5s is sufficient for CBIT to compute a valid local path on the new route.
              const double cooldown_sec = 1.0;
              double elapsed = (node_->get_clock()->now() - reroute_complete_time_).seconds();
              if (elapsed < cooldown_sec) {
                CLOG(DEBUG, "mission.state_machine")
                    << "HSHMAT: Ignoring DETECTED during post-reroute cooldown (" << elapsed << "s < " << cooldown_sec << "s)";
                return;
              }
              
              // Check if strategy needs VLM classification (rule_based or learned)
              StrategyType stype = wait_strategy_ ? wait_strategy_->type() : StrategyType::ALWAYS_WAIT;
              bool needs_vlm = (stype == StrategyType::RULE_BASED || stype == StrategyType::LEARNED);
              
              if (needs_vlm) {
                // Wait for VLM to classify obstacle before making decision
                obstacle_state_ = ObstacleState::AwaitingClassification;
                last_obstacle_type_ = "unknown";  // Reset to detect fresh classification
                setRobotPaused(true);
                should_speak_detected = true;  // Say "Obstacle detected." immediately
                CLOG(INFO, "mission.state_machine")
                    << "HSHMAT: DETECTED + Idle -> AwaitingClassification (waiting for VLM)";
              } else {
                // Strategies that don't need VLM (always_wait, always_detour, greedy_ctp)
                obstacle_state_ = ObstacleState::Waiting;
                should_start_episode = true;
                CLOG(INFO, "mission.state_machine")
                    << "HSHMAT: DETECTED + Idle -> starting episode";
              }
            } else {
              // Already in episode - ignore
              CLOG(DEBUG, "mission.state_machine")
                  << "HSHMAT: Ignoring DETECTED - already in episode (FSM=" 
                  << obstacleStateToString(obstacle_state_) << ")";
            }
          } else {
            // === CLEARED ===
            if (obstacle_state_ == ObstacleState::AwaitingClassification) {
              // Obstacle cleared while waiting for VLM - ignore (wait for VLM to respond)
              CLOG(DEBUG, "mission.state_machine")
                  << "HSHMAT: Ignoring CLEARED during VLM classification";
            } else if (obstacle_state_ == ObstacleState::Waiting) {
              // Obstacle cleared while waiting (always_wait) - handle it
              should_handle_cleared = true;
              CLOG(INFO, "mission.state_machine")
                  << "HSHMAT: CLEARED + Waiting -> handling";
            } else if (obstacle_state_ == ObstacleState::Rerouting) {
              // Only care about CLEARED if we're stuck with no alternate
              if (no_alternate_exists_ && !announcing_no_alternate_) {
                should_handle_cleared = true;
                CLOG(INFO, "mission.state_machine")
                    << "HSHMAT: CLEARED + Rerouting (no_alternate) -> handling";
              } else {
                CLOG(DEBUG, "mission.state_machine")
                    << "HSHMAT: Ignoring CLEARED during reroute computation";
              }
            }
          }
        }
        // Lock released
        
        if (should_speak_detected) {
          // For VLM strategies: say "Obstacle detected." immediately while VLM classifies
          speakAndWait("Obstacle detected.", 3.0);
        }
        
        if (should_start_episode) {
          startObstacleEpisode();
        } else if (should_handle_cleared) {
          handleObstacleCleared(state_snapshot);
        }
      }, sub_opt);  // Same callback group as following_route - mutually exclusive

  // HSHMAT: Track following route ids for mapping to replanner edges.
  following_route_sub_ = node_->create_subscription<vtr_navigation_msgs::msg::GraphRoute>(
      "following_route", rclcpp::QoS(10),
      [this](const vtr_navigation_msgs::msg::GraphRoute::SharedPtr route) {
        if (!route) return;
        
        bool call_reroute_complete = false;
        bool call_no_alternate = false;
        {
          LockGuard lock(obstacle_mutex_);
          
          // HSHMAT: During reroute, accept the new route from the planner.
          if (obstacle_state_ == ObstacleState::Rerouting && awaiting_new_route_ && !route->ids.empty()) {
            CLOG(INFO, "mission.state_machine")
                << "HSHMAT: Reroute SUCCESS - new path has " << route->ids.size() << " vertices";
            awaiting_new_route_ = false;
            following_route_ids_.assign(route->ids.begin(), route->ids.end());
            call_reroute_complete = true;
          }

          // Only update following_route_ids_ if we're NOT rejecting the route
          if (!call_reroute_complete && !call_no_alternate) {
            following_route_ids_.assign(route->ids.begin(), route->ids.end());
          }
        }
        // Lock released
        
        // Re-anchor learned edge stats baseline to current position on this path.
        // No lock needed: same MutuallyExclusive callback group as obstacle_status.
        if (!following_route_ids_.empty()) {
          const tactic::VertexId cur = getCurrentVertex();
          const int ri = followingRouteIndexOf(cur);
          last_path_index_ = (ri >= 0) ? ri : 0;
          CLOG(DEBUG, "navigation") << "HSHMAT: following_route stored (" << following_route_ids_.size()
                                    << " verts), last_path_index_=" << last_path_index_;
        }
        
        if (call_reroute_complete) {
          onRerouteComplete(true);
        } else if (call_no_alternate) {
          onNoAlternateRoute();
        }
      }, sub_opt);

  // HSHMAT: Subscribe to reroute status - detect when planner fails to find alternate route
  // Uses separate callback group to avoid blocking sensor callbacks
  reroute_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/vtr/reroute_status", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg) return;
        
        // Use UniqueLock so we can release before blocking speakAndWait in onNoAlternateRoute
        UniqueLock lock(obstacle_mutex_);
        
        if (msg->data == "reroute_failed" && obstacle_state_ == ObstacleState::Rerouting && awaiting_new_route_) {
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: Received reroute_failed - no alternate route exists.";
          awaiting_new_route_ = false;
          no_alternate_exists_ = true;
          // Release lock before blocking call
          lock.unlock();
          onNoAlternateRoute();
        }
      }, obstacle_sub_opt);

  // Hshmat: Occupancy grid from path obstacle detector (red cells = on-path obstacles).
  obstacle_grid_sub_ =
      node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/vtr/path_obs_detection/path_obstacle_costmap",
          rclcpp::SystemDefaultsQoS(),
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            if (!msg) return;
            last_obstacle_grid_ = *msg;
          },
          sub_opt);

  // Hshmat: Obstacle type (person/chair/...) from VLM/decision node.
  // For rule_based/learned strategies, this triggers startObstacleEpisode when VLM responds.
  obstacle_type_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/vtr/obstacle_type", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg) return;
        
        bool should_start_episode = false;
        {
          LockGuard lock(obstacle_mutex_);
          last_obstacle_type_ = msg->data;
          
          // If awaiting VLM classification, this triggers the episode
          if (obstacle_state_ == ObstacleState::AwaitingClassification) {
            CLOG(INFO, "mission.state_machine")
                << "HSHMAT: VLM classified obstacle as '" << msg->data << "' - starting episode";
            obstacle_state_ = ObstacleState::Waiting;
            should_start_episode = true;
          }
        }
        
        if (should_start_episode) {
          startObstacleEpisode();
        }
      },
      sub_opt);

  // HSHMAT: Subscribe to server state to reset obstacle FSM when Repeat starts.
  // Uses separate callback group to avoid blocking sensor callbacks
  using ServerStateMsg = vtr_navigation_msgs::msg::ServerState;
  last_goal_state_ = 0;
  server_state_sub_ = node_->create_subscription<ServerStateMsg>(
      "server_state", rclcpp::SystemDefaultsQoS(),
      [this](const ServerStateMsg::SharedPtr msg) {
        if (!msg) return;
        
        // Detect transition to STARTING state (new goal beginning)
        const bool goal_starting = (msg->current_goal_state == ServerStateMsg::STARTING) &&
                                   (last_goal_state_ != ServerStateMsg::STARTING);
        last_goal_state_ = msg->current_goal_state;
        
        if (goal_starting) {
          // Check if any goal is a Repeat goal
          bool is_repeat = false;
          for (const auto& goal : msg->goals) {
            if (goal.type == 2) {  // REPEAT = 2
              is_repeat = true;
              break;
            }
          }
          
          if (is_repeat) {
            LockGuard lock(obstacle_mutex_);
            CLOG(INFO, "mission.state_machine")
                << "HSHMAT: Repeat goal starting - resetting obstacle state.";
            resetObstacleState();
          }
        }
      },
      obstacle_sub_opt);

#ifdef VTR_ENABLE_LIDAR
if (pipeline->name() == "lidar"){
  lidar_frame_ = node_->declare_parameter<std::string>("lidar_frame", "lidar");
  gyro_frame_ = node_->declare_parameter<std::string>("gyro_frame", "gyro");
  gyro_bias_ = {
    node_->declare_parameter<double>("gyro_bias.x", 0.0),
    node_->declare_parameter<double>("gyro_bias.y", 0.0),
    node_->declare_parameter<double>("gyro_bias.z", 0.0)};
  T_lidar_robot_ = loadTransform(lidar_frame_, robot_frame_);
  T_gyro_robot_ = loadTransform(gyro_frame_, robot_frame_);
  // static transform
  tf_sbc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  // Publish a static TF for the configured robot/lidar frames (best-effort).
  // This is helpful when upstream sensors publish in a disconnected tree (e.g., os_sensor/os_lidar).
  //
  // Note: T_lidar_robot_ is obtained from TF lookup (lidar_frame_ <- robot_frame_). If TF is missing,
  // loadTransform falls back to identity, which still provides a consistent tree for internal use.
  {
    auto msg = tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot_.matrix()));
    msg.header.frame_id = robot_frame_;
    msg.child_frame_id = lidar_frame_;
  tf_sbc_->sendTransform(msg);
  }
  // Also publish a stable "lidar" alias if the configured lidar_frame_ is different.
  // This preserves legacy consumers (including route_screening fallbacks) that
  // assume a frame literally named "lidar" exists in the tree.
  if (lidar_frame_ != "lidar") {
    auto msg_alias = tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot_.matrix()));
    msg_alias.header.frame_id = robot_frame_;
    msg_alias.child_frame_id = "lidar";
    tf_sbc_->sendTransform(msg_alias);
  }
  // If robot_frame_ is not literally "robot", also publish an identity alias
  // robot -> robot_frame_ to connect common downstream consumers that assume "robot".
  if (robot_frame_ != "robot") {
    geometry_msgs::msg::TransformStamped alias;
    alias.header.stamp = node_->get_clock()->now();
    alias.header.frame_id = "robot";
    alias.child_frame_id = robot_frame_;
    alias.transform.rotation.w = 1.0;
    tf_sbc_->sendTransform(alias);
  }
  // lidar pointcloud data subscription
  const auto lidar_topic = node_->declare_parameter<std::string>("lidar_topic", "/points");


  auto lidar_qos = rclcpp::QoS(max_queue_size_);
  lidar_qos.reliable();
  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, lidar_qos, std::bind(&Navigator::lidarCallback, this, std::placeholders::_1), sensor_sub_opt);
}
#endif
#ifdef VTR_ENABLE_VISION
if (pipeline->name() == "stereo") {
  using namespace std::placeholders;

  camera_frame_ = node_->declare_parameter<std::string>("camera_frame", "camera");
  T_camera_robot_ = loadTransform(camera_frame_, robot_frame_);
  // static transform
  tf_sbc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  auto msg = tf2::eigenToTransform(Eigen::Affine3d(T_camera_robot_.inverse().matrix()));
  msg.header.frame_id = robot_frame_;
  msg.child_frame_id = camera_frame_;
  tf_sbc_->sendTransform(msg);
  // camera images subscription
  const auto right_image_topic = node_->declare_parameter<std::string>("camera_right_topic", "/image_right");
  const auto left_image_topic = node_->declare_parameter<std::string>("camera_left_topic", "/image_left");

  auto camera_qos = rclcpp::QoS(10);
  camera_qos.reliable();

  right_camera_sub_.subscribe(node_, right_image_topic, camera_qos.get_rmw_qos_profile());
  left_camera_sub_.subscribe(node_, left_image_topic, camera_qos.get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<ApproximateImageSync>>(ApproximateImageSync(10), right_camera_sub_, left_camera_sub_);
  sync_->registerCallback(&Navigator::cameraCallback, this);
}
#endif
  // clang-format on

#ifdef VTR_ENABLE_RADAR
if (pipeline->name() == "radar") {

  radar_frame_ = node_->declare_parameter<std::string>("radar_frame", "radar");
  gyro_frame_ = node_->declare_parameter<std::string>("gyro_frame", "gyro");
  gyro_bias_ = {
      node_->declare_parameter<double>("gyro_bias.x", 0.0),
      node_->declare_parameter<double>("gyro_bias.y", 0.0),
      node_->declare_parameter<double>("gyro_bias.z", 0.0)};
  // there are a radar and gyro frames
  T_radar_robot_ = loadTransform(radar_frame_, robot_frame_);
  T_gyro_robot_ = loadTransform(gyro_frame_, robot_frame_);
  // static transform make a shared pointer to the static transform broadcaster
  tf_sbc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  auto msg_radar = tf2::eigenToTransform(Eigen::Affine3d(T_radar_robot_.inverse().matrix()));
  msg_radar.header.frame_id = "robot";
  msg_radar.child_frame_id = "radar";
  auto msg_gyro = tf2::eigenToTransform(Eigen::Affine3d(T_gyro_robot_.inverse().matrix()));
  msg_gyro.header.frame_id = "robot";
  msg_gyro.child_frame_id = "gyro";
  std::vector<geometry_msgs::msg::TransformStamped> tfs = {msg_radar,msg_gyro};
  tf_sbc_->sendTransform(tfs);
  // radar pointcloud data subscription this is the default value
  const auto radar_topic = node_->declare_parameter<std::string>("radar_topic", "/radar_data/b_scan_msg");
  // not sure if the  radar data rate is low as well
  auto radar_qos = rclcpp::QoS(max_queue_size_);
  radar_qos.reliable();
  radar_sub_ = node_->create_subscription<navtech_msgs::msg::RadarBScanMsg>(radar_topic, radar_qos, std::bind(&Navigator::radarCallback, this, std::placeholders::_1), sensor_sub_opt);
}
#endif

  // Subscribe to the imu topic 
  auto gyro_qos = rclcpp::QoS(100);
  gyro_qos.reliable();
  const auto gyro_topic = node_->declare_parameter<std::string>("gyro_topic", "/ouster/imu");
  gyro_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(gyro_topic, gyro_qos, std::bind(&Navigator::gyroCallback, this, std::placeholders::_1), sensor_sub_opt);


  /// This creates a thread to process the sensor input
  thread_count_ = 1;
  process_thread_ = std::thread(&Navigator::process, this);
  CLOG(INFO, "navigation") << "VT&R3 initialization done!";
}

void Navigator::setRobotPaused(bool paused) {
  if (robot_paused_ == paused) return;
  robot_paused_ = paused;
  if (paused && pause_cmd_pub_) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.linear.z = 0.0;
    stop_cmd.angular.x = 0.0;
    stop_cmd.angular.y = 0.0;
    stop_cmd.angular.z = 0.0;
    pause_cmd_pub_->publish(stop_cmd);
  }
  if (pause_state_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = paused;
    pause_state_pub_->publish(msg);
  }
}

void Navigator::speak(const std::string& text) {
  if (speech_pub_ && !text.empty()) {
    speech_done_ = false;
    std_msgs::msg::String msg;
    msg.data = text;
    speech_pub_->publish(msg);
    CLOG(INFO, "mission.state_machine") << "HSHMAT Speech: " << text;
  }
}

void Navigator::speakAndWait(const std::string& text, double timeout_sec) {
  // HSHMAT: Speak and block until speech is done (or timeout).
  // This ensures robot doesn't move while speaking.
  if (text.empty()) return;
  
  speak(text);
  
  // Wait for speech_done_ signal with timeout
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::duration<double>(timeout_sec);
  
  while (!speech_done_) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= timeout) {
      CLOG(WARNING, "mission.state_machine")
          << "HSHMAT: speakAndWait timeout after " << timeout_sec << "s for: " << text;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
  CLOG(DEBUG, "mission.state_machine") << "HSHMAT: speakAndWait completed for: " << text;
}

void Navigator::resetObstacleState() {
  // HSHMAT: Reset obstacle FSM to clean slate (called on Repeat start)
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Resetting obstacle state. Previous FSM=" << obstacleStateToString(obstacle_state_);
  
  // Save learned strategy data before resetting (preserves statistics from previous run)
  if (wait_strategy_ && wait_strategy_->type() == StrategyType::LEARNED) {
    auto* learned = dynamic_cast<LearnedStrategy*>(wait_strategy_.get());
    if (learned) {
      learned->saveData();
      CLOG(INFO, "navigation") << "HSHMAT: Saved learned strategy data before reset";
    }
  }
  
  // Stop any running timers
  if (wait_timer_) {
    wait_timer_->cancel();
    wait_timer_.reset();
  }
  
  obstacle_state_ = ObstacleState::Idle;
  last_obstacle_status_msg_ = false;
  current_W_star_ = 0.0;
  last_obstacle_type_ = "unknown";
  wait_episode_start_sec_ = -1.0;
  wait_episode_type_ = "unknown";
  
  // Reset learned edge stats baseline for new repeat
  last_path_index_ = 0;
  
  // Ensure robot is not paused
  if (robot_paused_) {
    setRobotPaused(false);
  } else if (pause_state_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = false;
    pause_state_pub_->publish(msg);
  }
  
  // Clear greedy CTP banned edges if applicable
  if (wait_strategy_) {
    wait_strategy_->clearPermanentBans();
    wait_strategy_->resetMemory();
  }
  
  // Clear current blocked edges
  current_blocked_edges_.clear();
  
  // Clear reroute tracking
  reroute_snapshot_route_.clear();
  awaiting_new_route_ = false;
  no_alternate_exists_ = false;
  announcing_no_alternate_ = false;
  reroute_complete_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  
  CLOG(INFO, "mission.state_machine") << "HSHMAT: Obstacle state reset complete.";
}

void Navigator::setupLearnedStrategyGraphAccess() {
  // HSHMAT: Set up graph access functions for Learned strategy's TDSP
  if (!wait_strategy_ || !graph_) return;
  
  auto* learned = dynamic_cast<LearnedStrategy*>(wait_strategy_.get());
  if (!learned) return;
  
  // Create privileged subgraph ONCE and capture by shared_ptr for all queries
  // This avoids expensive getSubgraph() calls on every neighbor/edge lookup
  using PrivEval = tactic::PrivilegedEvaluator<tactic::GraphBase>;
  auto priv_eval = std::make_shared<PrivEval>(*graph_);
  auto priv_graph = std::make_shared<decltype(graph_->getSubgraph(priv_eval))>(graph_->getSubgraph(priv_eval));
  
  // Capture robot speed by value
  double robot_speed = wait_strategy_config_.robot_speed_mps;
  
  // Create neighbors function (captures priv_graph by shared_ptr)
  auto get_neighbors = [priv_graph](const tactic::VertexId& v) -> std::vector<tactic::VertexId> {
    std::vector<tactic::VertexId> neighbors;
    if (!priv_graph || !(*priv_graph)) {
      return neighbors;
    }
    
    try {
      for (const auto& n : (*priv_graph)->neighbors(v)) {
        neighbors.push_back(n);
      }
    } catch (...) {}
    
    return neighbors;
  };
  
  // Create travel time function (captures priv_graph by shared_ptr)
  auto get_travel_time = [priv_graph, robot_speed](const tactic::EdgeId& e) -> double {
    if (!priv_graph || !(*priv_graph)) {
      return std::numeric_limits<double>::infinity();
    }
    
    try {
      auto edge_ptr = (*priv_graph)->at(e);
      if (edge_ptr) {
        double len_m = edge_ptr->T().r_ab_inb().norm();
        if (robot_speed > 1e-6) {
          return len_m / robot_speed;
        }
      }
    } catch (...) {}
    
    return std::numeric_limits<double>::infinity();
  };
  
  learned->setGraphAccess(get_neighbors, get_travel_time);
  CLOG(INFO, "navigation") << "HSHMAT: Set up graph access for LearnedStrategy (cached subgraph).";
}

EdgeIdSet Navigator::computeBlockedEdges() const {
  // HSHMAT: Compute which edges are currently blocked by the obstacle
  // Based on obstacle distance and extent along the route
  EdgeIdSet blocked;
  
  if (following_route_ids_.empty() || !graph_) {
    return blocked;
  }
  
  try {
    // Get current localization
    auto loc = tactic_->getPersistentLoc();
    if (!loc.v.isValid()) return blocked;
    
    const auto current_vid = loc.v;
    int idx = -1;
    
    // Find current position in route
    for (size_t i = 0; i < following_route_ids_.size(); ++i) {
      if (tactic::VertexId(following_route_ids_[i]) == current_vid) {
        idx = static_cast<int>(i);
        break;
      }
    }
    
    if (idx < 0) return blocked;
    
    // Estimate obstacle extent
    const double obstacle_extent_m = estimateObstacleExtentFromGrid();
    const double obstacle_start = last_obstacle_distance_;
    const double obstacle_end = obstacle_start + std::max(0.5, obstacle_extent_m);
    
    // Walk through route edges and find which are blocked
    double cumulative_dist = 0.0;
    using PrivEval = tactic::PrivilegedEvaluator<tactic::GraphBase>;
    auto priv_eval = std::make_shared<PrivEval>(*graph_);
    auto priv_graph = graph_->getSubgraph(priv_eval);
    
    for (int i = idx; i + 1 < static_cast<int>(following_route_ids_.size()); ++i) {
      tactic::VertexId v1(following_route_ids_[i]);
      tactic::VertexId v2(following_route_ids_[i + 1]);
      
      double edge_length = 0.0;
      try {
        auto edge_ptr = priv_graph->at(tactic::EdgeId(v1, v2));
        if (edge_ptr) {
          edge_length = edge_ptr->T().r_ab_inb().norm();
        }
      } catch (...) {}
      
      double edge_start_dist = cumulative_dist;
      double edge_end_dist = cumulative_dist + edge_length;
      
      // Check if this edge overlaps with obstacle
      if (edge_end_dist > obstacle_start && edge_start_dist < obstacle_end) {
        blocked.insert(tactic::EdgeId(v1, v2));
      }
      
      cumulative_dist = edge_end_dist;
      
      // Stop if we've passed the obstacle
      if (cumulative_dist > obstacle_end + 10.0) break;
    }
  } catch (const std::exception& e) {
    CLOG(WARNING, "navigation") << "HSHMAT: Error computing blocked edges: " << e.what();
  }
  
  return blocked;
}

tactic::VertexId Navigator::getCurrentVertex() const {
  if (!tactic_) return tactic::VertexId::Invalid();
  auto loc = tactic_->getPersistentLoc();
  return loc.v.isValid() ? loc.v : tactic::VertexId::Invalid();
}

tactic::VertexId Navigator::getGoalVertex() const {
  if (following_route_ids_.empty()) return tactic::VertexId::Invalid();
  return tactic::VertexId(following_route_ids_.back());
}

int Navigator::followingRouteIndexOf(const tactic::VertexId& v) const {
  if (!v.isValid() || following_route_ids_.empty()) return -1;
  for (size_t i = 0; i < following_route_ids_.size(); ++i) {
    if (tactic::VertexId(following_route_ids_[i]) == v) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool Navigator::routesSameRemaining(
    const std::vector<uint64_t>& snapshot,
    const std::vector<uint64_t>& new_route,
    const tactic::VertexId& current_vid) const {
  // Compare remaining path from current vertex to goal in both routes.
  // E.g. snapshot [1,2,3,4], new_route [3,4], current at 3 -> both have [3,4] remaining -> same.
  if (snapshot.empty() || new_route.empty() || !current_vid.isValid())
    return snapshot.empty() && new_route.empty();

  // Find current vertex in snapshot
  bool found_snap = false;
  size_t idx_snap = 0;
  for (size_t i = 0; i < snapshot.size(); ++i) {
    if (tactic::VertexId(snapshot[i]) == current_vid) {
      idx_snap = i;
      found_snap = true;
      break;
    }
  }
  if (!found_snap) return false;

  // Find current vertex in new_route
  bool found_new = false;
  size_t idx_new = 0;
  for (size_t i = 0; i < new_route.size(); ++i) {
    if (tactic::VertexId(new_route[i]) == current_vid) {
      idx_new = i;
      found_new = true;
      break;
    }
  }
  if (!found_new) return false;

  // Compare remaining segments
  const size_t len_snap = snapshot.size() - idx_snap;
  const size_t len_new = new_route.size() - idx_new;
  if (len_snap != len_new) return false;
  for (size_t k = 0; k < len_snap; ++k) {
    if (snapshot[idx_snap + k] != new_route[idx_new + k]) return false;
  }
  return true;
}

void Navigator::startObstacleEpisode() {
  // HSHMAT: Called when obstacle detected
  // NOTE: obstacle_state_ is already set to Waiting by caller as a guard against re-entry
  
  try {
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: ============================================================";
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: OBSTACLE DETECTED! Type='" << last_obstacle_type_ << "'";
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: ============================================================";
    
    obstacle_start_time_ = node_->get_clock()->now();
    wait_episode_type_ = last_obstacle_type_;
    wait_episode_start_sec_ = obstacle_start_time_.seconds();
    
    // Pause robot immediately
    setRobotPaused(true);
    
    // Compute which edges are blocked by this obstacle
    current_blocked_edges_ = computeBlockedEdges();
    {
      std::stringstream ss;
      ss << "HSHMAT: Blocked edges (" << current_blocked_edges_.size() << "): ";
      for (const auto& e : current_blocked_edges_) ss << e << " ";
      ss << " [current_v=" << getCurrentVertex() << "]";
      CLOG(INFO, "mission.state_machine") << ss.str();
    }
    
    // Get current and goal vertices for TDSP
    tactic::VertexId current_v = getCurrentVertex();
    tactic::VertexId goal_v = getGoalVertex();
    double t_now = node_->get_clock()->now().seconds();

    // Learned p_block: edges along stored path since last obstacle (or since route anchor).
    // Example: first obstacle at index 23 -> +23, last_path_index_=23; next at 40 -> +17.
    // No lock needed: same MutuallyExclusive callback group handles following_route + obstacle_status.
    if (wait_strategy_ && wait_strategy_->type() == StrategyType::LEARNED) {
      const int idx_in_route = followingRouteIndexOf(current_v);
      if (idx_in_route >= 0) {
        const int baseline = last_path_index_;
        int edge_delta = idx_in_route - baseline;
        if (edge_delta < 0) edge_delta = 0;
        last_path_index_ = idx_in_route;
        
        if (edge_delta > 0) {
          auto* learned = dynamic_cast<LearnedStrategy*>(wait_strategy_.get());
          if (learned && learned->obstacleStats()) {
            learned->obstacleStats()->recordEdgeTraversals(edge_delta);
            CLOG(INFO, "navigation") << "HSHMAT: Learned obstacle_stats: +" << edge_delta
                                      << " edges (path index " << baseline << " -> " << idx_in_route << ")";
          }
          
          // HSHMAT: Clear memory for successfully traversed edges.
          // If robot traversed an edge that was previously marked as blocked in memory,
          // we now know it's free. This ensures memory stays consistent with reality.
          // Clear memory for edges from baseline to idx_in_route.
          if (learned && baseline >= 0 && idx_in_route > baseline) {
            for (int i = baseline; i < idx_in_route && i + 1 < static_cast<int>(following_route_ids_.size()); ++i) {
              tactic::VertexId v1(following_route_ids_[i]);
              tactic::VertexId v2(following_route_ids_[i + 1]);
              tactic::EdgeId edge(v1, v2);
              learned->clearMemoryForEdge(edge);
              // Also clear reverse edge
              tactic::EdgeId edge_rev(v2, v1);
              learned->clearMemoryForEdge(edge_rev);
            }
            CLOG(DEBUG, "navigation") << "HSHMAT: Cleared memory for " << edge_delta 
                                      << " successfully traversed edges";
          }
        }
      } else if (!following_route_ids_.empty()) {
        CLOG(WARNING, "navigation")
            << "HSHMAT: Current vertex " << current_v
            << " not found in following_route (" << following_route_ids_.size()
            << " verts); edge count for this episode skipped.";
      }
    }
    
    // Ensure graph access is set up for learned strategy
    setupLearnedStrategyGraphAccess();
    
    // Compute W* using the configured strategy (new interface)
    CLOG(INFO, "mission.state_machine") << "HSHMAT: Computing wait decision...";
    WaitDecision decision = wait_strategy_->computeWaitTime(
        last_obstacle_type_,
        current_blocked_edges_,
        current_v,
        goal_v,
        t_now,
        0.0);  // obstacle_t_first = 0 for new obstacle
    current_W_star_ = decision.W_star;
    
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Strategy=" << strategyTypeToString(wait_strategy_->type())
        << ", W*=" << current_W_star_ << "s, should_wait=" << decision.should_wait
        << ", speech='" << decision.speech << "'";
    
    if (!decision.should_wait || current_W_star_ <= 0.0) {
      // Immediate reroute - speak and WAIT before triggering reroute
      // This ensures speech completes before robot might start moving
      CLOG(INFO, "mission.state_machine") << "HSHMAT: Starting speakAndWait for reroute...";
      speakAndWait(decision.speech, 5.0);
      CLOG(INFO, "mission.state_machine") << "HSHMAT: speakAndWait completed, triggering reroute...";
      {
        LockGuard lock(obstacle_mutex_);
        obstacle_state_ = ObstacleState::Rerouting;
      }
      triggerReroute();
    } else {
      // Waiting - must use speakAndWait so we don't process CLEARED until
      // "person. Waiting." (etc.) finishes. Otherwise a speech_done from the
      // wait announcement can be mistaken for "Obstacle cleared." and the
      // robot would move before saying "Obstacle cleared."
      speakAndWait(decision.speech, 5.0);
      
      {
        LockGuard lock(obstacle_mutex_);
        obstacle_state_ = ObstacleState::Waiting;  // Confirm final state
        
        // Set up countdown announcements
        next_countdown_idx_ = 0;
        for (size_t i = 0; i < countdown_intervals_.size(); ++i) {
          if (countdown_intervals_[i] < current_W_star_) {
            next_countdown_idx_ = static_cast<int>(i);
            break;
          }
        }
      }
      
      // Start 1-second timer for countdown (uses obstacle callback group)
      wait_timer_ = node_->create_wall_timer(
          std::chrono::seconds(1),
          [this]() { onWaitTimerTick(); },
          obstacle_callback_group_);
      
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: FSM: Idle -> Waiting. Timer started for " << current_W_star_ << "s.";
    }
  } catch (const std::exception& e) {
    CLOG(ERROR, "mission.state_machine")
        << "HSHMAT: Exception in startObstacleEpisode: " << e.what();
    // Reset to Idle on error
    {
      LockGuard lock(obstacle_mutex_);
      obstacle_state_ = ObstacleState::Idle;
    }
    setRobotPaused(false);
  } catch (...) {
    CLOG(ERROR, "mission.state_machine")
        << "HSHMAT: Unknown exception in startObstacleEpisode";
    // Reset to Idle on error
    {
      LockGuard lock(obstacle_mutex_);
      obstacle_state_ = ObstacleState::Idle;
    }
    setRobotPaused(false);
  }
}

void Navigator::onWaitTimerTick() {
  bool should_timeout = false;
  bool should_announce = false;
  int announce_seconds = 0;
  
  {
    LockGuard lock(obstacle_mutex_);
    
    if (obstacle_state_ != ObstacleState::Waiting) {
      if (wait_timer_) {
        wait_timer_->cancel();
        wait_timer_.reset();
      }
      return;
    }
    
    double elapsed = (node_->get_clock()->now() - obstacle_start_time_).seconds();
    double remaining = current_W_star_ - elapsed;
    
    CLOG(DEBUG, "mission.state_machine")
        << "HSHMAT: Wait tick - elapsed=" << elapsed << "s, remaining=" << remaining << "s";
    
    // Check for countdown announcements
    if (next_countdown_idx_ < static_cast<int>(countdown_intervals_.size())) {
      int next_announce = countdown_intervals_[next_countdown_idx_];
      if (remaining <= next_announce && remaining > next_announce - 1) {
        should_announce = true;
        announce_seconds = next_announce;
        next_countdown_idx_++;
      }
    }
    
    // Check for timeout
    if (remaining <= 0.0) {
      should_timeout = true;
    }
  }
  // Lock released
  
  if (should_announce) {
    speak(std::to_string(announce_seconds) + " seconds left.");
  }
  
  if (should_timeout) {
    onWaitTimeout();
  }
}

void Navigator::onObstacleCleared() {
  // HSHMAT: Called when obstacle clears during Waiting state
  // NOTE: State is still Waiting - we keep it that way during speech to block new episodes
  double elapsed;
  std::string episode_type;
  {
    LockGuard lock(obstacle_mutex_);
    elapsed = (node_->get_clock()->now() - obstacle_start_time_).seconds();
    episode_type = wait_episode_type_;
  }
  
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Obstacle cleared after " << elapsed << "s (W*=" << current_W_star_ << "s)";
  
  // Stop timer
  if (wait_timer_) {
    wait_timer_->cancel();
    wait_timer_.reset();
  }
  
  // Record uncensored sample for learned strategy
  wait_strategy_->onObstacleCleared(episode_type, elapsed);
  
  // Also update old prior system if enabled
  updateObstaclePriorFromWaitEpisode(episode_type, elapsed);
  
  // HSHMAT: Speak and WAIT for speech to complete BEFORE resuming robot
  // This ensures the robot doesn't start moving while saying "Obstacle cleared"
  // State remains Waiting/Rerouting during speech to block new obstacle detections
  speakAndWait("Obstacle cleared.", 5.0);
  // NOW complete episode (sets state to Idle)
  completeEpisode();
}

void Navigator::onRerouteComplete(bool alternate_found) {
  // Called when reroute succeeds (alternate route found)
  {
    LockGuard lock(obstacle_mutex_);
    if (obstacle_state_ != ObstacleState::Rerouting || !alternate_found) return;
    no_alternate_exists_ = false;
  }
  
  CLOG(INFO, "mission.state_machine") << "HSHMAT: Reroute completed successfully.";
  completeEpisode();
}

void Navigator::onNoAlternateRoute() {
  // Called when planner determines no alternate route exists
  StrategyType strategy_type;
  {
    LockGuard lock(obstacle_mutex_);
    if (obstacle_state_ != ObstacleState::Rerouting) return;
    strategy_type = wait_strategy_ ? wait_strategy_->type() : StrategyType::ALWAYS_WAIT;
    no_alternate_exists_ = true;  // Mark for handleObstacleCleared
    announcing_no_alternate_ = true;  // Block handleObstacleCleared during announcement
  }
  
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: onNoAlternateRoute - strategy=" << strategyTypeToString(strategy_type);
  
  switch (strategy_type) {
    case StrategyType::ALWAYS_DETOUR: {
      // Announce no alternate, wait for obstacle to clear
      speakAndWait("No alternate path to goal. Waiting.", 5.0);
      
      // Speech done - NOW we start caring about obstacle status
      // Any changes during speech are ignored
      {
        LockGuard lock(obstacle_mutex_);
        announcing_no_alternate_ = false;
      }
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: 'No alternate' announcement complete - now waiting for obstacle to clear.";
      // Stay in Rerouting state - obstacle_status callback will handle resuming
      break;
    }
      
    case StrategyType::GREEDY_CTP: {
      // Mark no valid path, stay stuck forever (correct CTP behavior)
      auto* greedy = dynamic_cast<GreedyCTPStrategy*>(wait_strategy_.get());
      if (greedy) greedy->onNoValidPath();
      speakAndWait("No valid path to goal.", 5.0);
      {
        LockGuard lock(obstacle_mutex_);
        announcing_no_alternate_ = false;
      }
      // Stay in Rerouting state - never resume
      break;
    }
    
    case StrategyType::RULE_BASED:
    case StrategyType::LEARNED: {
      // HSHMAT: Same as ALWAYS_DETOUR - wait for obstacle to clear
      // These strategies attempted reroute but failed; wait for original path to clear
      speakAndWait("No alternate path to goal. Waiting.", 5.0);
      {
        LockGuard lock(obstacle_mutex_);
        announcing_no_alternate_ = false;
      }
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: 'No alternate' announcement complete - now waiting for obstacle to clear.";
      // Stay in Rerouting state - obstacle_status callback will handle resuming
      break;
    }
    
    default:
      // Unknown strategies: resume on original path (fallback)
      {
        LockGuard lock(obstacle_mutex_);
        no_alternate_exists_ = false;
        announcing_no_alternate_ = false;
      }
      speakAndWait("No alternate path. Resuming.", 5.0);
      completeEpisode();
      break;
  }
}

void Navigator::handleObstacleCleared(ObstacleState previous_state) {
  // Called when obstacle clears during an episode
  // Only called from obstacle_status callback in two cases:
  // 1. previous_state == Waiting (always_wait path cleared)
  // 2. previous_state == Rerouting AND no_alternate_exists_ (waiting for clearance)
  // NOTE: Called WITHOUT lock held
  
  StrategyType strategy_type;
  {
    LockGuard lock(obstacle_mutex_);
    strategy_type = wait_strategy_ ? wait_strategy_->type() : StrategyType::ALWAYS_WAIT;
  }
  
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: handleObstacleCleared - previous_state=" << obstacleStateToString(previous_state)
      << ", strategy=" << strategyTypeToString(strategy_type);
  
  if (previous_state == ObstacleState::Waiting) {
    // Obstacle cleared while waiting (always_wait strategy)
    onObstacleCleared();
    return;
  }
  
  if (previous_state == ObstacleState::Rerouting) {
    // Obstacle cleared while waiting for no-alternate
    // (Callback already verified no_alternate_exists_ && !announcing_no_alternate_)
    
    if (strategy_type == StrategyType::GREEDY_CTP) {
      // greedy_ctp stays stuck forever - ignore clearing
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: greedy_ctp stays stuck - ignoring obstacle cleared.";
      return;
    }
    
    // For always_detour (and others): resume now
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Obstacle cleared (no alternate existed) - resuming.";
    {
      LockGuard lock(obstacle_mutex_);
      no_alternate_exists_ = false;
      awaiting_new_route_ = false;
    }
    speakAndWait("Obstacle cleared.", 5.0);
    completeEpisode();
    return;
  }
  
  // Shouldn't reach here
  CLOG(WARNING, "mission.state_machine")
      << "HSHMAT: handleObstacleCleared called with unexpected state: " 
      << obstacleStateToString(previous_state);
}

void Navigator::onWaitTimeout() {
  // HSHMAT: Called when W* expires without obstacle clearing
  double elapsed;
  std::string episode_type;
  {
    LockGuard lock(obstacle_mutex_);
    if (obstacle_state_ != ObstacleState::Waiting) return;
    
    elapsed = (node_->get_clock()->now() - obstacle_start_time_).seconds();
    episode_type = wait_episode_type_;
    
    // Stop timer
    if (wait_timer_) {
      wait_timer_->cancel();
      wait_timer_.reset();
    }
    
    obstacle_state_ = ObstacleState::Rerouting;
  }
  // Lock released
  
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Wait timeout after " << elapsed << "s (W*=" << current_W_star_ << "s). Rerouting.";
  
  // Record censored sample for learned strategy
  wait_strategy_->onRerouteTimeout(episode_type, elapsed);
  
  // HSHMAT: Update memory timestamp after censored wait.
  // This updates t_last_confirmed so that future expected wait calculations
  // use the correct conditional survival probability P(R > t | R > c).
  // Without this, memory would be stale (c too old) and EW would be wrong.
  double t_after_wait = node_->get_clock()->now().seconds();
  wait_strategy_->updateMemoryAfterCensoredWait(current_blocked_edges_, t_after_wait);
  CLOG(DEBUG, "navigation") << "HSHMAT: Updated memory for " << current_blocked_edges_.size()
                            << " blocked edges after censored wait (t=" << t_after_wait << ")";
  
  // Use speakAndWait to ensure speech completes before robot starts moving
  speakAndWait("Time limit exceeded. Rerouting.", 5.0);
  
  triggerReroute();
}

void Navigator::completeEpisode() {
  // HSHMAT: Complete the current obstacle episode, return to Idle
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Episode complete. FSM: " << obstacleStateToString(obstacle_state_) << " -> Idle";
  
  // Clean up planner state (unless greedy CTP which keeps edges banned)
  if (wait_strategy_->type() != StrategyType::GREEDY_CTP) {
    try {
      if (auto bfs_ptr = std::dynamic_pointer_cast<vtr::route_planning::BFSPlanner>(route_planner_)) {
        bfs_ptr->clearExtraEdgeCosts();
        bfs_ptr->setUseTimeCost(false);
      } else if (auto tdsp = std::dynamic_pointer_cast<vtr::route_planning::TDSPPlanner>(route_planner_)) {
        tdsp->setStaticEdgeDelays({});
      }
    } catch (...) {}
  }
  
  {
    LockGuard lock(obstacle_mutex_);
    obstacle_state_ = ObstacleState::Idle;
    current_W_star_ = 0.0;
    wait_episode_start_sec_ = -1.0;
    no_alternate_exists_ = false;
    awaiting_new_route_ = false;
    announcing_no_alternate_ = false;
    // 500ms cooldown: ignore DETECTED signals while path detector updates to new route.
    // Without this, resuming after "obstacle cleared" immediately re-triggers an episode.
    reroute_complete_time_ = node_->get_clock()->now();
  }
  
  setRobotPaused(false);
}

void Navigator::loadObstaclePriorsFromFile() {
  obstacle_priors_sec_.clear();
  obstacle_default_prior_sec_ = 60.0;

  std::string path = route_cfg_.obstacle_cost_file;
  path = common::utils::expand_user(common::utils::expand_env(path));
  obstacle_priors_base_path_ = path;

  try {
    YAML::Node root = YAML::LoadFile(path);

    if (root["default"]) {
      obstacle_default_prior_sec_ = root["default"].as<double>();
    }

    YAML::Node map = root["obstacles"] ? root["obstacles"] : root;
    if (map && map.IsMap()) {
      for (auto it = map.begin(); it != map.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        // Skip the "default" key if the file is a flat map.
        if (key == "default") continue;
        obstacle_priors_sec_[key] = it->second.as<double>();
      }
    }

    CLOG(INFO, "navigation")
        << "Loaded obstacle priors from '" << path << "': "
        << obstacle_priors_sec_.size() << " entries, default="
        << obstacle_default_prior_sec_ << "s";
  } catch (const std::exception &e) {
    CLOG(WARNING, "navigation")
        << "Failed to load obstacle priors from '" << path
        << "': " << e.what() << ". Using defaults only.";
  }
}

double Navigator::expectedObstacleDurationSeconds() const {
  // VLM source: take the latest duration published by the decision system.
  if (route_cfg_.obstacle_cost_source == "vlm") {
    return last_obstacle_expected_duration_sec_;
  }

  // File source: look up by obstacle type, else default.
  const std::string type = last_obstacle_type_;
  auto it = obstacle_priors_sec_.find(type);
  if (it != obstacle_priors_sec_.end()) return it->second;

  // Common aliases.
  if (type == "human") {
    auto it2 = obstacle_priors_sec_.find("person");
    if (it2 != obstacle_priors_sec_.end()) return it2->second;
  }

  return obstacle_default_prior_sec_;
}

void Navigator::updateObstaclePriorFromWaitEpisode(const std::string &type_in,
                                                  const double actual_wait_sec_in) {
  // Only update priors when using file-based durations.
  if (route_cfg_.obstacle_cost_source != "file") return;
  if (!route_cfg_.update_obstacle_priors) return;

  // Normalize key.
  std::string type = type_in;
  std::transform(type.begin(), type.end(), type.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (type.empty()) return;
  if (type == "human") type = "person";

  const double actual_wait_sec = std::max(0.0, actual_wait_sec_in);
  const double alpha = std::min(1.0, std::max(0.0, route_cfg_.obstacle_prior_update_alpha));

  const double min_sec = route_cfg_.obstacle_prior_update_min_sec;
  const double max_sec = route_cfg_.obstacle_prior_update_max_sec;

  const double old_prior =
      (obstacle_priors_sec_.count(type) ? obstacle_priors_sec_.at(type) : obstacle_default_prior_sec_);
  double new_prior = (1.0 - alpha) * old_prior + alpha * actual_wait_sec;
  new_prior = std::min(max_sec, std::max(min_sec, new_prior));

  obstacle_priors_sec_[type] = new_prior;

  bool persisted = false;
  std::string err;
  if (!obstacle_priors_runtime_path_.empty()) {
    persisted = writeObstaclePriorsYamlAtomic(obstacle_priors_runtime_path_,
                                             obstacle_default_prior_sec_,
                                             obstacle_priors_sec_, &err);
  } else {
    err = "runtime priors path is empty";
  }

  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Obstacle prior update (WAIT): type='" << type
      << "', actual_wait=" << actual_wait_sec << "s"
      << ", prior_old=" << old_prior << "s"
      << ", prior_new=" << new_prior << "s"
      << ", alpha=" << alpha
      << ", persisted=" << (persisted ? "true" : "false")
      << (persisted ? "" : (", err=" + err))
      << (obstacle_priors_runtime_path_.empty() ? "" : (", runtime='" + obstacle_priors_runtime_path_ + "'"));
}

/**
 * Estimate the spatial extent (width) of obstacles from the occupancy grid.
 * This is used to determine how much of the path is blocked by obstacles.
 *
 * @return The estimated obstacle extent in meters along the path direction.
 */
double Navigator::estimateObstacleExtentFromGrid() const {
  // Default extent if we can't compute from grid data
  constexpr double kDefaultExtent = 0.5;

  // Validate grid data exists and has valid resolution
  if (last_obstacle_grid_.data.empty() || last_obstacle_grid_.info.resolution <= 0.0)
    return kDefaultExtent;

  const auto &info = last_obstacle_grid_.info;
  const double res = info.resolution;  // Grid cell size in meters
  const double origin_x = info.origin.position.x;  // Grid origin X coordinate
  bool found = false;

  // Initialize bounds to find min/max X coordinates of obstacles
  double min_x = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();

  // Scan through all grid cells
  for (uint32_t y = 0; y < info.height; ++y) {
    for (uint32_t x = 0; x < info.width; ++x) {
      const int idx = y * info.width + x;

      // Bounds checking
      if (idx < 0 || static_cast<size_t>(idx) >= last_obstacle_grid_.data.size())
        continue;

      // Only consider on-path obstacles (value 100 from path_obstacle_detector)
      if (last_obstacle_grid_.data[idx] != 100) continue;

      // Convert grid coordinates to world coordinates
      // cell_x = origin_x + (x + 0.5) * resolution
      const double cell_x = origin_x + (static_cast<double>(x) + 0.5) * res;

      // Update bounds
      min_x = std::min(min_x, cell_x);
      max_x = std::max(max_x, cell_x);
      found = true;
    }
  }

  // If no on-path obstacles found, return default
  if (!found) return kDefaultExtent;

  // Calculate extent: distance from leftmost to rightmost obstacle
  // Add resolution to account for cell width, ensure at least 1 cell
  const double extent = std::max(res, max_x - min_x + res);

  // Ensure minimum reasonable extent (at least 10cm)
  return std::max(0.1, extent);
}

/// HSHMAT: Build per-edge delay costs based on the latest obstacle grid.
/// - Edges that pass through "red" cells (on-path obstacles) receive a delay
///   based on the current obstacle type (person/chair).
/// - Edges that pass through "orange" cells (off-path connected region) within
///   the local costmap window receive a very large delay, so that Dijkstra
///   will strongly prefer routes that avoid them.
///
/// Chain: VertexId -> trunk frame -> lidar frame -> grid cell
/// TF publishes "loc vertex frame" for the trunk vertex.
///
/// NOTE: This function is conservative and may examine many edges; it is only
/// called when we are committed to rerouting.
static std::unordered_map<vtr::tactic::EdgeId, double>
buildObstacleEdgeDelays(
    const tactic::Graph::Ptr &graph,
    const GraphMapServer::Ptr &graph_map_server,
    const nav_msgs::msg::OccupancyGrid &grid,
    const std::vector<vtr::tactic::EdgeId> &active_edges,
    const double active_delay_sec,
    const double huge_delay_sec,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const vtr::tactic::VertexId &trunk_vid,
    const double screening_radius_m) {
  using vtr::tactic::EdgeId;
  std::unordered_map<EdgeId, double> delays;

  if (!graph || grid.data.empty() || grid.info.resolution <= 0.0)
    return delays;

  if (!trunk_vid.isValid()) {
    CLOG(WARNING, "navigation")
        << "HSHMAT: buildObstacleEdgeDelays called with invalid trunk vertex";
    return delays;
  }

  // Optional screening radius: only apply grid-based screening to edges near the robot.
  // Note: active edges are always included regardless of radius.

  // Build a set of active edges for fast lookup
  std::unordered_set<EdgeId> active_edges_set(active_edges.begin(), active_edges.end());

  const auto &info = grid.info;
  const double res = info.resolution;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const uint32_t w = info.width;
  const uint32_t h = info.height;

  // TF publishes trunk vertex as "loc vertex frame"
  const std::string trunk_frame = "loc vertex frame";
  
  // Use the occupancy grid's published frame.
  const std::string grid_frame = grid.header.frame_id;
  
  // ===== CRITICAL: Ensure vertex transforms are cached =====
  // Route screening requires transforms for ALL vertices that might be screened.
  // ensureVertexTransformsCached() populates any missing transforms via graph traversal.
  if (graph_map_server) {
    graph_map_server->ensureVertexTransformsCached();
  }

  // Get transform from trunk frame to grid frame
  geometry_msgs::msg::TransformStamped tf_trunk_to_grid;
  bool have_tf = false;
  if (tf_buffer) {
    try {
      tf_trunk_to_grid = tf_buffer->lookupTransform(
          grid_frame, trunk_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
      have_tf = true;
      CLOG(DEBUG, "navigation")
          << "HSHMAT-TF: Got transform " << trunk_frame << " -> " << grid_frame;
    } catch (const tf2::TransformException &ex) {
      // No TF: can't check grid for alternative paths; active edges still get delays below.
    }
  } else {
    CLOG(WARNING, "navigation")
        << "HSHMAT: No TF buffer, assigning active_delay to all active edges without grid check";
    // If no TF buffer, fall back to assigning active_delay_sec to all active_edges
    for (const auto &e : active_edges) {
      delays[e] = active_delay_sec;
    }
    CLOG(INFO, "navigation")
        << "HSHMAT: Added " << active_edges.size() << " active edges with delay="
        << active_delay_sec << "s (no TF buffer, no grid check)";
    return delays;
  }

  auto inBounds = [&](int ix, int iy) {
    return ix >= 0 && iy >= 0 &&
           ix < static_cast<int>(w) && iy < static_cast<int>(h);
  };

  // Transform a point from trunk frame to grid/eval frame.
  auto transformPoint = [&](double x_trunk, double y_trunk, double &x_grid, double &y_grid) -> bool {
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header.frame_id = trunk_frame;
    pt_in.point.x = x_trunk;
    pt_in.point.y = y_trunk;
    pt_in.point.z = 0.0;
    try {
      tf2::doTransform(pt_in, pt_out, tf_trunk_to_grid);
      x_grid = pt_out.point.x;
      y_grid = pt_out.point.y;
      return true;
    } catch (...) {
      return false;
    }
  };

  auto edgeHitsCellType = [&](const EdgeId &e,
                              bool &hits_red,
                              bool &hits_orange) -> void {
    hits_red = false;
    hits_orange = false;

    auto edge_ptr = graph->at(e);
    if (!edge_ptr) return;

    const auto v1 = e.id1();
    const auto v2 = e.id2();
    double x1_trunk, y1_trunk, t1;
    double x2_trunk, y2_trunk, t2;
    try {
      // Get vertex positions relative to trunk vertex
      std::tie(x1_trunk, y1_trunk, t1) = graph_map_server->getVertexRelativeTo(v1, trunk_vid);
      std::tie(x2_trunk, y2_trunk, t2) = graph_map_server->getVertexRelativeTo(v2, trunk_vid);
    } catch (...) {
      return;
    }

    // Transform vertices from trunk frame to grid frame
    double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
    if (!transformPoint(x1_trunk, y1_trunk, x1, y1) ||
        !transformPoint(x2_trunk, y2_trunk, x2, y2)) {
      return;
    }

    // Optional screening radius: only apply grid-based screening to edges near the robot.
    //
    // Semantics:
    // - screening_radius_m == 0.0 => screen 0m ahead => ignore all non-active edges.
    // - screening_radius_m  > 0.0 => only screen edges within this radius of the robot (in trunk frame).
    //
    // Note: active edges are always included regardless of radius.
    if (active_edges_set.count(e) == 0) {
      const double d1 = std::sqrt(x1 * x1 + y1 * y1);
      const double d2 = std::sqrt(x2 * x2 + y2 * y2);
      if (std::min(d1, d2) >= screening_radius_m) {
        return;
      }
    }

    // Check segment against the grid window.

    // Quick AABB check against grid bounds
    const double min_x = std::min(x1, x2);
    const double max_x = std::max(x1, x2);
    const double min_y = std::min(y1, y2);
    const double max_y = std::max(y1, y2);

    const double grid_min_x = origin_x;
    const double grid_max_x = origin_x + static_cast<double>(w) * res;
    const double grid_min_y = origin_y;
    const double grid_max_y = origin_y + static_cast<double>(h) * res;

    if (max_x < grid_min_x || min_x > grid_max_x ||
        max_y < grid_min_y || min_y > grid_max_y) {
      return;
    }

    // Sample along the segment between v1 and v2 inside the grid window.
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length < 1e-3) return;

    const int num_samples =
        std::max(1, static_cast<int>(std::ceil(length / res)));
    for (int k = 0; k <= num_samples; ++k) {
      const double t = static_cast<double>(k) / num_samples;
      const double sx = x1 + t * dx;
      const double sy = y1 + t * dy;
      const int ix = static_cast<int>(std::floor((sx - origin_x) / res));
      const int iy = static_cast<int>(std::floor((sy - origin_y) / res));
      if (!inBounds(ix, iy)) continue;
      const int idx = iy * static_cast<int>(w) + ix;
      if (idx < 0 || static_cast<size_t>(idx) >= grid.data.size()) continue;

      const int8_t cell = grid.data[idx];
      if (cell == 100) hits_red = true;
      else if (cell == 50) hits_orange = true;

      if (hits_red && hits_orange) return;
    }
  };

  // FIRST: Assign active_delay_sec to all active edges (initially detected blocking edges).
  // These edges were already identified as blocking the current path, no grid check needed.
  for (const auto &e : active_edges) {
    delays[e] = active_delay_sec;
  }
  const int active_count = static_cast<int>(active_edges.size());
  CLOG(INFO, "navigation")
      << "HSHMAT: Added " << active_count << " active edges with delay="
      << active_delay_sec << "s (no grid check needed)";

  // SECOND: Scan all OTHER edges (not active edges) and check if they hit RED or ORANGE cells.
  // If they do, assign huge_delay_sec to discourage using those alternative paths.
  int edges_scanned = 0;
  int edges_with_huge_delay = 0;
  
  if (have_tf) {
  auto guard = graph->guard();
  for (auto it = graph->beginEdge(), ite = graph->endEdge(); it != ite; ++it) {
      const EdgeId e = it->id();
    ++edges_scanned;
      
      // Skip active edges - they're already handled above
      if (active_edges_set.find(e) != active_edges_set.end()) {
        continue;
      }
      
      // Check if this edge hits RED or ORANGE cells
    bool hits_red = false, hits_orange = false;
    edgeHitsCellType(e, hits_red, hits_orange);
      
    if (hits_red || hits_orange) {
      delays[e] = huge_delay_sec;
      ++edges_with_huge_delay;
      CLOG(DEBUG, "navigation")
          << "HSHMAT-EDGE-DELAY: Edge " << e << " hits "
          << (hits_red ? "RED" : "") << (hits_orange ? "ORANGE" : "")
          << " -> huge_delay=" << huge_delay_sec << "s";
    }
    }
  } else {
    // No TF: can't check grid, so only active edges get delays
    CLOG(WARNING, "navigation")
        << "HSHMAT: No TF, skipping grid-based edge scanning for alternative paths";
  }

  // Summary logging
  CLOG(INFO, "navigation")
      << "HSHMAT-EDGE-DELAYS SUMMARY: grid_frame=" << grid.header.frame_id
      << ", grid_size=" << w << "x" << h
      << ", resolution=" << res << "m"
      << ", origin=(" << origin_x << ", " << origin_y << ")"
      << ", have_tf=" << (have_tf ? "true" : "false")
      << ", edges_scanned=" << edges_scanned
      << ", active_edges=" << active_count
      << " (delay=" << active_delay_sec << "s, no grid check)"
      << ", huge_delay_edges=" << edges_with_huge_delay
      << " (delay=" << huge_delay_sec << "s, from grid check)"
      << ", total_delayed=" << delays.size();

  return delays;
}

/// HSHMAT: Opportunistically verify TDSP remembered blockages against the current
/// local obstacle grid within a radius around the robot. If an edge was predicted
/// blocked until some future time, but the local grid indicates it does not
/// intersect any obstacle cells (RED/ORANGE), we erase its blockage interval.
///
/// This is intentionally conservative:
/// - Only checks edges whose endpoints are near the robot (within radius_m).
/// - Only checks edges that fall inside the current grid window (via the same
///   sampling used by buildObstacleEdgeDelays).
static size_t pruneRememberedBlockagesUsingGrid(
    std::unordered_map<vtr::tactic::EdgeId, vtr::navigation::EdgeBlockageInterval> &blockages,
    const tactic::Graph::Ptr &graph,
    const GraphMapServer::Ptr &graph_map_server,
    const nav_msgs::msg::OccupancyGrid &grid,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const vtr::tactic::VertexId &trunk_vid,
    const double now_sec,
    const double radius_m) {
  using vtr::tactic::EdgeId;

  if (!graph || !graph_map_server || grid.data.empty() || grid.info.resolution <= 0.0)
    return 0;
  if (!tf_buffer) return 0;
  if (!trunk_vid.isValid()) return 0;
  if (radius_m <= 0.0) return 0;

  const auto &info = grid.info;
  const double res = info.resolution;
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const uint32_t w = info.width;
  const uint32_t h = info.height;

  const std::string trunk_frame = "loc vertex frame";
  const std::string grid_frame = grid.header.frame_id;

  geometry_msgs::msg::TransformStamped tf_trunk_to_grid;
  try {
    tf_trunk_to_grid = tf_buffer->lookupTransform(
        grid_frame, trunk_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
  } catch (...) {
    return 0;
  }

  auto inBounds = [&](int ix, int iy) {
    return ix >= 0 && iy >= 0 && ix < static_cast<int>(w) && iy < static_cast<int>(h);
  };

  auto transformPoint = [&](double x_trunk, double y_trunk, double &x_grid, double &y_grid) -> bool {
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header.frame_id = trunk_frame;
    pt_in.point.x = x_trunk;
    pt_in.point.y = y_trunk;
    pt_in.point.z = 0.0;
    try {
      tf2::doTransform(pt_in, pt_out, tf_trunk_to_grid);
      x_grid = pt_out.point.x;
      y_grid = pt_out.point.y;
      return true;
    } catch (...) {
      return false;
    }
  };

  auto edgeHitsRedOrOrange = [&](const EdgeId &e) -> bool {
    const auto v1 = e.id1();
    const auto v2 = e.id2();

    double x1_trunk, y1_trunk, t1;
    double x2_trunk, y2_trunk, t2;
    try {
      std::tie(x1_trunk, y1_trunk, t1) = graph_map_server->getVertexRelativeTo(v1, trunk_vid);
      std::tie(x2_trunk, y2_trunk, t2) = graph_map_server->getVertexRelativeTo(v2, trunk_vid);
    } catch (...) {
      return true;  // if we can't test, keep blockage (conservative)
    }

    double x1, y1, x2, y2;
    if (!transformPoint(x1_trunk, y1_trunk, x1, y1) ||
        !transformPoint(x2_trunk, y2_trunk, x2, y2)) {
      return true;  // conservative
    }

    // Only consider edges near the robot within radius_m.
    const double d1 = std::sqrt(x1 * x1 + y1 * y1);
    const double d2 = std::sqrt(x2 * x2 + y2 * y2);
    if (std::min(d1, d2) > radius_m) return true;  // out of scope -> keep blockage

    // AABB check against grid bounds
    const double min_x = std::min(x1, x2);
    const double max_x = std::max(x1, x2);
    const double min_y = std::min(y1, y2);
    const double max_y = std::max(y1, y2);

    const double grid_min_x = origin_x;
    const double grid_max_x = origin_x + static_cast<double>(w) * res;
    const double grid_min_y = origin_y;
    const double grid_max_y = origin_y + static_cast<double>(h) * res;

    if (max_x < grid_min_x || min_x > grid_max_x || max_y < grid_min_y || min_y > grid_max_y) {
      return true;  // not in grid window -> can't validate -> keep blockage
    }

    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length < 1e-3) return true;

    const int num_samples = std::max(1, static_cast<int>(std::ceil(length / res)));
    for (int k = 0; k <= num_samples; ++k) {
      const double t = static_cast<double>(k) / num_samples;
      const double sx = x1 + t * dx;
      const double sy = y1 + t * dy;
      const int ix = static_cast<int>(std::floor((sx - origin_x) / res));
      const int iy = static_cast<int>(std::floor((sy - origin_y) / res));
      if (!inBounds(ix, iy)) continue;
      const int idx = iy * static_cast<int>(w) + ix;
      if (idx < 0 || static_cast<size_t>(idx) >= grid.data.size()) continue;
      const int8_t cell = grid.data[idx];
      if (cell == 100 || cell == 50) return true;
    }

    // No RED/ORANGE intersections found in the current local grid -> consider free now.
    return false;
  };

  size_t removed = 0;
  for (auto it = blockages.begin(); it != blockages.end();) {
    if (it->second.end_sec <= now_sec) {
      it = blockages.erase(it);
      ++removed;
      continue;
    }

    if (!edgeHitsRedOrOrange(it->first)) {
      it = blockages.erase(it);
      ++removed;
      continue;
    }
    ++it;
  }

  return removed;
}

/**
 * Initiates the rerouting process when obstacles are detected on the planned path.
 *
 * This function performs several key steps:
 * 1. Validates that rerouting is enabled and we're in an appropriate state
 * 2. Identifies which graph edges are affected by the obstacle
 * 3. Configures the route planner to avoid blocked paths
 * 4. Signals the mission state machine to execute the reroute
 *
 * The function supports both Dijkstra (BFS) and TDSP (Time-Dependent) planners
 * with different strategies for handling temporary vs permanent blockages.
 */
void Navigator::triggerReroute() {
  // HSHMAT: Validate FSM state and config - should be Rerouting
  {
    LockGuard lock(obstacle_mutex_);
    if (obstacle_state_ != ObstacleState::Rerouting) {
      CLOG(WARNING, "mission.state_machine")
          << "HSHMAT: triggerReroute called but FSM is "
          << obstacleStateToString(obstacle_state_) << "; ignoring.";
      return;
    }
  }

  // Check if rerouting is enabled in configuration
  const bool reroute_enable = route_cfg_.enable_reroute;
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: triggerReroute - enable_reroute="
      << (reroute_enable ? "true" : "false");
  if (!reroute_enable) {
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Rerouting disabled. Falling back to wait for obstacle to clear.";
    speak("Rerouting disabled. Waiting for obstacle to clear.");
    {
      LockGuard lock(obstacle_mutex_);
      obstacle_state_ = ObstacleState::Waiting;
      current_W_star_ = std::numeric_limits<double>::infinity();
    }
    return;
  }

  // Validate we're in navigation mode (Repeat.Follow, etc.)
  try {
    const std::string curr = state_machine_->name();
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Checking state mode: " << curr;
    const bool in_repeat = curr.find("Repeat") != std::string::npos;
    if (!in_repeat) {
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: Not in Repeat mode (" << curr << "). Falling back to wait for obstacle to clear.";
      speak("Cannot reroute now. Waiting for obstacle to clear.");
      {
        LockGuard lock(obstacle_mutex_);
        obstacle_state_ = ObstacleState::Waiting;
        current_W_star_ = std::numeric_limits<double>::infinity();
      }
      return;
    }
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: In Repeat mode (" << curr << "), proceeding with reroute.";
  } catch (...) {
    CLOG(WARNING, "mission.state_machine")
        << "HSHMAT: Failed to get state machine name. Falling back to wait.";
    speak("Cannot reroute now. Waiting for obstacle to clear.");
    {
      LockGuard lock(obstacle_mutex_);
      obstacle_state_ = ObstacleState::Waiting;
      current_W_star_ = std::numeric_limits<double>::infinity();
    }
    return;
  }

  // HSHMAT: Proceeding with reroute logic (FSM stays in Rerouting until route changes)
  
  // HSHMAT: Save snapshot of current route to detect when planner produces different route
  {
    LockGuard lock(obstacle_mutex_);
    reroute_snapshot_route_ = following_route_ids_;
    awaiting_new_route_ = true;
  }
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Saved route snapshot (" << reroute_snapshot_route_.size() 
      << " vertices) for reroute detection.";

  // ===== STEP 4b: RECOMPUTE BLOCKED EDGES WITH CURRENT LOCALIZATION =====
  // Recompute blocked edges now, using the current (refined) localization.
  // During the speech delay, localization may have caught up with the true position.
  // Using stale blocked edges from detection time can cause issues when the robot's
  // localization estimate changed (e.g., robot was at vertex 16 at detection but 
  // localization refined to vertex 14 by planning time).
  EdgeIdSet fresh_blocked = computeBlockedEdges();
  
  // Also update current_blocked_edges_ for consistency with route validation
  {
    LockGuard lock(obstacle_mutex_);
    current_blocked_edges_ = fresh_blocked;
  }
  
  std::vector<vtr::tactic::EdgeId> affected_edges;
  affected_edges.reserve(fresh_blocked.size());
  for (const auto& e : fresh_blocked) {
    affected_edges.push_back(e);
  }
  
  {
    std::stringstream ss;
    ss << "HSHMAT: Recomputed blocked edges (" << affected_edges.size() << "): ";
    for (const auto& e : affected_edges) ss << e << " ";
    ss << " [current_v=" << getCurrentVertex() << "]";
    CLOG(INFO, "mission.state_machine") << ss.str();
  }

  // Get trunk vertex for TF lookups (still need current position for transforms)
  vtr::tactic::VertexId trunk_vid;
  try {
    const auto loc = tactic_->getPersistentLoc();
    trunk_vid = loc.v;
  } catch (...) {
    trunk_vid = vtr::tactic::VertexId::Invalid();
  }
  
  if (affected_edges.empty()) {
    CLOG(WARNING, "mission.state_machine")
        << "HSHMAT: No blocked edges from detection time - cannot reroute";
  }
  
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Trunk vertex for TF: " << trunk_vid;

  // ===== DEBUGGING: LOG OBSTACLE GRID INFORMATION =====
  // Log details about the obstacle grid for debugging rerouting issues
  if (!last_obstacle_grid_.data.empty()) {
    const auto &info = last_obstacle_grid_.info;
    const std::string grid_frame = last_obstacle_grid_.header.frame_id;
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Obstacle grid info - frame=" << grid_frame
        << ", size=" << info.width << "x" << info.height
        << ", resolution=" << info.resolution << "m"
        << ", origin=(" << info.origin.position.x << ", " << info.origin.position.y << ")";
    
    // Check if coordinate transforms are available for grid operations
    // This determines if we can map between robot poses and obstacle grid coordinates
    if (tf_buffer_) {
      const std::string trunk_frame = "loc vertex frame";
      try {
        auto tf = tf_buffer_->lookupTransform(grid_frame, trunk_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: TF available: " << trunk_frame << " -> " << grid_frame
            << ", translation=(" << tf.transform.translation.x << ", "
            << tf.transform.translation.y << ", " << tf.transform.translation.z << ")";
      } catch (const tf2::TransformException &ex) {
        CLOG(WARNING, "mission.state_machine")
            << "HSHMAT: TF NOT available: " << trunk_frame << " -> " << grid_frame
            << " (" << ex.what() << ")";
      }
    }
  }

  // ===== STEP 6: CONFIGURE ROUTE PLANNER FOR OBSTACLE AVOIDANCE =====
  // Set up the path planner to avoid the identified blocked edges
  // Supports two planner types with different obstacle handling strategies:
  //
  // DIJKSTRA MODE: Simple cost-based routing
  // - Assigns fixed time delays to blocked edges
  // - Planner finds lowest-cost path around obstacles
  //
  // TDSP MODE: Time-aware routing with clearance prediction
  // - Uses time intervals for temporary blockages
  // - Can plan paths that arrive after obstacles clear
  //
  // ROUTE SCREENING: Prevents rerouting onto other blocked paths
  // - Scans obstacle grid for additional blocked edges
  // - Assigns huge penalties to prevent using obstructed alternatives
  try {
    const double huge_delay_sec = 1e6;  // effectively "blocked"
    const double dur_sec = expectedObstacleDurationSeconds();
    const double now_sec = node_->get_clock()->now().seconds();

    // ALWAYS_DETOUR is memoryless: clear all remembered blockages so only the
    // current obstacle is avoided (no memory of past obstacles).
    if (wait_strategy_ && wait_strategy_->type() == StrategyType::ALWAYS_DETOUR) {
      edge_blockages_.clear();
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: always_detour - cleared edge_blockages_ (memoryless)";
    } else {
      // Other strategies: prune only expired blockages (best-effort).
      for (auto it = edge_blockages_.begin(); it != edge_blockages_.end();) {
        if (it->second.end_sec <= now_sec) it = edge_blockages_.erase(it);
        else ++it;
      }
    }

    // ===== GREEDY CTP: Collect permanently banned edges =====
    // For greedy_ctp strategy, all edges that have been blocked in this mission
    // should remain banned permanently (until mission ends).
    // 
    // Note: The affected_edges were already added to banned_edges_ during 
    // computeWaitTime() in startObstacleEpisode(). We call banEdgePermanently()
    // again here for safety (it's idempotent since banned_edges_ is a set).
    std::vector<vtr::tactic::EdgeId> permanently_banned_edges;
    if (wait_strategy_ && wait_strategy_->type() == StrategyType::GREEDY_CTP) {
      auto* greedy = dynamic_cast<GreedyCTPStrategy*>(wait_strategy_.get());
      if (greedy) {
        // Ban edges NOW with accurate localization (not at detection time)
        for (const auto& e : affected_edges) {
          greedy->banEdgePermanently(e);
        }
        // Collect all permanently banned edges
        for (const auto& edge : greedy->getBannedEdges()) {
          permanently_banned_edges.push_back(edge);
        }
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: greedy_ctp - permanently banned edges: " << permanently_banned_edges.size();
      }
    }

    // ===== CONFIGURE TDSP PLANNER =====
    if (auto tdsp = std::dynamic_pointer_cast<vtr::route_planning::TDSPPlanner>(route_planner_)) {
      const StrategyType st = wait_strategy_ ? wait_strategy_->type() : StrategyType::ALWAYS_WAIT;
      
      // Build the set of edges to ban
      std::unordered_set<vtr::tactic::EdgeId> edges_to_ban;
      
      if (st == StrategyType::GREEDY_CTP) {
        // GREEDY_CTP: Simply ban all permanently banned edges. That's it.
        for (const auto& e : permanently_banned_edges) {
          edges_to_ban.insert(e);
        }
        tdsp->setBannedEdges(edges_to_ban);
        tdsp->setStaticEdgeDelays({});
        tdsp->setEdgeBlockages({});
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: greedy_ctp - banned " << edges_to_ban.size() << " edges";
            
      } else if (st == StrategyType::ALWAYS_DETOUR || st == StrategyType::RULE_BASED) {
        // ALWAYS_DETOUR / RULE_BASED: Ban current blocked edges only (memoryless)
        for (const auto& e : affected_edges) {
          edges_to_ban.insert(e);
        }
        tdsp->setBannedEdges(edges_to_ban);
        tdsp->setStaticEdgeDelays({});
        tdsp->setEdgeBlockages({});
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: " << strategyTypeToString(st) << " - banned " << edges_to_ban.size() << " edges";
            
      } else if (st == StrategyType::LEARNED) {
        // LEARNED: Use time-dependent blockages (can wait for obstacle to clear)
        for (const auto &e : affected_edges) {
          edge_blockages_[e] = EdgeBlockageInterval{now_sec, now_sec + dur_sec};
        }
        std::unordered_map<vtr::tactic::EdgeId, vtr::route_planning::TDSPPlanner::BlockageInterval> b;
        for (const auto &kv : edge_blockages_) {
          b.emplace(kv.first, vtr::route_planning::TDSPPlanner::BlockageInterval{kv.second.start_sec, kv.second.end_sec});
        }
        tdsp->setBannedEdges({});
        tdsp->setStaticEdgeDelays({});
        tdsp->setEdgeBlockages(b);
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: learned - " << b.size() << " time-dependent blockages";
            
      } else {
        // ALWAYS_WAIT or unknown: no special planner config
        tdsp->setBannedEdges({});
        tdsp->setStaticEdgeDelays({});
        tdsp->setEdgeBlockages({});
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: " << strategyTypeToString(st) << " - no edge bans";
      }
      
      tdsp->setNominalSpeed(route_cfg_.nominal_speed_mps);
    // ===== CONFIGURE BFS PLANNER =====
    } else if (auto bfs_ptr = std::dynamic_pointer_cast<vtr::route_planning::BFSPlanner>(route_planner_)) {
      const StrategyType st = wait_strategy_ ? wait_strategy_->type() : StrategyType::ALWAYS_WAIT;
      
      // Build edges to ban
      std::vector<vtr::tactic::EdgeId> edges_to_ban;
      if (st == StrategyType::GREEDY_CTP) {
        for (const auto& e : permanently_banned_edges) {
          edges_to_ban.push_back(e);
        }
      } else if (st == StrategyType::ALWAYS_DETOUR || st == StrategyType::RULE_BASED) {
        for (const auto& e : affected_edges) {
          edges_to_ban.push_back(e);
        }
      }
      
      bfs_ptr->setBannedEdges(edges_to_ban);
      bfs_ptr->setNominalSpeed(route_cfg_.nominal_speed_mps);
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: BFS planner - banned " << edges_to_ban.size() << " edges";
    } else {
      // Fallback: Unknown planner type - log warning but continue
      CLOG(WARNING, "mission.state_machine")
          << "HSHMAT: Unknown route planner type; cannot apply reroute costs.";
    }

  } catch (const std::exception &e) {
    // Planner configuration failed - log but don't crash the system
    CLOG(WARNING, "navigation")
        << "Failed to configure replanner in triggerReroute: " << e.what();
  }

  // ===== STEP 7: NOTIFY MISSION STATE MACHINE =====
  // Signal the high-level mission planner that obstacle rerouting is ready
  // The state machine will handle the actual route transition
  try {
    CLOG(INFO, "mission.state_machine")
        << "⏱ TIMING: Sending Signal::ObstacleDetected to state machine NOW";
    state_machine_->handle(std::make_shared<mission_planning::Event>(
        mission_planning::Signal::ObstacleDetected));
    CLOG(INFO, "mission.state_machine")
        << "⏱ TIMING: Signal::ObstacleDetected handled, state machine processing...";
  } catch (const std::exception &e) {
    CLOG(WARNING, "navigation")
        << "Failed to send ObstacleDetected: " << e.what();
  }
}


Navigator::~Navigator() {
  UniqueLock lock(mutex_);
  // send stop signal
  stop_ = true;
  cv_set_or_stop_.notify_all();
  //
  cv_thread_finish_.wait(lock, [this] { return thread_count_ == 0; });
  if (process_thread_.joinable()) process_thread_.join();

  /// explicitly destruct each building block in order to emplasize the order of
  /// destruction -> although it is not necessary since we declare them in the
  /// correct order in the class
  state_machine_.reset();
  mission_server_.reset();
  route_planner_.reset();
  path_planner_.reset();
  tactic_.reset();
  graph_.reset();
  graph_map_server_.reset();

  CLOG(INFO, "navigation") << "VT&R3 destruction done! Bye-bye.";
}

void Navigator::process() {
  el::Helpers::setThreadName("navigation.sensor_input");
  CLOG(DEBUG, "navigation.sensor_input") << "Starting the sensor input thread.";
  while (true) {
    UniqueLock lock(mutex_);

    cv_set_or_stop_.wait(lock, [this] { return stop_ || (!queue_.empty()); });
    if (stop_) {
      --thread_count_;
      CLOG(INFO, "navigation.sensor_input")
          << "Stopping the sensor input thread.";
      cv_thread_finish_.notify_all();
      return;
    }

    // get the front in queue
    auto qdata0 = queue_.front();
    queue_.pop();

    // unlock the queue so that new data can be added
    lock.unlock();

    // execute the pipeline
    tactic_->input(qdata0);
 
    // handle any transitions triggered by changes in localization status
    state_machine_->handle();
  };
}

void Navigator::envInfoCallback(const tactic::EnvInfo::SharedPtr msg) {
  UniqueLock lock(mutex_);
  CLOG(DEBUG, "navigation") << "Received environment info update.";
  env_info_ = *msg;
}

#ifdef VTR_ENABLE_LIDAR
void Navigator::lidarCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  // set the timestamp
  Timestamp timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

  // CLOG(DEBUG, "navigation") << "Received a lidar pointcloud with stamp " << timestamp;

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<lidar::LidarQueryCache>();

  LockGuard lock(mutex_);

  /// Discard old frames if our queue is too big
  if (queue_.size() > max_queue_size_) {
    // CLOG(WARNING, "navigation")
    //     << "Dropping pointcloud message " << *queue_.front()->stamp << " because the queue is full.";
    queue_.pop();
  }


  // some modules require node for visualization
  query_data->node = node_;

  query_data->stamp.emplace(timestamp);

  // add the current environment info
  query_data->env_info.emplace(env_info_);

  // put in the pointcloud msg pointer into query data
  query_data->pointcloud_msg = msg;

  query_data->gyro_msgs.emplace(gyro_msgs_);
  gyro_msgs_.clear();

  // fill in the vehicle to sensor transform and frame names
  query_data->T_s_r_gyro.emplace(T_gyro_robot_);
  query_data->T_s_r.emplace(T_lidar_robot_);

  // add to the queue and notify the processing thread
  queue_.push(query_data);
  cv_set_or_stop_.notify_one();
};
#endif

// Radar callback: Similar to Lidar, we need to know what to do when a radar message is received
#ifdef VTR_ENABLE_RADAR
void Navigator::radarCallback(
    const navtech_msgs::msg::RadarBScanMsg::SharedPtr msg) {

  // set the timestamp
  Timestamp timestamp_radar = msg->b_scan_img.header.stamp.sec * 1e9 + msg->b_scan_img.header.stamp.nanosec;

  CLOG(DEBUG, "navigation") << "Received a radar Image with stamp " << timestamp_radar;

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<radar::RadarQueryCache>();

  // CLOG(DEBUG, "navigation") << "Sam: In the callback: Created radar query cache";

  LockGuard lock(mutex_);

  // Drop frames if queue is too big and if it is not a scan message (just gyro)
  if (queue_.size() > max_queue_size_ && !(std::dynamic_pointer_cast<radar::RadarQueryCache>(queue_.front())->scan_msg)) {
    CLOG(WARNING, "navigation")
        << "Dropping old message because the queue is full.";
    queue_.pop();
  }


  // some modules require node for visualization
  query_data->node = node_;

  // set the timestamp
  // Timestamp timestamp = msg_r->header.stamp.sec * 1e9 + msg_r->header.stamp.nanosec;
  query_data->stamp.emplace(timestamp_radar);

  // add the current environment info
  query_data->env_info.emplace(env_info_);

  // put in the radar msg pointer into query data
  query_data->scan_msg = msg;

  query_data->gyro_msgs.emplace(gyro_msgs_);
  gyro_msgs_.clear();

  // fill in the vehicle to sensor transform and frame names
  query_data->T_s_r_gyro.emplace(T_gyro_robot_);
  query_data->T_s_r.emplace(T_radar_robot_);

  // add to the queue and notify the processing thread
  CLOG(DEBUG, "navigation") << "Sam: In the callback: Adding radar message to the queue";
  queue_.push(query_data);

  cv_set_or_stop_.notify_one();
}

#endif


void Navigator::gyroCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg) {

  // set the timestamp
  Timestamp timestamp_gyro = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

  CLOG(DEBUG, "navigation") << "Received gyro data with stamp " << timestamp_gyro;

  // Convert message to query_data format and store into query_data
  // auto query_data = std::make_shared<radar::RadarQueryCache>();

  LockGuard lock(mutex_);
  msg->angular_velocity.x -= gyro_bias_[0];
  msg->angular_velocity.y -= gyro_bias_[1];
  msg->angular_velocity.z -= gyro_bias_[2];
  gyro_msgs_.push_back(*msg);
}

#ifdef VTR_ENABLE_VISION
void Navigator::cameraCallback(
    const sensor_msgs::msg::Image::SharedPtr msg_r, const sensor_msgs::msg::Image::SharedPtr msg_l) {
  LockGuard lock(mutex_);
  CLOG(DEBUG, "navigation") << "Received an image.";

  /// Discard old frames if our queue is too big
  if (queue_.size() > max_queue_size_) {
    CLOG(WARNING, "navigation")
        << "Dropping old image " << *queue_.front()->stamp << " because the queue is full.";
    queue_.pop();
  }

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<vision::CameraQueryCache>();

  // some modules require node for visualization
  query_data->node = node_;

  query_data->left_image = msg_l;
  query_data->right_image = msg_r;

  // set the timestamp
  Timestamp timestamp = msg_r->header.stamp.sec * 1e9 + msg_r->header.stamp.nanosec;
  query_data->stamp.emplace(timestamp);

  // add the current environment info
  query_data->env_info.emplace(env_info_);


  // fill in the vehicle to sensor transform and frame names
  query_data->T_s_r.emplace(T_camera_robot_);


  // add to the queue and notify the processing thread
  queue_.push(query_data);
  cv_set_or_stop_.notify_one();
};
#endif

}  // namespace navigation
}  // namespace vtr

