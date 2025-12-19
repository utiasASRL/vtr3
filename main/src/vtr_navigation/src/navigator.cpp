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

std::string nowStampForFilename() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return ss.str();
}

std::string makeRuntimePriorsPathNextToBase(const std::string &base_path) {
  namespace fs = std::filesystem;
  const fs::path base(base_path);
  const fs::path dir = base.has_parent_path() ? base.parent_path() : fs::path(".");
  const int pid = static_cast<int>(::getpid());
  const std::string stamp = nowStampForFilename();
  const std::string fname = "obstacles_runtime_" + stamp + "_" + std::to_string(pid) + ".yaml";
  return (dir / fname).string();
}

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

  const std::string planner_name =
      route_cfg_.memory ? route_cfg_.planner_with_memory
                        : route_cfg_.planner_without_memory;

  if (planner_name == "time_dependent_shortest_path") {
    // TDSP (label-setting / Dijkstra-like on earliest-arrival labels).
    auto tdsp = std::make_shared<vtr::route_planning::TDSPPlanner>(graph_);
    tdsp->setNominalSpeed(route_cfg_.nominal_speed_mps);
    tdsp->setNowSecCallback([this]() { return node_->get_clock()->now().seconds(); });
    route_planner_ = tdsp;
  } else if (planner_name == "dijkstra") {
    // Dijkstra-like (BFSPlanner + masked Dijkstra + time-based weights).
    auto bfs = std::make_shared<vtr::route_planning::BFSPlanner>(graph_);
    bfs->setNominalSpeed(route_cfg_.nominal_speed_mps);
    bfs->setUseMasked(route_cfg_.enable_reroute);
    route_planner_ = bfs;
  } else {
    // Default to dijkstra for unknown planner names (safe fallback).
    CLOG(WARNING, "navigation")
        << "Unknown planner name '" << planner_name
        << "'. Falling back to 'dijkstra'.";
    auto bfs = std::make_shared<vtr::route_planning::BFSPlanner>(graph_);
    bfs->setNominalSpeed(route_cfg_.nominal_speed_mps);
    bfs->setUseMasked(route_cfg_.enable_reroute);
    route_planner_ = bfs;
  }

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
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  // robot frame
  robot_frame_ = node_->declare_parameter<std::string>("robot_frame", "robot");
  // environment info
  const auto env_info_topic = node_->declare_parameter<std::string>("env_info_topic", "env_info");
  env_info_sub_ = node_->create_subscription<tactic::EnvInfo>(env_info_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::envInfoCallback, this, std::placeholders::_1), sub_opt);

  max_queue_size_ = node->declare_parameter<int>("queue_size", max_queue_size_);

  // Hshmat: Initialize robot paused state
  robot_paused_ = false;
  waiting_for_reroute_ = false;
  processing_obstacle_ = false;
  obstacle_active_ = false;
  waiting_for_decision_logged_ = false;
  active_decision_.clear();
  decision_accepting_ = true;
  pending_clear_resume_ = false;
  pending_reroute_resume_ = false;
  const auto clock_type = node_->get_clock()->get_clock_type();
  last_decision_time_ = rclcpp::Time(0, 0, clock_type);
  obstacle_start_time_ = rclcpp::Time(0, 0, clock_type);
  
  // Hshmat: Declare and publish use_chatgpt parameter
  use_chatgpt_ = node_->declare_parameter<bool>("route_planning.use_chatgpt", true);
  default_decision_ = node_->declare_parameter<std::string>("route_planning.default_decision", std::string("wait"));
  if (default_decision_ != "wait" && default_decision_ != "reroute") {
    default_decision_ = "wait";
    CLOG(WARNING, "navigation") << "Invalid route_planning.default_decision; falling back to 'wait'";
  }
  use_chatgpt_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/vtr/use_chatgpt", rclcpp::SystemDefaultsQoS());
  // Publish once at startup
  {
    std_msgs::msg::Bool msg;
    msg.data = use_chatgpt_;
    use_chatgpt_pub_->publish(msg);
    CLOG(INFO, "navigation") << "ChatGPT enabled: " << (use_chatgpt_ ? "true" : "false")
                              << ", default_decision='" << default_decision_ << "'";
  }
  
  // Hshmat: Create publisher for emergency stop (zero velocity)
  pause_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/j100_0365/platform/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS());
  pause_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/vtr/navigation_pause", rclcpp::SystemDefaultsQoS());
  {
    std_msgs::msg::Bool msg;
    msg.data = false;
    pause_state_pub_->publish(msg);
  }
  
  // Hshmat: Subscribe to obstacle decision ("wait" or "reroute") from the
  // obstacle decision node. The decision node may internally use ChatGPT or
  // other logic, but from the navigator's perspective this is a generic
  // obstacle decision signal.
  obstacle_decision_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/vtr/chatgpt_decision", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg) return;
        auto now = node_->get_clock()->now();
        std::string decision = msg->data;
        std::transform(decision.begin(), decision.end(), decision.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        {
          LockGuard lock(chatgpt_mutex_);
          latest_obstacle_decision_ = decision;
          last_decision_time_ = now;
        }
        waiting_for_decision_ = false;
        processing_obstacle_ = false;  // Clear processing flag when decision arrives
        waiting_for_decision_logged_ = false;
        CLOG(INFO, "mission.state_machine") << "HSHMAT-TTS: Received obstacle decision from detector: '" << decision << "'";
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT-TTS: Robot says: 'Decision source "
            << (use_chatgpt_ ? "uses ChatGPT internally" : "does not use ChatGPT")
            << ". Decision: " << decision << ".'";
        if (decision == "abort") {
          // Decision node aborted this episode (e.g., insufficient data).
          // Clear the "waiting for decision" latch so obstacle_status=false can end the episode.
          active_decision_ = "abort";
          waiting_for_decision_ = false;
          processing_obstacle_ = false;
          waiting_for_decision_logged_ = false;
          wait_episode_start_sec_ = -1.0;

          // If the obstacle has already cleared (false-positive that we already ignored once),
          // proactively resume now or latch resume until decision_accepting_==true.
          if (!last_obstacle_status_msg_) {
            CLOG(INFO, "mission.state_machine")
                << "Navigator: received decision=abort and obstacle_status=false; ending episode and resuming.";
            obstacle_active_ = false;
            waiting_for_reroute_ = false;
            active_decision_.clear();

            if (robot_paused_) {
              if (decision_accepting_) {
                setRobotPaused(false);
              } else {
                pending_clear_resume_ = true;
              }
            }
          } else {
            CLOG(INFO, "mission.state_machine")
                << "Navigator: received decision=abort but obstacle_status=true; staying paused and waiting for retry/clear.";
          }
        } else if (decision == "wait") {
          // Enter WAIT mode and start measuring actual wait duration for online prior updates.
          active_decision_ = "wait";
          wait_episode_type_ = last_obstacle_type_;
          wait_episode_start_sec_ = now.seconds();
          // The obstacle_status callback already pauses the robot at episode start in ChatGPT mode,
          // but keep this idempotent to avoid any timing corner cases.
          setRobotPaused(true);
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: Decision=WAIT. Starting wait timer for obstacle_type='"
              << wait_episode_type_ << "'";
        } else if (decision == "reroute") {
          active_decision_ = "reroute";
          // Not a wait episode.
          wait_episode_start_sec_ = -1.0;
          CLOG(INFO, "mission.state_machine")
              << "Navigator: triggering reroute flow directly from decision subscriber.";
          triggerReroute();
        }
      });
  
  // Hshmat: Subscribe to obstacle distance (meters along path where obstacle detected)
  last_obstacle_distance_ = 0.0;
  obstacle_distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/vtr/obstacle_distance", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!msg) return;
        last_obstacle_distance_ = msg->data;
        CLOG(DEBUG, "mission.state_machine") << "📏 Obstacle distance updated: " << last_obstacle_distance_ << " m along path";
      }, sub_opt);

  // Optional: expected duration published by decision system when
  // route_planning.obstacle_costs.source == "vlm". Message is in seconds.
  obstacle_expected_duration_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/vtr/obstacle_expected_duration_sec", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!msg) return;
        last_obstacle_expected_duration_sec_ = msg->data;
      }, sub_opt);

  // Lookahead radii are configured via route_planning.* parameters (YAML).
  decision_accepting_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/vtr/decision_accepting", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg) return;
        decision_accepting_ = msg->data;
        if (!decision_accepting_) return;
        if (pending_clear_resume_) {
          pending_clear_resume_ = false;
          if (robot_paused_) {
            CLOG(INFO, "mission.state_machine") << "Navigator: decision node released hold after obstacle clear.";
            setRobotPaused(false);
          }
        }
        if (pending_reroute_resume_) {
          pending_reroute_resume_ = false;
          if (robot_paused_) {
            CLOG(INFO, "mission.state_machine") << "Navigator: decision node released hold after reroute.";
            setRobotPaused(false);
          }
        }
      });
  
  // Hshmat: Subscribe to obstacle status to trigger replanning during Repeat::Follow
  obstacle_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/vtr/obstacle_status", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg) return;
        last_obstacle_status_msg_ = msg->data;
        
        if (!msg->data) {
          if (waiting_for_decision_) {
            CLOG(INFO, "mission.state_machine") << "Navigator: obstacle status cleared but decision still pending; ignoring.";
            return;
          }
          if (waiting_for_reroute_) {
            CLOG(INFO, "mission.state_machine") << "Navigator: obstacle cleared while reroute pending; holding reroute latch.";
            obstacle_active_ = true;
            pending_clear_resume_ = false;
            pending_reroute_resume_ = true;
            return;
          }
          bool has_pending_decision = false;
          {
            LockGuard lock(chatgpt_mutex_);
            has_pending_decision = !latest_obstacle_decision_.empty();
          }
          if (has_pending_decision && !decision_accepting_) {
            CLOG(INFO, "mission.state_machine") << "Navigator: obstacle status flickered low while decision pending; ignoring.";
            return;
          }
          obstacle_active_ = false;
          // If this episode ended after a WAIT decision, measure the actual wait time and
          // update file-based obstacle priors (per-type) if enabled.
          if (wait_episode_start_sec_ >= 0.0) {
            const double now_sec = node_->get_clock()->now().seconds();
            const double actual_wait_sec = now_sec - wait_episode_start_sec_;
            updateObstaclePriorFromWaitEpisode(wait_episode_type_, actual_wait_sec);
          }
          wait_episode_start_sec_ = -1.0;
          active_decision_.clear();
          waiting_for_decision_logged_ = false;
          waiting_for_decision_ = false;
          processing_obstacle_ = false;
          waiting_for_reroute_ = false;

          // Episode cleanup:
          // - Dijkstra mode: keep reroute mode enabled, but clear the last episode's penalties
          //   so the cleared obstacle does not keep affecting future plans.
          // - TDSP mode: clear per-episode static (huge) screening delays; only remembered blockage intervals persist.
          try {
            if (auto bfs_ptr = std::dynamic_pointer_cast<vtr::route_planning::BFSPlanner>(route_planner_)) {
              CLOG(INFO, "mission.state_machine")
                  << "HSHMAT: Episode cleanup (Dijkstra): clearing episode penalties (extra delays, timecost).";
              bfs_ptr->clearExtraEdgeCosts();
              bfs_ptr->setUseTimeCost(false);
            } else if (auto tdsp = std::dynamic_pointer_cast<vtr::route_planning::TDSPPlanner>(route_planner_)) {
              CLOG(INFO, "mission.state_machine")
                  << "HSHMAT: Episode cleanup (TDSP): clearing static screening delays; keeping remembered blockage intervals.";
              tdsp->setStaticEdgeDelays({});
            }
          } catch (...) {
            // Best-effort; do not disrupt episode transitions if a dynamic cast fails.
          }

          if (robot_paused_) {
            if (decision_accepting_) {
              CLOG(INFO, "mission.state_machine") << "Navigator: obstacle cleared and decision node ready. Resuming navigation.";
              setRobotPaused(false);
            } else {
              pending_clear_resume_ = true;
              CLOG(INFO, "mission.state_machine") << "Navigator: obstacle cleared but decision node still announcing. Waiting for release.";
            }
          } else {
            pending_clear_resume_ = false;
          }
          pending_reroute_resume_ = false;
          return;
        }
        
        if (!obstacle_active_ && !waiting_for_reroute_) {
          // Let the decision node own the notion of "episodes".
          // If it is still announcing / processing a previous obstacle
          // (decision_accepting_ == false), ignore new obstacle_status
          // signals here to avoid triggering multiple reroutes inside
          // a single episode.
          if (!decision_accepting_) {
            CLOG(DEBUG, "mission.state_machine")
                << "Navigator: /vtr/obstacle_status=true but decision node "
                << "is not accepting new decisions; ignoring this obstacle signal.";
            return;
          }
          
          obstacle_active_ = true;
          obstacle_start_time_ = node_->get_clock()->now();
          active_decision_.clear();
          wait_episode_type_ = "unknown";
          wait_episode_start_sec_ = -1.0;
          processing_obstacle_ = false;
          waiting_for_decision_logged_ = false;
          waiting_for_decision_ = true;
          // Starting a new obstacle episode; ensure previous reroute wait is cleared
          waiting_for_reroute_ = false;
          if (use_chatgpt_) {
            LockGuard lock(chatgpt_mutex_);
            if (decision_accepting_) {
              latest_obstacle_decision_.clear();
              last_decision_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
            }
            
            // Only pause robot if we need to wait for ChatGPT/human decision
            // In operator mode (use_chatgpt_=false), we use default_decision immediately, so no pause needed
            CLOG(INFO, "mission.state_machine") << "HSHMAT: Publishing ZERO velocity to stop robot while waiting for human/ChatGPT decision";
            setRobotPaused(true);  // Mark robot as paused while asking
          }
          
          CLOG(INFO, "mission.state_machine") << "HSHMAT-TTS: ============================================================";
          CLOG(INFO, "mission.state_machine") << "HSHMAT-TTS: Robot says: 'Obstacle detected.'";
          CLOG(INFO, "mission.state_machine") << "HSHMAT-TTS: ============================================================";
        }

        // In ChatGPT-enabled mode, the actual decision ("wait" vs "reroute")
        // and the call to triggerReroute() are handled exclusively by the
        // /vtr/chatgpt_decision subscriber. Here we only manage the high-level
        // obstacle episode bookkeeping and pausing the robot. This avoids a
        // second reroute being triggered from a cached decision when
        // /vtr/obstacle_status flickers near the reroute join point.
        if (use_chatgpt_) {
          return;
        }

        std::string decision_snapshot;
        rclcpp::Time decision_time_snapshot;
        {
          LockGuard lock(chatgpt_mutex_);
          decision_snapshot = latest_obstacle_decision_;
          decision_time_snapshot = last_decision_time_;
        }

        CLOG(INFO, "mission.state_machine") << "HSHMAT: Step 1 - Obstacle detected, received /vtr/obstacle_status=true";
        CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Latest decision = '" << (decision_snapshot.empty() ? "NONE" : decision_snapshot) << "'";

        if (processing_obstacle_) {
          // active_decision_ reflects the last applied decision for this obstacle episode.
          CLOG(DEBUG, "mission.state_machine")
              << "HSHMAT: Already processing obstacle detection (decision='"
              << (active_decision_.empty() ? "unknown" : active_decision_)
              << "'), ignoring duplicate signal";
          return;
        }

        if (waiting_for_reroute_) {
          CLOG(DEBUG, "mission.state_machine") << "HSHMAT: Already processing reroute, ignoring duplicate obstacle signal";
          return;
        }
        
        // Check if robot is paused AND we don't have a fresh decision yet
        // If we have a fresh decision, we need to process it even if paused
        if (robot_paused_ && use_chatgpt_) {
          double time_diff = (decision_time_snapshot - obstacle_start_time_).seconds();
          bool has_fresh_decision = !decision_snapshot.empty() && time_diff >= -0.1;
          if (!has_fresh_decision) {
            CLOG(DEBUG, "mission.state_machine") << "HSHMAT: Robot paused, waiting for decision (time_diff=" << time_diff << "s), ignoring duplicate obstacle signal";
            return;
          }
          CLOG(INFO, "mission.state_machine") << "HSHMAT: Robot paused but fresh decision available, proceeding to process it";
        }

        const bool reroute_enable = route_cfg_.enable_reroute;
        CLOG(INFO, "mission.state_machine") << "HSHMAT: Step 2 - Checking reroute config: enable_reroute=" << (reroute_enable ? "true" : "false");
        if (!reroute_enable) {
          CLOG(INFO, "mission.state_machine") << "HSHMAT: Rerouting disabled, ignoring obstacle";
          return;
        }

        if (!use_chatgpt_) {
          decision_snapshot = default_decision_;
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: ChatGPT disabled, defaulting to '" << decision_snapshot << "'";
        } else {
          // Wait for a fresh decision that came AFTER this specific obstacle was first detected
          // Use small tolerance (0.1s) to account for timing precision
          double time_diff = (decision_time_snapshot - obstacle_start_time_).seconds();
          if (decision_snapshot.empty() || time_diff < -0.1) {
            if (!waiting_for_decision_logged_) {
              CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Waiting for ChatGPT/human decision before taking action (time_diff=" 
                   << time_diff << "s)";
              waiting_for_decision_logged_ = true;
            }
            return;
          }
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Decision is fresh enough (time_diff=" << time_diff << "s), proceeding";
        }

        std::string decision = decision_snapshot;
        std::transform(decision.begin(), decision.end(), decision.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        processing_obstacle_ = true;
        waiting_for_decision_logged_ = false;

        CLOG(INFO, "mission.state_machine") << "⏱ TIMING: Decision is '" << decision << "', checking mode...";

        if (decision == "wait") {
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Decision is WAIT - Pausing robot instead of replanning";
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Publishing ZERO velocity to stop robot";

          setRobotPaused(true);
          active_decision_ = "wait";
          wait_episode_type_ = last_obstacle_type_;
          wait_episode_start_sec_ = node_->get_clock()->now().seconds();
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Robot stopped. Will wait for obstacle to clear (obstacle_status=false)";
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Not triggering replanning - obstacle may move on its own";
          return;
        } else if (decision == "reroute") {
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Decision is REROUTE - Proceeding with rerouting logic";
          active_decision_ = "reroute";
          wait_episode_start_sec_ = -1.0;
          triggerReroute();
          return;
        } else {
          CLOG(WARNING, "mission.state_machine") << "HSHMAT-ChatGPT: No valid decision received (got '" << decision << "'), defaulting to REROUTE";
          active_decision_ = "reroute";
          wait_episode_start_sec_ = -1.0;
          triggerReroute();
          return;
        }
      });

  // Hshmat: Track following route ids for mapping to replanner edges.
  following_route_sub_ = node_->create_subscription<vtr_navigation_msgs::msg::GraphRoute>(
      "following_route", rclcpp::QoS(10),
      [this](const vtr_navigation_msgs::msg::GraphRoute::SharedPtr route) {
        if (!route) return;

        // Check if route changed (replanning occurred)
        bool route_changed = false;
        if (route->ids.size() != following_route_ids_.size()) {
          route_changed = true;
        } else {
          for (size_t i = 0; i < route->ids.size(); ++i) {
            if (route->ids[i] != following_route_ids_[i]) {
              route_changed = true;
              break;
            }
          }
        }

        // Log if replanning completed
        if (route_changed && waiting_for_reroute_ && !route->ids.empty()) {
          waiting_for_reroute_ = false;
          if (decision_accepting_) {
            if (robot_paused_) {
              CLOG(INFO, "mission.state_machine")
                  << "Navigator: reroute committed with new route. Resuming navigation.";
              setRobotPaused(false);
            }
            pending_reroute_resume_ = false;
          } else {
            pending_reroute_resume_ = true;
            CLOG(INFO, "mission.state_machine")
                << "Navigator: reroute committed but decision node still speaking. Waiting for release.";
          }
        } else if (!route_changed && waiting_for_reroute_ && !route->ids.empty()) {
          waiting_for_reroute_ = false;
          if (!obstacle_active_) {
            if (decision_accepting_) {
              if (robot_paused_) {
                CLOG(INFO, "mission.state_machine")
                    << "Navigator: route unchanged but obstacle cleared - resuming original path.";
                setRobotPaused(false);
              }
              pending_reroute_resume_ = false;
            } else {
              pending_reroute_resume_ = true;
              CLOG(INFO, "mission.state_machine")
                  << "Navigator: route unchanged, waiting for decision node release.";
            }
          } else {
            CLOG(INFO, "mission.state_machine")
                << "Navigator: route unchanged and obstacle still active - remaining paused.";
            pending_reroute_resume_ = false;
          }
        }

        following_route_ids_.assign(route->ids.begin(), route->ids.end());
      });

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

  // Hshmat: Obstacle type (person/chair/...) from decision node.
  obstacle_type_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/vtr/obstacle_type", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg) return;
        last_obstacle_type_ = msg->data;
      },
      sub_opt);

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
  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, lidar_qos, std::bind(&Navigator::lidarCallback, this, std::placeholders::_1), sub_opt);
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
  radar_sub_ = node_->create_subscription<navtech_msgs::msg::RadarBScanMsg>(radar_topic, radar_qos, std::bind(&Navigator::radarCallback, this, std::placeholders::_1), sub_opt);
}
#endif

  // Subscribe to the imu topic 
  auto gyro_qos = rclcpp::QoS(100);
  gyro_qos.reliable();
  const auto gyro_topic = node_->declare_parameter<std::string>("gyro_topic", "/ouster/imu");
  gyro_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(gyro_topic, gyro_qos, std::bind(&Navigator::gyroCallback, this, std::placeholders::_1), sub_opt);


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

double Navigator::estimateObstacleExtentFromGrid() const {
  constexpr double kDefaultExtent = 0.5;
  if (last_obstacle_grid_.data.empty() || last_obstacle_grid_.info.resolution <= 0.0)
    return kDefaultExtent;

  const auto &info = last_obstacle_grid_.info;
  const double res = info.resolution;
  const double origin_x = info.origin.position.x;
  bool found = false;
  double min_x = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();

  for (uint32_t y = 0; y < info.height; ++y) {
    for (uint32_t x = 0; x < info.width; ++x) {
      const int idx = y * info.width + x;
      if (idx < 0 || static_cast<size_t>(idx) >= last_obstacle_grid_.data.size())
        continue;
      if (last_obstacle_grid_.data[idx] != 100) continue;  // only on-path cells
      const double cell_x = origin_x + (static_cast<double>(x) + 0.5) * res;
      min_x = std::min(min_x, cell_x);
      max_x = std::max(max_x, cell_x);
      found = true;
    }
  }

  if (!found) return kDefaultExtent;

  const double extent = std::max(res, max_x - min_x + res);
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

void Navigator::triggerReroute() {
  if (waiting_for_reroute_) {
    CLOG(DEBUG, "mission.state_machine")
        << "HSHMAT: triggerReroute called but reroute already pending; ignoring.";
    return;
  }

  const bool reroute_enable = route_cfg_.enable_reroute;
  CLOG(INFO, "mission.state_machine")
      << "HSHMAT: Step 2 - Checking reroute config: enable_reroute="
      << (reroute_enable ? "true" : "false");
  if (!reroute_enable) {
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Rerouting disabled, ignoring reroute decision";
    return;
  }

  // Only respond during Repeat.* modes (Follow, Plan, MetricLoc, etc.).
  try {
    const std::string curr = state_machine_->name();
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Step 3 - Checking state mode: " << curr;
    const bool in_repeat = curr.find("Repeat") != std::string::npos;
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: State analysis - in_repeat="
        << (in_repeat ? "true" : "false");
    if (!in_repeat) {
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: Not in Repeat mode, ignoring reroute decision";
      return;
    }
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: In Repeat mode (" << curr
        << "), proceeding with rerouting";
  } catch (...) {
    return;
  }

  // At this point we are committed to attempting a reroute.
  waiting_for_reroute_ = true;
  pending_reroute_resume_ = true;

  // Determine edges affected by the obstacle range along the path.
  std::vector<vtr::tactic::EdgeId> affected_edges;
  try {
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Step 4b - Evaluating edges along current route for obstacle overlap";
    const auto loc = tactic_->getPersistentLoc();
    const auto current_vid = loc.v;
    int idx = -1;
    for (size_t i = 0; i < following_route_ids_.size(); ++i) {
      if (vtr::tactic::VertexId(following_route_ids_[i]) == current_vid) {
        idx = static_cast<int>(i);
        break;
      }
    }
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Found current vertex at route index: " << idx;

    if (idx >= 0) {
      const double obstacle_extent_m = estimateObstacleExtentFromGrid();
      const double obstacle_start = last_obstacle_distance_;
      const double obstacle_end = obstacle_start + obstacle_extent_m;
      double accumulated_distance = 0.0;

      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: Obstacle detected at " << last_obstacle_distance_
          << " m ahead, banning edges in range [" << obstacle_start << " m, "
          << obstacle_end << " m] (extent from grid: " << obstacle_extent_m
          << " m)";

      for (int i = idx; i + 1 < static_cast<int>(following_route_ids_.size());
           ++i) {
        vtr::tactic::VertexId v1(following_route_ids_[i]);
        vtr::tactic::VertexId v2(following_route_ids_[i + 1]);

        double edge_length = 0.0;
        try {
          auto edge_ptr = graph_->at(vtr::tactic::EdgeId(v1, v2));
          if (edge_ptr) {
            edge_length = edge_ptr->T().r_ab_inb().norm();
            double edge_start = accumulated_distance;
            double edge_end = accumulated_distance + edge_length;
            accumulated_distance = edge_end;

            bool edge_in_obstacle_range =
                (edge_end > obstacle_start && edge_start < obstacle_end);

            if (edge_in_obstacle_range) {
              affected_edges.emplace_back(v1, v2);
              CLOG(INFO, "mission.state_machine")
                  << "HSHMAT: Edge " << v1 << " -> " << v2
                  << " overlaps obstacle range (edge range: [" << edge_start
                  << " m, " << edge_end << " m])";
            } else {
              CLOG(DEBUG, "mission.state_machine")
                  << "HSHMAT: Skipping edge " << v1 << " -> " << v2
                  << " (edge range: [" << edge_start << " m, " << edge_end
                  << " m])";
            }

            if (edge_start >= obstacle_end) {
              CLOG(INFO, "mission.state_machine")
                  << "HSHMAT: Passed obstacle range, stopping edge search";
              break;
            }
          } else {
            CLOG(WARNING, "mission.state_machine")
                << "HSHMAT: Edge " << v1 << " -> " << v2 << " not found in graph";
          }
        } catch (const std::exception &e) {
          CLOG(WARNING, "mission.state_machine")
              << "HSHMAT: Failed to query edge " << v1 << " -> " << v2
              << ": " << e.what();
        }
      }
      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: Total edges affected: " << affected_edges.size();

    } else {
      CLOG(WARNING, "mission.state_machine")
          << "HSHMAT: Could not find current vertex in following route";
    }
  } catch (const std::exception &e) {
    CLOG(WARNING, "navigation")
        << "Failed to evaluate obstacle edges: " << e.what();
  }

  // Get trunk vertex for coordinate transforms
  vtr::tactic::VertexId trunk_vid;
  try {
    const auto loc = tactic_->getPersistentLoc();
    trunk_vid = loc.v;
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Trunk vertex for TF: " << trunk_vid;
  } catch (const std::exception &e) {
    CLOG(WARNING, "mission.state_machine")
        << "HSHMAT: Could not get trunk vertex: " << e.what();
  }

  // Log grid info for debugging
  if (!last_obstacle_grid_.data.empty()) {
    const auto &info = last_obstacle_grid_.info;
    const std::string grid_frame = last_obstacle_grid_.header.frame_id;
    CLOG(INFO, "mission.state_machine")
        << "HSHMAT: Obstacle grid info - frame=" << grid_frame
        << ", size=" << info.width << "x" << info.height
        << ", resolution=" << info.resolution << "m"
        << ", origin=(" << info.origin.position.x << ", " << info.origin.position.y << ")";
    
    // Log if TF is available for trunk -> grid_frame transform
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

  // Configure the planner based on the new config.
  //
  // - Dijkstra mode: preserves the old "deterministic_timecost" behavior by
  //   assigning a fixed per-edge delay (seconds) on affected edges.
  // - TDSP mode: assigns blockage *intervals* (start,end) on affected edges and
  //   runs FIFO label-setting TDSP, so the planner can reason about clearing
  //   (e.g., arriving after end time).
  //
  // Route screening behavior:
  // - We always mark the currently-followed (active) edges in the obstacle range
  //   as "blocked" (Dijkstra: active_delay_sec; TDSP: blockage interval).
  // - We also scan other edges that intersect ORANGE/RED cells in the local
  //   obstacle grid and assign them a huge static delay (1e6s). This is the
  //   "route screening" mechanism that prevents rerouting onto other routes
  //   that are also obstructed in the same local window.
  try {
        const double huge_delay_sec = 1e6;  // effectively "blocked"
    const double dur_sec = expectedObstacleDurationSeconds();

    // Prune expired blockages (best-effort).
    const double now_sec = node_->get_clock()->now().seconds();
    for (auto it = edge_blockages_.begin(); it != edge_blockages_.end();) {
      if (it->second.end_sec <= now_sec) it = edge_blockages_.erase(it);
      else ++it;
    }

    if (auto tdsp = std::dynamic_pointer_cast<vtr::route_planning::TDSPPlanner>(route_planner_)) {
      // Before applying new obstacle windows, opportunistically verify existing
      // remembered blockages in the local grid window (within a radius). If an
      // edge is predicted blocked but the grid suggests it is now free, erase it.
      const double radius_m = route_cfg_.verify_blockage_lookahead_m;
      if (radius_m > 0.0) {
        const size_t removed = pruneRememberedBlockagesUsingGrid(
            edge_blockages_, graph_, graph_map_server_, last_obstacle_grid_, tf_buffer_, trunk_vid,
            now_sec, radius_m);
        if (removed > 0) {
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: TDSP blockage verification removed " << removed
              << " remembered blockage(s) as 'free now' within radius "
              << radius_m << "m";
        }
      }

      // TDSP: static delays come only from non-active edges via grid checks.
      // Active/affected edges have *no* static delay; they are handled as time windows.
      auto static_delays = buildObstacleEdgeDelays(
          graph_, graph_map_server_, last_obstacle_grid_, affected_edges,
          /*active_delay_sec=*/0.0, huge_delay_sec, tf_buffer_, trunk_vid,
          /*screening_radius_m=*/route_cfg_.screening_lookahead_m);
      tdsp->setStaticEdgeDelays(static_delays);
      tdsp->setNominalSpeed(route_cfg_.nominal_speed_mps);

      // Newest-overrides: overwrite the blockage interval for each affected edge.
      for (const auto &e : affected_edges) {
        edge_blockages_[e] = EdgeBlockageInterval{now_sec, now_sec + dur_sec};
      }

      // Convert blockage map into TDSPPlanner type.
      std::unordered_map<vtr::tactic::EdgeId, vtr::route_planning::TDSPPlanner::BlockageInterval> b;
      b.reserve(edge_blockages_.size());
      for (const auto &kv : edge_blockages_) {
        b.emplace(kv.first, vtr::route_planning::TDSPPlanner::BlockageInterval{kv.second.start_sec, kv.second.end_sec});
      }
      tdsp->setEdgeBlockages(b);

      CLOG(INFO, "mission.state_machine")
          << "HSHMAT: TDSP replanner configured: affected_edges="
          << affected_edges.size() << ", static_delays=" << static_delays.size()
          << ", duration=" << dur_sec << "s, remembered_blockages=" << b.size();

      // Detailed TDSP logging (mission.state_machine): which edges are blocked and how much time remains.
      if (!affected_edges.empty()) {
        std::stringstream ss;
        ss << "HSHMAT: TDSP blocked edges (newest overwrite, remaining_sec): ";
        for (const auto &e : affected_edges) {
          double rem = 0.0;
          auto it = edge_blockages_.find(e);
          if (it != edge_blockages_.end()) {
            rem = std::max(0.0, it->second.end_sec - now_sec);
          }
          ss << e << "(rem=" << rem << "s) ";
        }
        CLOG(INFO, "mission.state_machine") << ss.str();
      }
      if (!static_delays.empty()) {
        std::stringstream ss_huge;
        size_t huge_cnt = 0;
        ss_huge << "HSHMAT: TDSP route-screening edges with huge_delay=" << huge_delay_sec << "s: ";
        for (const auto &kv : static_delays) {
          if (kv.second == huge_delay_sec) {
            ++huge_cnt;
            ss_huge << kv.first << " ";
          }
        }
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: TDSP route-screening: huge_edges=" << huge_cnt
            << " (static_delays total=" << static_delays.size() << ")";
        if (huge_cnt > 0) CLOG(INFO, "mission.state_machine") << ss_huge.str();
      }
    } else if (auto bfs_ptr = std::dynamic_pointer_cast<vtr::route_planning::BFSPlanner>(route_planner_)) {
      // Dijkstra/timecost: preserve deterministic_timecost behavior.
        auto delays = buildObstacleEdgeDelays(
            graph_, graph_map_server_, last_obstacle_grid_, affected_edges,
          /*active_delay_sec=*/dur_sec, huge_delay_sec, tf_buffer_, trunk_vid,
          /*screening_radius_m=*/route_cfg_.screening_lookahead_m);
        bfs_ptr->setExtraEdgeCosts(delays);
        bfs_ptr->setUseTimeCost(true);
        bfs_ptr->setUseMasked(true);
      bfs_ptr->setNominalSpeed(route_cfg_.nominal_speed_mps);
        CLOG(INFO, "mission.state_machine")
          << "HSHMAT: Dijkstra(timecost) replanner configured with "
          << delays.size() << " delayed edges (active_delay=" << dur_sec
            << "s, huge_delay=" << huge_delay_sec << "s).";

      // Detailed Dijkstra logging (mission.state_machine): preserve old deterministic_timecost edge lists.
        if (!delays.empty()) {
        std::stringstream ss_active, ss_huge;
        size_t active_cnt = 0, huge_cnt = 0;
        ss_active << "HSHMAT: Dijkstra edges with active_delay=" << dur_sec << "s: ";
        ss_huge << "HSHMAT: Dijkstra edges with huge_delay=" << huge_delay_sec << "s: ";
          for (const auto &kv : delays) {
          if (kv.second == huge_delay_sec) {
            ++huge_cnt;
            ss_huge << kv.first << " ";
          } else if (kv.second > 0.0) {
            ++active_cnt;
            ss_active << kv.first << " ";
          }
        }
          CLOG(INFO, "mission.state_machine")
            << "HSHMAT: Dijkstra delayed edges summary: active=" << active_cnt
            << ", huge=" << huge_cnt << ", total=" << delays.size();
        if (active_cnt > 0) CLOG(INFO, "mission.state_machine") << ss_active.str();
        if (huge_cnt > 0) CLOG(INFO, "mission.state_machine") << ss_huge.str();
      }
    } else {
      CLOG(WARNING, "mission.state_machine")
          << "HSHMAT: Unknown route planner type; cannot apply reroute costs.";
    }
  } catch (const std::exception &e) {
    CLOG(WARNING, "navigation")
        << "Failed to configure replanner in triggerReroute: " << e.what();
  }

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

