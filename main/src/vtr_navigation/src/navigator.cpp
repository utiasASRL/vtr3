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
#include <algorithm>
#include <cctype>
#include <sstream>

#include "vtr_common/utils/filesystem.hpp"
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

}  // namespace

Navigator::Navigator(const rclcpp::Node::SharedPtr& node) : node_(node) {
  el::Helpers::setThreadName("navigator");
  CLOG(INFO, "navigation") << "Starting VT&R3 system - hello!";

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
  route_planner_ = std::make_shared<BFSPlanner>(graph_);
  if (auto bfs_ptr = std::dynamic_pointer_cast<BFSPlanner>(route_planner_)) {
    const bool reroute_enable =
        node_->declare_parameter<bool>("route_planning.enable_reroute", false);
    const bool permanent_ban =
        node_->declare_parameter<bool>("route_planning.permanent_ban", false);
    replanner_type_ = node_->declare_parameter<std::string>(
        "route_planning.replanner_type", std::string("masked_bfs"));
    nominal_speed_mps_ = node_->declare_parameter<double>(
        "route_planning.nominal_speed_mps", 0.5);
    delay_person_seconds_ = node_->declare_parameter<double>(
        "route_planning.obstacle_delay_person", 5.0);
    delay_chair_seconds_ = node_->declare_parameter<double>(
        "route_planning.obstacle_delay_chair", 60.0);

    bfs_ptr->setUseMasked(reroute_enable);
    bfs_ptr->setPermanentBan(permanent_ban);
    bfs_ptr->setNominalSpeed(nominal_speed_mps_);

    CLOG(INFO, "navigation")
        << "Route planning config: enable_reroute=" << reroute_enable
        << ", permanent_ban=" << permanent_ban
        << ", replanner_type=" << replanner_type_
        << ", nominal_speed_mps=" << nominal_speed_mps_
        << ", delay_person_seconds=" << delay_person_seconds_
        << ", delay_chair_seconds=" << delay_chair_seconds_;
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
        if (decision == "reroute") {
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
          active_decision_.clear();
          waiting_for_decision_logged_ = false;
          waiting_for_decision_ = false;
          processing_obstacle_ = false;
          waiting_for_reroute_ = false;
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
          CLOG(DEBUG, "mission.state_machine") << "HSHMAT: Already processing obstacle detection (decision='" << active_decision_ << "'), ignoring duplicate signal";
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

        bool reroute_enable = false;
        try { reroute_enable = node_->get_parameter("route_planning.enable_reroute").get_value<bool>(); } catch (...) {}
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
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Robot stopped. Will wait for obstacle to clear (obstacle_status=false)";
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Not triggering replanning - obstacle may move on its own";
          return;
        } else if (decision == "reroute") {
          CLOG(INFO, "mission.state_machine") << "HSHMAT-ChatGPT: Decision is REROUTE - Proceeding with rerouting logic";
          triggerReroute();
          return;
        } else {
          CLOG(WARNING, "mission.state_machine") << "HSHMAT-ChatGPT: No valid decision received (got '" << decision << "'), defaulting to REROUTE";
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
  auto msg = tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot_.inverse().matrix()));
  msg.header.frame_id = "robot";
  msg.child_frame_id = "lidar";
  tf_sbc_->sendTransform(msg);
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
  msg.header.frame_id = "robot";
  msg.child_frame_id = "camera";
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

double Navigator::obstacleTypeDelaySeconds() const {
  const std::string type = last_obstacle_type_;
  if (type == "person" || type == "human") return delay_person_seconds_;
  if (type == "chair") return delay_chair_seconds_;
  // Default: treat unknown as static obstacle with larger delay.
  return delay_chair_seconds_;
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
void Navigator::triggerReroute() {
  if (waiting_for_reroute_) {
    CLOG(DEBUG, "mission.state_machine")
        << "HSHMAT: triggerReroute called but reroute already pending; ignoring.";
    return;
  }

  bool reroute_enable = false;
  try {
    reroute_enable =
        node_->get_parameter("route_planning.enable_reroute").get_value<bool>();
  } catch (...) {
  }
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
  active_decision_ = "reroute";
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

  // Configure the planner based on replanner_type_.
  try {
    auto bfs_ptr =
        std::dynamic_pointer_cast<vtr::route_planning::BFSPlanner>(
            route_planner_);
    if (bfs_ptr) {
      if (replanner_type_ == "deterministic_timecost") {
        // Time-based replanning: penalize affected edges instead of outright banning.
        std::unordered_map<vtr::tactic::EdgeId, double> delays;
        const double delay_sec = obstacleTypeDelaySeconds();
        for (const auto &e : affected_edges) {
          delays[e] = delay_sec;
        }
        // IMPORTANT:
        //  - Replace (not accumulate) extra edge costs for this obstacle episode.
        //    Each obstacle episode should have its own delays, not accumulate
        //    from previous episodes. This ensures we only penalize edges from
        //    the current obstacle, not stale delays from past obstacles.
        bfs_ptr->setExtraEdgeCosts(delays);
        bfs_ptr->setUseTimeCost(true);
        bfs_ptr->setUseMasked(true);
        CLOG(INFO, "mission.state_machine")
            << "HSHMAT: deterministic_timecost replanner configured with delay="
            << delay_sec << "s on " << delays.size() << " edges.";
        // Debug: log which edges are getting delays
        if (!delays.empty()) {
          std::stringstream ss;
          ss << "HSHMAT: Edges with delay " << delay_sec << "s: ";
          for (const auto &kv : delays) {
            ss << kv.first << " ";
          }
          CLOG(INFO, "mission.state_machine") << ss.str();
        }
      } else {
        // Legacy masked BFS: ban affected edges.
        if (!affected_edges.empty()) {
          bfs_ptr->addBannedEdges(affected_edges);
          bfs_ptr->setUseMasked(true);
          bfs_ptr->setUseTimeCost(false);
          bfs_ptr->clearExtraEdgeCosts();
          CLOG(INFO, "mission.state_machine")
              << "HSHMAT: masked_bfs replanner: banned " << affected_edges.size()
              << " edges (grid-based).";
        }
      }
    } else {
      CLOG(WARNING, "mission.state_machine")
          << "HSHMAT: Could not cast route planner to BFSPlanner";
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

