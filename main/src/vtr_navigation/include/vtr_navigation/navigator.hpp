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
#include "vtr_tactic/tactic.hpp"

#ifdef VTR_ENABLE_LIDAR
#include "sensor_msgs/msg/point_cloud2.hpp"
#endif

namespace vtr {
namespace navigation {

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

  /// robot and sensor specific stuff, these are set in constructor, so no need
  /// to lock
  rclcpp::CallbackGroup::SharedPtr callback_group_;
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
#ifdef VTR_ENABLE_LIDAR
  bool pointcloud_in_queue_ = false;
#endif

  /// VTR building blocks
  GraphMapServer::Ptr graph_map_server_;
  tactic::Graph::Ptr graph_;
  tactic::Tactic::Ptr tactic_;
  path_planning::PathPlannerInterface::Ptr path_planner_;
  route_planning::RoutePlannerInterface::Ptr route_planner_;
  ROSMissionServer::Ptr mission_server_;
  mission_planning::StateMachine::Ptr state_machine_;

  /// Threading
  bool stop_ = false;
  size_t thread_count_ = 0;
  /** \brief the event processing thread */
  std::thread process_thread_;
};

}  // namespace navigation
}  // namespace vtr
