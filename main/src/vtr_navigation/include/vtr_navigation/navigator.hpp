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
 * \brief Navigator class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <filesystem>
#include <queue>

#include <tf2_ros/transform_listener.h>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_common/timing/time_utils.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_lgmath_extensions/conversions.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_mission_planning/ros_mission_server.hpp>
#include <vtr_navigation/map_projector.hpp>
#include <vtr_path_planning/simple_planner.hpp>
#include <vtr_path_tracker/base.hpp>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_base.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/pipelines/pipeline_factory.hpp>
#include <vtr_tactic/publisher_interface.hpp>
#include <vtr_tactic/tactic.hpp>
#include <vtr_tactic/types.hpp>

#include <std_msgs/msg/bool.hpp>
#include <vtr_messages/msg/graph_path.hpp>
#include <vtr_messages/msg/robot_status.hpp>
#include <vtr_messages/msg/time_stamp.hpp>

#ifdef VTR_ENABLE_LIDAR
#include <vtr_lidar/pipeline.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

#ifdef VTR_ENABLE_CAMERA
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/pipeline.hpp>

#include <vtr_messages/msg/rig_image_calib.hpp>
#endif

namespace vtr {
namespace navigation {

using PathTrackerMsg = std_msgs::msg::UInt8;
using TimeStampMsg = vtr_messages::msg::TimeStamp;
using PathMsg = vtr_messages::msg::GraphPath;
using RobotStatusMsg = vtr_messages::msg::RobotStatus;
using ResultMsg = std_msgs::msg::Bool;
using ExampleDataMsg = std_msgs::msg::Bool;

using namespace vtr::tactic;
using namespace vtr::pose_graph;

class Navigator : public PublisherInterface {
 public:
  Navigator(const rclcpp::Node::SharedPtr node);
  ~Navigator();

  /// Publisher interface required
  /** \brief Sets the path followed by the robot for UI update */
  void publishPath(const LocalizationChain &chain) const override;
  /** \brief Clears the path followed by the robot for UI update */
  void clearPath() const override;
  /** \brief Updates robot messages for UI */
  void publishRobot(
      const Localization &persistentLoc, uint64_t pathSeq = 0,
      const Localization &targetLoc = Localization(),
      const std::shared_ptr<rclcpp::Time> stamp = nullptr) const override;

  /// Expose internal blocks for testing and debugging
  const Tactic::Ptr tactic() const { return tactic_; }
  const RCGraph::Ptr graph() const { return graph_; }
  const state::StateMachine::Ptr sm() const { return state_machine_; }
  const std::string &robot_frame() const { return robot_frame_; }
#ifdef VTR_ENABLE_LIDAR
  const std::string &lidar_frame() const { return lidar_frame_; }
  const lgmath::se3::TransformationWithCovariance &T_lidar_robot() const {
    return T_lidar_robot_;
  }
#endif
#ifdef VTR_ENABLE_CAMERA
  const std::string &camera_frame() const { return camera_frame_; }
  const lgmath::se3::TransformationWithCovariance &T_camera_robot() const {
    return T_camera_robot_;
  }
#endif

 private:
  void process();

  /// Sensor specific stuff
  void exampleDataCallback(const ExampleDataMsg::SharedPtr);
#ifdef VTR_ENABLE_LIDAR
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
#endif
#ifdef VTR_ENABLE_CAMERA
  void imageCallback(const vtr_messages::msg::RigImageCalib::SharedPtr msg);
#endif

  /** \brief ROS-handle for communication */
  const rclcpp::Node::SharedPtr node_;

  /// VTR building blocks
  state::StateMachine::Ptr state_machine_;
  RCGraph::Ptr graph_;
  Tactic::Ptr tactic_;
  RosMissionServer::UniquePtr mission_server_;
  path_planning::PlanningInterface::Ptr route_planner_;
  MapProjector::Ptr map_projector_;

  /// Publisher interface
  /** \brief Publisher to send the path tracker new following paths. */
  rclcpp::Publisher<PathMsg>::SharedPtr following_path_publisher_;
  rclcpp::Publisher<RobotStatusMsg>::SharedPtr robot_publisher_;

  /// Internal thread handle
  /** \brief a flag to let the process() thread know when to quit */
  std::atomic<bool> quit_ = false;
  /** \brief the queue processing thread */
  std::thread process_thread_;
  /** \brief a notifier for when jobs are on the queue */
  std::condition_variable process_;

  ///
  /** \brief the data queue */
  std::queue<QueryCache::Ptr> queue_;
  /** \brief a lock to coordinate adding/removing jobs from the queue */
  std::mutex queue_mutex_;

  /// robot and sensor specific stuff
  // robot
  std::string robot_frame_;
  // example data
  rclcpp::Subscription<ExampleDataMsg>::SharedPtr example_data_sub_;
#ifdef VTR_ENABLE_LIDAR
  std::string lidar_frame_;
  /** \brief Lidar data subscriber */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  std::atomic<bool> pointcloud_in_queue_ = false;
  lgmath::se3::TransformationWithCovariance T_lidar_robot_;
#endif
#ifdef VTR_ENABLE_CAMERA
  std::string camera_frame_;
  /** \brief camera camera data subscriber */
  rclcpp::Subscription<vtr_messages::msg::RigImageCalib>::SharedPtr image_sub_;
  std::atomic<bool> image_in_queue_ = false;
  lgmath::se3::TransformationWithCovariance T_camera_robot_;
#endif

  /** \brief Pipeline running result publisher */
  rclcpp::Publisher<ResultMsg>::SharedPtr result_pub_;
};

}  // namespace navigation
}  // namespace vtr
