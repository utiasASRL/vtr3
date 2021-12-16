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
#include "vtr_navigation_v2/navigator.hpp"

#include "vtr_common/utils/filesystem.hpp"

namespace vtr {
namespace navigation {

using namespace vtr::tactic;

namespace {

EdgeTransform loadTransform(const std::string& source_frame,
                            const std::string& target_frame) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf_buffer{clock};
  tf2_ros::TransformListener tf_listener{tf_buffer};
  if (tf_buffer.canTransform(source_frame, target_frame, tf2::TimePoint(),
                             tf2::durationFromSec(1))) {
    auto tf_source_target = tf_buffer.lookupTransform(
        source_frame, target_frame, tf2::TimePoint(), tf2::durationFromSec(1));
    tf2::Stamped<tf2::Transform> tf2_source_target;
    tf2::fromMsg(tf_source_target, tf2_source_target);
    EdgeTransform T_source_target(
        common::rosutils::fromStampedTransformation(tf2_source_target));
    T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
    CLOG(DEBUG, "navigator")
        << "Transform from " << target_frame << " to " << source_frame
        << " has been set to" << T_source_target;
    return T_source_target;
  }
  CLOG(WARNING, "navigator")
      << "Transform not found - source: " << source_frame
      << " target: " << target_frame << ". Default to identity.";
  EdgeTransform T_source_target(Eigen::Matrix4d(Eigen::Matrix4d::Identity()));
  T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
  return T_source_target;
}

}  // namespace

Navigator::Navigator(const rclcpp::Node::SharedPtr& node) : node_(node) {
  el::Helpers::setThreadName("navigator");
  CLOG(INFO, "navigator") << "Starting VT&R3 system - hello!";

  /// data storage directory (must have been set at this moment)
  auto data_dir = node_->get_parameter("data_dir").get_value<std::string>();
  data_dir = common::utils::expand_user(common::utils::expand_env(data_dir));
  CLOG(INFO, "navigator") << "Data directory set to: " << data_dir;
#if false
  /// graph map server (pose graph callback, tactic callback)
  graph_map_server_ = std::make_shared<GraphMapServer>();

  /// pose graph \todo set up callback
  auto new_graph = node_->declare_parameter<bool>("start_new_graph", false);
  graph_ = tactic::Graph::MakeShared(data_dir + "/graph", !new_graph);
  graph_map_server_->start(node_, graph_);
#endif
  /// robot and sensor transformation, subscription
  // clang-format off
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  // robot frame
  robot_frame_ = node_->declare_parameter<std::string>("robot_frame", "robot");
  // environment info
  const auto env_info_topic = node_->declare_parameter<std::string>("env_info_topic", "env_info");
  env_info_sub_ = node_->create_subscription<tactic::EnvInfo>(env_info_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::envInfoCallback, this, std::placeholders::_1), sub_opt);
#ifdef VTR_ENABLE_LIDAR
  lidar_frame_ = node_->declare_parameter<std::string>("lidar_frame", "lidar");
  T_lidar_robot_ = loadTransform(lidar_frame_, robot_frame_);
  // lidar pointcloud data subscription
  const auto lidar_topic = node_->declare_parameter<std::string>("lidar_topic", "/points");
  // \note lidar point cloud data frequency is low, and we cannot afford dropping data
  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::lidarCallback, this, std::placeholders::_1), sub_opt);
#endif
  // clang-format on

  ///
  thread_count_ = 1;
  process_thread_ = std::thread(&Navigator::process, this);
  CLOG(INFO, "navigator") << "VT&R3 initialization done!";
}

Navigator::~Navigator() {
  UniqueLock lock(mutex_);
  // send stop signal
  stop_ = true;
  cv_set_or_stop_.notify_all();
  //
  cv_thread_finish_.wait(lock, [this] { return thread_count_ == 0; });
  if (process_thread_.joinable()) process_thread_.join();

  /// manually destruct each building block in order
  graph_.reset();
  graph_map_server_.reset();

  CLOG(INFO, "navigator") << "VT&R3 destruction done! Bye-bye.";
}

void Navigator::process() {
  el::Helpers::setThreadName("navigator.sensor_input");
  CLOG(INFO, "navigator.sensor_input") << "Starting the sensor input thread.";
  while (true) {
    UniqueLock lock(mutex_);

    /// print a warning if our queue is getting too big
    if (queue_.size() > 5) {
      CLOG_EVERY_N(10, WARNING, "navigator.sensor_input")
          << " Input queue size is " << queue_.size();
    }

    cv_set_or_stop_.wait(lock, [this] { return stop_ || (!queue_.empty()); });
    if (stop_) {
      --thread_count_;
      CLOG(INFO, "navigator.sensor_input")
          << "Stopping the sensor input thread.";
      cv_thread_finish_.notify_all();
      return;
    }

    // get the front in queue
    auto qdata0 = queue_.front();
    queue_.pop();
#ifdef VTR_ENABLE_LIDAR
    if (const auto qdata =
            std::dynamic_pointer_cast<lidar::LidarQueryCache>(qdata0))
      if (qdata->pointcloud_msg.valid()) pointcloud_in_queue_ = false;
#endif

    // unlock the queue so that new data can be added
    lock.unlock();

    // execute the pipeline
    // tactic_->input(qdata0);  /// \todo

    // handle any transitions triggered by changes in localization status
    // state_machine_->handle(); /// \todo
  };
}

void Navigator::envInfoCallback(const tactic::EnvInfo::SharedPtr msg) {
  UniqueLock lock(mutex_);
  CLOG(DEBUG, "navigator") << "Received environment info update.";
  env_info_ = *msg;
}

#ifdef VTR_ENABLE_LIDAR
void Navigator::lidarCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  LockGuard lock(mutex_);
  CLOG(DEBUG, "navigator") << "Received a lidar pointcloud.";

  if (pointcloud_in_queue_) {
    CLOG_EVERY_N(10, INFO, "navigator")
        << "Skip pointcloud message because there is already "
           "one in queue.";
    return;
  }

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<lidar::LidarQueryCache>();

  // some modules require node for visualization
  query_data->node = node_;

  /// \todo (yuchen) need to distinguish this with stamp
  query_data->rcl_stamp.emplace(msg->header.stamp);
  storage::Timestamp timestamp =
      msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  query_data->stamp.emplace(timestamp);

  // add the current environment info
  query_data->env_info.emplace(env_info_);

  // put in the pointcloud msg pointer into query data
  query_data->pointcloud_msg = msg;

  // fill in the vehicle to sensor transform and frame names
  query_data->robot_frame.emplace(robot_frame_);
  query_data->lidar_frame.emplace(lidar_frame_);
  query_data->T_s_r.emplace(T_lidar_robot_);

  // add to the queue and notify the processing thread
  queue_.push(query_data);
  pointcloud_in_queue_ = true;
  cv_set_or_stop_.notify_one();
};
#endif

}  // namespace navigation
}  // namespace vtr
