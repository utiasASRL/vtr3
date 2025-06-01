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

  CLOG(DEBUG, "navigation") << "Received a lidar pointcloud with stamp " << timestamp;

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<lidar::LidarQueryCache>();

  LockGuard lock(mutex_);

  /// Discard old frames if our queue is too big
  if (queue_.size() > max_queue_size_) {
    CLOG(WARNING, "navigation")
        << "Dropping pointcloud message " << *queue_.front()->stamp << " because the queue is full.";
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
