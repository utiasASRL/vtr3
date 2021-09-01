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
 * \brief Navigator class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "rclcpp/rclcpp.hpp"

#include <vtr_navigation/navigator.hpp>

namespace vtr {
namespace navigation {

namespace {

EdgeTransform loadTransform(std::string source_frame,
                            std::string target_frame) {
  rclcpp::Clock::SharedPtr clock =
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf_buffer{clock};
  tf2_ros::TransformListener tf_listener{tf_buffer};
  if (tf_buffer.canTransform(source_frame, target_frame, tf2::TimePoint(),
                             tf2::durationFromSec(1))) {
    auto tf_source_target = tf_buffer.lookupTransform(
        source_frame, target_frame, tf2::TimePoint(), tf2::durationFromSec(1));
    tf2::Stamped<tf2::Transform> tf2_source_target;
    tf2::fromMsg(tf_source_target, tf2_source_target);
    lgmath::se3::TransformationWithCovariance T_source_target(
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

Navigator::Navigator(const rclcpp::Node::SharedPtr node) : node_(node) {
  el::Helpers::setThreadName("navigator");

  CLOG(INFO, "navigator") << "Starting the navigator node - Hello!";

  /// data storage directory (must have been set at this moment)
  auto data_dir = node_->get_parameter("data_dir").get_value<std::string>();
  data_dir = common::utils::expand_user(common::utils::expand_env(data_dir));
  CLOG(INFO, "navigator") << "Data directory set to: " << data_dir;

  /// publisher interfaces
  // clang-format off
  following_path_publisher_ = node_->create_publisher<PathMsg>("out/following_path", 1);
  robot_publisher_ = node_->create_publisher<RobotStatusMsg>("robot", 5);
  result_pub_ = node_->create_publisher<ResultMsg>("result", 1);  // callback on pipeline completion
  // clang-format on

  /// pose graph
  /// \todo yuchen may need to add an option to overwrite existing graph.
  graph_ = pose_graph::RCGraph::LoadOrCreate(data_dir + "/graph.index", 0);
  CLOG_IF(!graph_->numberOfVertices(), INFO, "navigator")
      << "Creating a new pose graph.";
  CLOG_IF(graph_->numberOfVertices(), INFO, "navigator")
      << "Loaded pose graph has " << graph_->numberOfVertices() << " vertices.";

  /// callbacks for pose graph updates (basically the pose graph publisher)
  map_projector_ = std::make_shared<MapProjector>(graph_, node_);
  map_projector_->setPublisher(this);
  graph_->setCallbackMode(map_projector_);

  /// route planner
  /// \todo currently only support distance based planner, timedelta planner not
  /// ported to vtr3 (check vtr2 code base).
  route_planner_ =
      std::make_shared<path_planning::SimplePlanner<pose_graph::RCGraph>>(
          graph_);

  /// path tracker
  /// \todo currently only support "robust_mpc_path_tracker"
  /// \todo create a path tracker factory in the path tracker package.
  auto path_tracker_ = path_tracker::PathTrackerMPC::Create(graph_, node_);

  /// pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node_);
#ifdef VTR_ENABLE_LIDAR
  pipeline_factory->add<lidar::LidarPipeline>();
#endif
#ifdef VTR_ENABLE_CAMERA
  pipeline_factory->add<vision::StereoPipeline>();
#endif
  auto pipeline = pipeline_factory->make("pipeline");

  /// tactic (state estimator)
  tactic_ = std::make_shared<Tactic>(Tactic::Config::fromROS(node_), node_,
                                     pipeline, graph_);
  tactic_->setPublisher(this);
  tactic_->setPipeline(mission_planning::PipelineMode::Idle);
  tactic_->setPathTracker(path_tracker_);
  if (graph_->contains(VertexId(0, 0))) tactic_->setTrunk(VertexId(0, 0));

  /// state machine
  state_machine_ = state::StateMachine::InitialState(tactic_.get());
  state_machine_->setPlanner(route_planner_);

  /// mission server
  mission_server_.reset(new RosMissionServer(node_, state_machine_));

  /// robot and sensor transformation, subscription
  // avoid early data in queue
  std::lock_guard<std::mutex> queue_lock(queue_mutex_);
  // clang-format off
  // robot frame
  robot_frame_ = node_->declare_parameter<std::string>("robot_frame", "base_link");
  // example data subscription, start with this to add new data subscription
  example_data_sub_ = node_->create_subscription<ExampleDataMsg>("/example_data", rclcpp::SensorDataQoS(), std::bind(&Navigator::exampleDataCallback, this, std::placeholders::_1));
#ifdef VTR_ENABLE_LIDAR
  lidar_frame_ = node_->declare_parameter<std::string>("lidar_frame", "velodyne");
  T_lidar_robot_ = loadTransform(lidar_frame_, robot_frame_);
  // lidar pointcloud data subscription
  const auto lidar_topic = node_->declare_parameter<std::string>("lidar_topic", "/points");
  // \note lidar point cloud data frequency is low, and we cannot afford dropping data
  lidar_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::lidarCallback, this, std::placeholders::_1));
#endif
#ifdef VTR_ENABLE_CAMERA
  camera_frame_ = node_->declare_parameter<std::string>("camera_frame", "front_xb3");
  T_camera_robot_ = loadTransform(camera_frame_, robot_frame_);
  const auto camera_topic = node_->declare_parameter<std::string>("camera_topic", "/xb3_images");
  const auto camera_calibration_topic = node_->declare_parameter<std::string>("camera_calibration_topic", "/xb3_calibration");
  image_sub_ = node_->create_subscription<vtr_messages::msg::RigImages>(camera_topic, rclcpp::SensorDataQoS(), std::bind(&Navigator::imageCallback, this, std::placeholders::_1));
  rig_calibration_client_ = node_->create_client<vtr_messages::srv::GetRigCalibration>(camera_calibration_topic);
#endif
  // clang-format on

  /// launch the processing thread
  process_thread_ = std::thread(&Navigator::process, this);

  CLOG(INFO, "navigator") << "Initialization done!";
}

Navigator::~Navigator() {
  // wait for the processing thread to join
  quit_ = true;

  if (process_thread_.joinable()) {
    process_.notify_one();
    process_thread_.join();
  }

  std::lock_guard<std::mutex> lck(queue_mutex_);

  route_planner_.reset();

  mission_server_->halt();
  mission_server_.reset();

  state_machine_.reset();
  // state estimation block
  tactic_.reset();
  // graph
  graph_->halt();
  map_projector_.reset();
  graph_.reset();

  CLOG(INFO, "navigator") << "Destruction done! Bye-bye.";
}

void Navigator::process() {
  el::Helpers::setThreadName("navigator.process");
  while (!quit_) {
    std::unique_lock<std::mutex> queue_lock(queue_mutex_);

    /// print a warning if our queue is getting too big
    if (queue_.size() > 5) {
      CLOG_EVERY_N(10, WARNING, "navigator")
          << "Cache queue size is " << queue_.size();
    }

    /// wait for the data to be added to the queues
    while (queue_.empty() && quit_ == false) {
      // wait for a data point
      process_.wait(queue_lock);
    }

    /// process data if there is one in queue
    if (queue_.empty()) continue;

    // get the front in queue
    auto qdata0 = queue_.front();
#ifdef VTR_ENABLE_LIDAR
    if (const auto qdata =
            std::dynamic_pointer_cast<lidar::LidarQueryCache>(qdata0))
      if (qdata->pointcloud_msg.is_valid()) pointcloud_in_queue_ = false;
#endif
#ifdef VTR_ENABLE_CAMERA
    if (const auto qdata =
            std::dynamic_pointer_cast<vision::CameraQueryCache>(qdata0))
      if (qdata->rig_images.is_valid()) image_in_queue_ = false;
#endif
    // pop the data off the front because we don't need them now
    queue_.pop();

    // unlock the queue so that new data can be added
    queue_lock.unlock();

    // execute the pipeline
    tactic_->runPipeline(qdata0);

    // handle any transitions triggered by changes in localization status
    state_machine_->handleEvents();

    // (temp) process completion callback
    result_pub_->publish(ResultMsg());
  };

  CLOG(INFO, "navigator") << "Data processing thread completed.";
}

void Navigator::exampleDataCallback(const ExampleDataMsg::SharedPtr) {
  CLOG(DEBUG, "navigator") << "Received an example sensor data.";

  /// Some necessary processing

  /// Add to the queue and notify the processing thread
  // queue_.push(query_data);
  // process_.notify_one();
}
#ifdef VTR_ENABLE_LIDAR
void Navigator::lidarCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  CLOG(DEBUG, "navigator") << "Received a lidar pointcloud.";

  if (pointcloud_in_queue_) {
    CLOG_EVERY_N(10, INFO, "navigator")
        << "Skip pointcloud message because there is already "
           "one in queue.";
    return;
  }

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<lidar::LidarQueryCache>();

  /// \todo (yuchen) need to distinguish this with stamp
  query_data->rcl_stamp.fallback(msg->header.stamp);

  // set time stamp
  TimeStampMsg stamp;
  stamp.nanoseconds_since_epoch =
      msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  query_data->stamp.fallback(stamp);

  // put in the pointcloud msg pointer into query data
  query_data->pointcloud_msg = msg;

  // fill in the vehicle to sensor transform and frame names
  query_data->robot_frame.fallback(robot_frame_);
  query_data->lidar_frame.fallback(lidar_frame_);
  query_data->T_s_r.fallback(T_lidar_robot_);

  // add to the queue and notify the processing thread
  queue_.push(query_data);
  pointcloud_in_queue_ = true;
  process_.notify_one();
};
#endif
#ifdef VTR_ENABLE_CAMERA
void Navigator::imageCallback(
    const vtr_messages::msg::RigImages::SharedPtr msg) {
  CLOG(DEBUG, "navigator") << "Received an stereo image.";

  if (image_in_queue_) {
    CLOG_EVERY_N(4, INFO, "navigator")
        << "Skip images message because there is already one in queue.";
    return;
  }

  if (!rig_calibration_) {
    fetchRigCalibration();
    CLOG(WARNING, "navigator") << "Dropping frame because no calibration data";
    return;
  }

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<vision::CameraQueryCache>();

  /// \todo (yuchen) need to distinguish this with stamp
  query_data->rcl_stamp.fallback(node_->now());

  // set time stamp
  query_data->stamp.fallback(msg->vtr_header.sensor_time_stamp);

  // add the rig names
  auto &rig_names = query_data->rig_names.fallback();
  rig_names->push_back(camera_frame_);
  msg->name = camera_frame_;  /// \todo (yuchen) should not be set here

  // fill in the images
  auto &images = query_data->rig_images.fallback();
  images->emplace_back(messages::copyImages(*msg));

  // fill in the calibration
  auto &calibration_list = query_data->rig_calibrations.fallback();
  calibration_list->push_back(*rig_calibration_);

  // fill in the vehicle to sensor transform and frame names
  query_data->robot_frame.fallback(robot_frame_);
  query_data->camera_frame.fallback(camera_frame_);
  query_data->T_sensor_vehicle.fallback(T_camera_robot_);

  // add to the queue and notify the processing thread
  queue_.push(query_data);
  image_in_queue_ = true;
  process_.notify_one();
}

void Navigator::fetchRigCalibration() {
  // wait for the service
  while (!rig_calibration_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      CLOG(ERROR, "navigator")
          << "Interrupted while waiting for the service. Exiting.";
      return;
    }
    CLOG(INFO, "navigator") << "Rig calibration not available, waiting again.";
  }

  // send and wait for the result
  auto request =
      std::make_shared<vtr_messages::srv::GetRigCalibration::Request>();
  auto response_callback =
      [this](rclcpp::Client<vtr_messages::srv::GetRigCalibration>::SharedFuture
                 future) {
        auto response = future.get();
        rig_calibration_ = std::make_shared<vtr::vision::RigCalibration>(
            messages::copyCalibration(response->calibration));
      };
  auto response =
      rig_calibration_client_->async_send_request(request, response_callback);
}
#endif
void Navigator::publishPath(const tactic::LocalizationChain &chain) const {
  CLOG(INFO, "navigator") << "Publishing path from: " << chain.trunkVertexId()
                          << " To: " << chain.endVertexID();

  PathMsg path_msg;
  path_msg.base_vertex_id = chain.trunkVertexId();
  for (auto it = chain.begin(); it != chain.end(); ++it)
    path_msg.vertex_id_list.push_back(it->v()->id());

  following_path_publisher_->publish(path_msg);
}

void Navigator::clearPath() const {
  // Publish an empty path message for the UI
  PathMsg path_msg;
  path_msg.base_vertex_id = uint64_t(-1);
  path_msg.vertex_id_list.clear();
  following_path_publisher_->publish(path_msg);
}

void Navigator::publishRobot(const Localization &persistent_loc,
                             uint64_t path_seq, const Localization &target_loc,
                             const std::shared_ptr<rclcpp::Time> stamp) const {
  if (!persistent_loc.v.isSet()) return;

  RobotStatusMsg msg;

  if (state_machine_) msg.state = state_machine_->name();
  msg.header.stamp = stamp == nullptr ? node_->now() : *stamp;

  msg.path_seq = path_seq;

  msg.trunk_vertex = persistent_loc.v;
  msg.t_leaf_trunk << persistent_loc.T;
  if (persistent_loc.T.covarianceSet()) {
    msg.cov_leaf_trunk.push_back(std::sqrt(persistent_loc.T.cov()(0, 0)));
    msg.cov_leaf_trunk.push_back(std::sqrt(persistent_loc.T.cov()(1, 1)));
    msg.cov_leaf_trunk.push_back(std::sqrt(persistent_loc.T.cov()(5, 5)));
  }

  if (target_loc.localized && target_loc.successes > 5) {
    msg.target_vertex = target_loc.v;
    msg.t_leaf_target << target_loc.T;
    if (target_loc.T.covarianceSet()) {
      msg.cov_leaf_target.push_back(std::sqrt(target_loc.T.cov()(0, 0)));
      msg.cov_leaf_target.push_back(std::sqrt(target_loc.T.cov()(1, 1)));
      msg.cov_leaf_target.push_back(std::sqrt(target_loc.T.cov()(5, 5)));
    }
  } else {
    msg.target_vertex = VertexId((uint64_t)-1);
  }

  /// Get the 2D projection of the persistent and target loc on map
  map_projector_->projectRobot(persistent_loc, target_loc, msg);

  // Publish the robot position
  robot_publisher_->publish(msg);
}

}  // namespace navigation
}  // namespace vtr
