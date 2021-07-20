#include "rclcpp/rclcpp.hpp"

#include <vtr_navigation/navigator.hpp>

// These utilities need to be moved somewhere else
namespace {

using namespace vtr;
using namespace vtr::navigation;

void copyPointcloud(const PointCloudMsg::SharedPtr msg,
                    std::vector<PointXYZ> &pts, std::vector<double> &ts) {
  size_t N = (size_t)(msg->width * msg->height);
  pts.reserve(N);
  ts.reserve(N);
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
       iter_y(*msg, "y"), iter_z(*msg, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // Add all points to the vector container
    pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
  }

  for (sensor_msgs::PointCloud2ConstIterator<double> iter(*msg, "t");
       iter != iter.end(); ++iter) {
    // Add all timestamps to the vector container
    ts.push_back(*iter);
  }
};

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
    LOG(DEBUG) << "Transform from " << target_frame << " to " << source_frame
               << " has been set to" << T_source_target;
    return T_source_target;
  }
  LOG(WARNING) << "Transform not found - source: " << source_frame
               << " target: " << target_frame << ". Default to identity.";
  EdgeTransform T_source_target(Eigen::Matrix4d(Eigen::Matrix4d::Identity()));
  T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
  return T_source_target;
}

}  // namespace

namespace vtr {
namespace navigation {

Navigator::Navigator(const rclcpp::Node::SharedPtr node) : node_(node) {
  /// data storage directory (must have been set at this moment)
  auto data_dir = node_->get_parameter("data_dir").get_value<std::string>();
  data_dir = common::utils::expand_user(common::utils::expand_env(data_dir));
  LOG(INFO) << "[Navigator] Data directory set to: " << data_dir;

  /// publisher interfaces
  following_path_publisher_ =
      node_->create_publisher<PathMsg>("out/following_path", 1);
  robot_publisher_ = node_->create_publisher<RobotStatusMsg>("robot", 5);
  // temporary callback on pipeline completion
  result_pub_ = node_->create_publisher<ResultMsg>("result", 1);

  /// pose graph
  /// \todo yuchen make need to add an option to overwrite existing graph.
  graph_ = pose_graph::RCGraph::LoadOrCreate(data_dir + "/graph.index", 0);
  LOG_IF(!graph_->numberOfVertices(), INFO)
      << "[Navigator] Creating a new pose graph.";
  LOG_IF(graph_->numberOfVertices(), INFO)
      << "[Navigator] Loaded pose graph has " << graph_->numberOfVertices()
      << " vertices.";

  /// callbacks for graph publishing/relaxation
  map_projector_ = std::make_shared<MapProjector>(graph_, node_);
  map_projector_->setPublisher(this);
  graph_->setCallbackMode(map_projector_);

  /// route planner
  auto planner_type =
      node_->declare_parameter<std::string>("planner_type", "distance");
  if (planner_type == "distance") {
    route_planner_.reset(
        new path_planning::SimplePlanner<pose_graph::RCGraph>(graph_));
  } else if (planner_type == "timedelta") {
    throw std::runtime_error{"Time delta planner not ported to VTR3!"};
  } else {
    LOG(ERROR) << "Planner type " << planner_type
               << " not recognized; defaulting to distance planning.";
    route_planner_.reset(
        new path_planning::SimplePlanner<pose_graph::RCGraph>(graph_));
  }
  LOG(INFO) << "Creating a route planner of type: " << planner_type;

  /// state estimation block
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node_);
  auto pipeline = pipeline_factory->make("pipeline");
  pipeline->setModuleFactory(std::make_shared<ROSModuleFactory>(node_));
  tactic_ = std::make_shared<Tactic>(Tactic::Config::fromROS(node_), node_,
                                     pipeline, graph_);
  tactic_->setPublisher(this);
  tactic_->setPipeline(mission_planning::PipelineMode::Idle);
  if (graph_->contains(VertexId(0, 0))) tactic_->setTrunk(VertexId(0, 0));

  /// path tracker
  /// \todo currently only support "robust_mpc_path_tracker"
  /// \todo create a path tracker factory in the path tracker package.
  auto path_tracker_ = path_tracker::PathTrackerMPC::Create(graph_, node_);
  tactic_->setPathTracker(path_tracker_);
  // clang-format off
  path_tracker_subscription_ = node_->create_subscription<std_msgs::msg::UInt8>("path_done_status", rclcpp::SystemDefaultsQoS(), std::bind(&Navigator::finishPath, this, std::placeholders::_1));
  // clang-format on

  /// state machine
  state_machine_ = state::StateMachine::InitialState(tactic_.get());
  state_machine_->setPlanner(route_planner_);

  /// mission server
  mission_server_.reset(new RosMissionServer(node_, state_machine_));

  // clang-format off
  /// robot, sensor frames and transforms
  robot_frame_ = node_->declare_parameter<std::string>("control_frame", "base_link");
  camera_frame_ = node_->declare_parameter<std::string>("camera_frame", "front_xb3");
  lidar_frame_ = node_->declare_parameter<std::string>("lidar_frame", "velodyne");
  T_lidar_robot_ = loadTransform(lidar_frame_, robot_frame_);
  T_camera_robot_ = loadTransform(camera_frame_, robot_frame_);

  /// data subscriptions
  // avoid early data in queue
  std::lock_guard<std::mutex> queue_lock(queue_mutex_);
  // example data subscription, start with this to add new data subscription
  example_data_sub_ = node_->create_subscription<ExampleDataMsg>("/example_data", rclcpp::SensorDataQoS(), std::bind(&Navigator::exampleDataCallback, this, std::placeholders::_1));
  // lidar pointcloud data subscription
  lidar_sub_ = node_->create_subscription<PointCloudMsg>("/raw_points", rclcpp::SensorDataQoS(), std::bind(&Navigator::lidarCallback, this, std::placeholders::_1));
  // stereo image subscription
  image_sub_ = node_->create_subscription<RigImagesMsg>("/xb3_images", rclcpp::SensorDataQoS(), std::bind(&Navigator::imageCallback, this, std::placeholders::_1));
  rig_calibration_client_ = node_->create_client<RigCalibrationSrv>("/xb3_calibration");
  // clang-format on

  /// launch the processing thread
  process_thread_ = std::thread(&Navigator::process, this);

  LOG(INFO) << "[Navigator] Initialization done!";
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

  LOG(INFO) << "[Navigator] Destruction done! Bye-bye.";
}

void Navigator::process() {
  // el::Helpers::setThreadName("processing-thread");
  while (!quit_) {
    std::unique_lock<std::mutex> queue_lock(queue_mutex_);

    /// print a warning if our queue is getting too big
    if (queue_.size() > 5) {
      LOG_EVERY_N(10, WARNING)
          << "[Navigator] Cache queue size is " << queue_.size();
    }

    /// wait for the data to be added to the queues
    while (queue_.empty() && quit_ == false) {
      // wait for a data point
      process_.wait(queue_lock);
    }

    /// process data if there is one in queue
    if (queue_.empty()) continue;

    // get the front in queue
    auto query_data = queue_.front();

    // sensor specific: make sure that we can add data to the queue again
    if (query_data->raw_pointcloud.is_valid()) pointcloud_in_queue_ = false;
    if (query_data->rig_images.is_valid()) image_in_queue_ = false;

    // pop the data off the front because we don't need them now
    queue_.pop();

    // unlock the queue so that new data can be added
    queue_lock.unlock();

    // execute the pipeline
    tactic_->runPipeline(query_data);

    // handle any transitions triggered by changes in localization status
    state_machine_->handleEvents();

    // (temp) process completion callback
    result_pub_->publish(ResultMsg());
  };

  LOG(INFO) << "[Navigator] Data processing thread completed.";
}

void Navigator::exampleDataCallback(const ExampleDataMsg::SharedPtr) {
  LOG(DEBUG) << "[Navigator] Received an example sensor data.";

  /// Some necessary processing

  /// Add to the queue and notify the processing thread
  // queue_.push(query_data);
  // process_.notify_one();
}

void Navigator::lidarCallback(const PointCloudMsg::SharedPtr msg) {
  LOG(DEBUG) << "[Navigator] Received a lidar pointcloud.";

  if (pointcloud_in_queue_) {
    LOG_EVERY_N(10, INFO)
        << "[Navigator] Skip pointcloud message because there is already "
           "one in queue.";
    return;
  }

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<QueryCache>();

  /// \todo (yuchen) need to distinguish this with stamp
  query_data->rcl_stamp.fallback(msg->header.stamp);

  // set time stamp
  TimeStampMsg stamp;
  stamp.nanoseconds_since_epoch =
      msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  query_data->stamp.fallback(stamp);

  // fill in the pointcloud
  std::vector<PointXYZ> pts;
  std::vector<double> ts;
  copyPointcloud(msg, pts, ts);
  query_data->raw_pointcloud.fallback(pts);
  query_data->raw_pointcloud_time.fallback(ts);

  // fill in the vehicle to sensor transform
  query_data->T_s_r.fallback(T_lidar_robot_);

  // add to the queue and notify the processing thread
  queue_.push(query_data);
  pointcloud_in_queue_ = true;
  process_.notify_one();
};

void Navigator::imageCallback(const RigImagesMsg::SharedPtr msg) {
  LOG(DEBUG) << "[Navigator] Received an stereo image.";

  if (image_in_queue_) {
    LOG_EVERY_N(16, INFO)
        << "[Navigator] Skip images message because there is already one "
           "in queue.";
    return;
  }

  if (!rig_calibration_) {
    fetchRigCalibration();
    LOG(WARNING) << "[Navigator] Dropping frame because no calibration data";
    return;
  }

  // Convert message to query_data format and store into query_data
  auto query_data = std::make_shared<QueryCache>();

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

  /// \todo get T_camera_robot
  // fill in the vehicle to sensor transform
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
      LOG(ERROR) << "Interrupted while waiting for the service. Exiting.";
      return;
    }
    LOG(INFO) << "Rig calibration not available, waiting again.";
  }

  // send and wait for the result
  auto request = std::make_shared<RigCalibrationSrv::Request>();
  auto response_callback =
      [this](rclcpp::Client<RigCalibrationSrv>::SharedFuture future) {
        auto response = future.get();
        rig_calibration_ = std::make_shared<vtr::vision::RigCalibration>(
            messages::copyCalibration(response->calibration));
      };
  auto response =
      rig_calibration_client_->async_send_request(request, response_callback);
}

void Navigator::publishPath(const tactic::LocalizationChain &chain) const {
  LOG(INFO) << "Publishing path from: " << chain.trunkVertexId()
            << " To: " << chain.endVertexID();

  PathMsg path_msg;

  path_msg.base_vertex_id = chain.trunkVertexId();
#if false
  path_msg.path.controlFrame = control_frame_;
  path_msg.path.localizationBaseFrame = "/path_base_link";
  path_msg.path.header.stamp = ros::Time::now();
  path_msg.path.header.frame_id = "path_base_link";
#endif
  for (auto it = chain.begin(); it != chain.end(); ++it) {
#if 0
    EdgeTransform pose = chain.pose(it);
    path_msg.path.poses.push_back(rosutil::toPoseMessage(pose.matrix()));
    path_msg.path.vertexIdList.push_back(it->v()->id());
#endif
    path_msg.vertex_id_list.push_back(it->v()->id());
  }

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

void Navigator::finishPath(const PathTrackerMsg::SharedPtr status_msg) {
  std::lock_guard<std::mutex> lck(queue_mutex_);

  auto name = state_machine_->name();
  LOG(DEBUG) << "[Lock Requested] finishPath";
  auto plck = tactic_->lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] finishPath";

  if (name != "::Repeat::Follow") {
    LOG(WARNING) << "Got following path response in state "
                 << state_machine_->name();
    if (name == "::Repeat::MetricLocalize" || name == "::Repeat::Plan") {
      LOG(WARNING) << "[Navigator] Path tracker was unable to process the "
                      "desired path; dropping to ::Idle";
      state_machine_->handleEvents(
          mission_planning::Event(mission_planning::state::Action::Abort),
          false);
      clearPath();
    }
    return;
  }

  if (status_msg->data == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    LOG(INFO) << "Path tracking complete";
    state_machine_->handleEvents(
        mission_planning::Event(mission_planning::state::Signal::GoalReached),
        true);
  } else if (status_msg->data == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
    LOG(ERROR) << "[Navigator] Path tracker was unable to process the desired "
                  "path; dropping to ::Idle";
    state_machine_->handleEvents(
        mission_planning::Event(mission_planning::state::Action::Abort), true);
  } else {
    LOG(ERROR) << "[Navigator] Got the following path response that didn't "
                  "make sense: "
               << status_msg->data
               << ". See ROS2's action_msgs/msg/GoalStatus.msg";
    state_machine_->handleEvents(
        mission_planning::Event(mission_planning::state::Action::Abort), true);
  }

  LOG(DEBUG) << "[Lock Released] finishPath";
}

}  // namespace navigation
}  // namespace vtr
