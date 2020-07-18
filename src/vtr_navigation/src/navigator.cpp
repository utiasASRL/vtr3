#include <vtr/navigation/navigator.h>
#include <vtr/planning/ros_callbacks.h>

#include <asrl/common/rosutil/transformation_utilities.hpp>

#if 0
#include <Eigen/Geometry>

#include <tf_conversions/tf_eigen.h>

#include <asrl/navigation/factories/RosTacticFactory.hpp>
#include <asrl/navigation/tactics/BasicTactic.hpp>
#include <asrl/navigation/tactics/ParallelTactic.hpp>

#include <babelfish_robochunk_translator/RigCalibrationCliServ.h>
#include <asrl/common/timing/TimeUtils.hpp>
#include <robochunk_sensor_msgs_RigCalibration_Conversions.hpp>
#endif

namespace vtr {
namespace navigation {

void Navigator::process(void) {
  // make sure the tactic instance hasn't yet been destroyed
  while (!quit_) {
#if 0  
    // print a warning if our queue is getting too big
    if (queue_.size() > 5) {
      LOG_EVERY_N(10, WARNING)
          << "Navigator cache queue size is " << queue_.size();
    }
    // easy reference
    QueryCachePtr query_data;

    {
      std::unique_lock<std::mutex> mylock(queue_lock_);

      // wait for the data to be added to the queues
      while (queue_.empty() && quit_ == false) {
        // unset busy flag.
        busy_ = false;

        // wait for a data point
        process_.wait(mylock);
      }

      // check if we have any data
      if (queue_.empty()) {
        // unset busy flag.
        busy_ = false;

        // see if we need to quit
        continue;
      }

      // copy
      query_data = queue_.front();

      // make sure that we can add images to the queue again
      if (query_data->rig_images.is_valid()) {
        image_in_queue_ = false;
      }

      // pop the data off the front because we don't need them now
      queue_.pop();
    }

    // fill in joy with latest joy msg
    if (joy_msg_) {
      query_data->joy = joy_msg_->buttons;
      joy_msg_.reset();
    }

    try {
      // Process the image data
      tactic_->runPipeline(query_data);

      // Handle any automatic transitions
      state_machine_->handleEvents();

      // Publish to rviz.
      // publishT_0_q(query_data);
      // publishPathViz(query_data, tactic_->chain_);

    } catch (const std::exception &e) {
      LOG(FATAL) << "Saving graph and dying due to unhandled exception: "
                 << e.what();
      graph_->save();
    }
#endif
  }
}

#if 0
// TODO: move somewhere common??? ros message conversion utils?
vision::RigImages Navigator::copyImages(
    const babelfish_robochunk_robochunk_sensor_msgs::RigImages &ros_images) {
  vision::RigImages rig_images;
  rig_images.name = ros_images.name;
  for (auto &ros_channel : ros_images.channels) {
    rig_images.channels.emplace_back();
    auto &rig_channel = rig_images.channels.back();
    rig_channel.name = ros_channel.name;
    for (auto &ros_image : ros_channel.cameras) {
      rig_channel.cameras.emplace_back();
      auto &image = rig_channel.cameras.back();
      image.name = ros_image.name;
      std::string encoding = ros_image.encoding;
      if (encoding == "mono8") {
        auto temp_cv = image.data =
            cv::Mat(ros_image.height, ros_image.width, CV_8UC1,
                    (char *)&ros_image.data[0]);
        image.data = temp_cv.clone();
        // memcpy((void*)&ros_image.data,(void*)&image.data,ros_image.width*ros_image.height);
      } else if (encoding == "bgr8") {
        auto temp_cv = cv::Mat(ros_image.height, ros_image.width, CV_8UC3,
                               (char *)&ros_image.data[0]);
        image.data = temp_cv.clone();
        // memcpy((void*)&ros_image.data,(void*)&image.data,ros_image.width*ros_image.height*3);
      } else {
        image.data = cv::Mat();
      }
    }
  }
  return rig_images;
}

// Image call back
void Navigator::ImageCallback(
    const babelfish_robochunk_robochunk_sensor_msgs::RigImages &rig_images) {
  if (wait_for_pos_msg_) {
    LOG(WARNING)
        << "[Navigator] Dropping frame because we're waiting for a pos message";
    return;
  }

  // Set busy flag.
  busy_ = true;

  if (drop_frames_ && image_in_queue_) {
    LOG(WARNING) << "[Navigator] Dropping frame (one already in queue)";
    return;
  }

  if (rig_calibration_ == nullptr) {
    rig_calibration_ = fetchCalibration();
  }

  // Set up the query data
  navigation::QueryCachePtr query_data(new navigation::QueryCache);
  auto &images = query_data->rig_images.fallback();
  auto &calibration_list = query_data->rig_calibrations.fallback();

  // Set the time stamp.
  robochunk::std_msgs::TimeStamp stamp;
  stamp.set_nanoseconds_since_epoch(
      rig_images.channels[0].cameras[0].stamp.nanoseconds_since_epoch);
  *query_data->stamp = stamp;

  // add the rig names
  auto &rig_names = query_data->rig_names.fallback();
  rig_names->push_back(sensor_frame_);

  // Fill in the calibration
  calibration_list->push_back(*rig_calibration_.get());

  // fill in the images
  images->emplace_back(copyImages(rig_images));

  // re-get the transform if we need to
  if (nonstatic_sensor_frame_) {
    tf::StampedTransform Tf_sensor_vehicle;
    if (gimbal_msg_) {
      // if a gimbal message exists, use that
      T_sensor_vehicle_ = rosutil::fromPoseMessage(gimbal_msg_->pose);
    } else {
      // otherwise, try and get it from the tf tree
      try {
        tf_listener_.lookupTransform(sensor_frame_, control_frame_,
                                     ros::Time(0), Tf_sensor_vehicle);
        // Set vehicle --> sensor transform
        T_sensor_vehicle_ =
            rosutil::fromStampedTransformation(Tf_sensor_vehicle);
        T_sensor_vehicle_.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
        tactic_->setTSensorVehicle(T_sensor_vehicle_);
      } catch (tf::TransformException ex) {
        LOG(ERROR) << "Could not look up sensor->vehicle transform!";
      }
    }
    T_sensor_vehicle_.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
    tactic_->setTSensorVehicle(T_sensor_vehicle_);
  }

  // add to the queue and notify
  queue_.push(query_data);
  image_in_queue_ = true;
  process_.notify_one();
}

// GPS call back
void Navigator::NavSatFixCallback(const sensor_msgs::NavSatFix &fix) {
  // Set busy flag.
  busy_ = true;

  // Set up the query data
  asrl::navigation::QueryCachePtr query_data(new asrl::navigation::QueryCache);

  // insert the GPS data as a position estimate
  asrl::navigation::Position pos;
  pos(0) = fix.latitude;
  pos(1) = fix.longitude;
  pos(2) = fix.altitude;
  query_data->position = pos;

  // Set the time stamp.
  robochunk::std_msgs::TimeStamp stamp;
  stamp.set_nanoseconds_since_epoch(fix.header.stamp.toNSec());
  *query_data->stamp = stamp;

  // we now have processed a position message
  wait_for_pos_msg_ = false;

  // add to the queue and notify
  queue_.push(query_data);
  process_.notify_one();
}

// Odometry message callback
void Navigator::OdomCallback(const nav_msgs::Odometry &odom_data) {
  // Set busy flag.
  busy_ = true;

  // Set up the query data
  asrl::navigation::QueryCachePtr query_data(new asrl::navigation::QueryCache);

  // insert the odom data as a position estimate
  asrl::navigation::Position pos;
  pos(0) = odom_data.pose.pose.position.x;
  pos(1) = odom_data.pose.pose.position.y;
  pos(2) = odom_data.pose.pose.position.z;
  query_data->position = pos;

  // insert the odom data as an orientation estimate
  auto &gimbal_ori = odom_data.pose.pose.orientation;
  query_data->orientation = asrl::navigation::Orientation(
      gimbal_ori.w, gimbal_ori.x, gimbal_ori.y, gimbal_ori.z);

  // Set the time stamp.
  robochunk::std_msgs::TimeStamp stamp;
  stamp.set_nanoseconds_since_epoch(odom_data.header.stamp.toNSec());
  *query_data->stamp = stamp;

  // we now have processed a position message
  wait_for_pos_msg_ = false;

  // add to the queue and notify
  queue_.push(query_data);
  process_.notify_one();
}

void Navigator::JoyCallback(const ::sensor_msgs::JoyPtr &joy_msg) {
  joy_msg_ = joy_msg;
}

void Navigator::ImuCallback(const ::sensor_msgs::ImuPtr &imu_msg) {
  // Set busy flag.
  busy_ = true;

  if (imu_calibration_ == nullptr) {
    imu_calibration_ = fetchImuCalibration();
  }

  // Set up the query data
  asrl::navigation::QueryCachePtr query_data(new asrl::navigation::QueryCache);
  auto &calibration_list = query_data->imu_calibrations.fallback();

  // Fill in the calibration
  calibration_list->push_back(*imu_calibration_.get());

  // insert the imu orientation data
  auto &ori = imu_msg->orientation;
  query_data->orientation =
      asrl::navigation::Orientation(ori.w, ori.x, ori.y, ori.z);

  // insert the angular velocity data
  asrl::navigation::AngularVelocity angvel;
  angvel(0) = imu_msg->angular_velocity.x;
  angvel(1) = imu_msg->angular_velocity.y;
  angvel(2) = imu_msg->angular_velocity.z;
  query_data->angular_velocity = angvel;

  // insert the linear acceleration data
  asrl::navigation::AngularVelocity linacc;
  linacc(0) = imu_msg->linear_acceleration.x;
  linacc(1) = imu_msg->linear_acceleration.y;
  linacc(2) = imu_msg->linear_acceleration.z;
  query_data->linear_acceleration = linacc;

  // add the pre-calibrated IMU bias
  query_data->imu_bias = imu_calibration_->bias;

  // Set the time stamp.
  robochunk::std_msgs::TimeStamp stamp;
  stamp.set_nanoseconds_since_epoch(imu_msg->header.stamp.toNSec());
  *query_data->stamp = stamp;

  // add to the queue and notify
  queue_.push(query_data);
  process_.notify_one();

  if (tactic_->path_tracker_) {
    asrl::common::timing::time_point chrono_stamp =
        asrl::common::timing::toChrono(stamp);
    tactic_->path_tracker_->updateImuState(*query_data->orientation,
                                           *query_data->linear_acceleration,
                                           chrono_stamp);
  }
}

void Navigator::GimbalCallback(
    const ::geometry_msgs::PoseStampedPtr &gimbal_msg) {
  gimbal_msg_ = gimbal_msg;
}

Navigator::RigCalibrationPtr Navigator::fetchCalibration() {
  babelfish_robochunk_translator::RigCalibrationCliServ srv;
  bool success = false;

  while (!success && ros::ok()) {
    if (ros::service::waitForService("/in/calibration_client",
                                     ros::Duration(0.1))) {
      auto calibration_client = nh_.serviceClient<
          ::babelfish_robochunk_translator::RigCalibrationCliServ>(
          "/babel/rig_calibration");
      success = calibration_client.call(srv);
    } else {
      LOG(ERROR) << "Calibration service unavailable";
    }

    if (!success) {
      LOG(INFO) << "Waiting for camera calibration!!";
    }
  }

  // TODO: we should be getting a generic rig claibration from ROS...
  auto &ros_calibration = srv.response.rigCalibResponse;
  robochunk::sensor_msgs::RigCalibration proto_calibration;
  babelfish::convertMessage(ros_calibration, &proto_calibration);

  auto rig_calibration = std::make_shared<vision::RigCalibration>(
      messages::copyCalibration(proto_calibration));

  return rig_calibration;
}

Navigator::ImuCalibrationPtr Navigator::fetchImuCalibration() {
  tf::StampedTransform Tf_imu_vehicle;
  for (int idx = 0; idx < 10; ++idx) {
    try {
      tf_listener_.waitForTransform(imu_frame_, control_frame_, ros::Time(0),
                                    ros::Duration(10));
      tf_listener_.lookupTransform(imu_frame_, control_frame_, ros::Time(0),
                                   Tf_imu_vehicle);
      break;
    } catch (tf::TransformException ex) {
      LOG(ERROR) << ex.what();
    }
  }

  tf::Transform tff = Tf_imu_vehicle.inverse();
  Eigen::Affine3d out;
  tf::transformTFToEigen(tff, out);

  // make a new calibration container
  ImuCalibrationPtr calibration(new vision::IMUCalibration);

  // copy the calibration matrix
  calibration->extrinsics = vision::Transform(out.matrix());

  // fill out the calibration bias with zeros for now;
  calibration->bias << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  return calibration;
}
#endif
void Navigator::initializePipeline() {
  tactic_->setPublisher(this);

  // Set the tactic into Idle initially
  tactic_->setPipeline(vtr::planning::PipelineType::Idle);

  nh_.param<std::string>("control_frame", control_frame_, "base_link");
  nh_.param<std::string>("sensor_frame", sensor_frame_, "front_xb3");
#if 0
  nh_.param<std::string>("imu_frame", imu_frame_, "imu");
  nh_.param<bool>("wait_for_pos_msg", wait_for_pos_msg_, false);
  nh_.param<bool>("drop_frames", drop_frames_, false);
#endif

  // Extract the Vehicle->Sensor transformation.
  tf::StampedTransform Tf_sensor_vehicle;
  for (int idx = 0; idx < 10; ++idx) {
    try {
      tf_listener_.waitForTransform(sensor_frame_, control_frame_, ros::Time(0),
                                    ros::Duration(10));
      tf_listener_.lookupTransform(sensor_frame_, control_frame_, ros::Time(0),
                                   Tf_sensor_vehicle);
      break;
    } catch (tf::TransformException ex) {
      LOG(ERROR) << ex.what();
    }
  }
  // Set vehicle --> sensor transform
  T_sensor_vehicle_ =
      asrl::rosutil::fromStampedTransformation(Tf_sensor_vehicle);
  T_sensor_vehicle_.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
  tactic_->setTSensorVehicle(T_sensor_vehicle_);
  // Set the state machine into the initial state, and instantiate a planner
  state_machine_ =
      vtr::planning::state::StateMachine::InitialState(tactic_.get());

  // Initialize callbacks for graph publishing/relaxation
  graphCallbacks_ = std::make_shared<vtr::planning::RosCallbacks>(graph_, nh_);
  graph_->setCallbackMode(graphCallbacks_);

  _buildPlanner();

  // Initialize the mission server
  mission_server_.reset(
      new vtr::planning::RosMissionServer(nh_, state_machine_));

  if (graph_->numberOfVertices() > 0) {
    graphCallbacks_->updateRelaxation();
    graph_->save();
  }

  if (graph_->contains(VertexId(0, 0))) {
    tactic_->setTrunk(VertexId(0, 0));
  }
  LOG(INFO) << "-- Initialization complete --";
}

bool Navigator::halt(bool force, bool save) {
#if 0
  // wait for the processing thread to join
  quit_ = true;
  if (process_thread_.joinable()) {
    process_.notify_one();
    process_thread_.join();
  }

  std::lock_guard<std::mutex> lck(queue_lock_);

  // If we never initialized, then halting always succeeds
  if (!mission_server_.get()) {
    return true;
  }
  // We require the user to explicitly force the halt if we are in a state other
  // than ::Idle, because this is messy
  if (!force && state_machine_->name() != "::Idle") {
    LOG(ERROR) << "Tried to halt the Navigator from " << state_machine_->name()
               << " without the force flag set!";
    return false;
  }

  // Things can get ugly if we kill the mission server while it has goals in it
  mission_server_->halt();

  // Reset ALL the things so they get cleaned up and stop accepting
  // message/release memory
  planner_.reset();
  state_machine_.reset();
  mission_server_.reset();
  tactic_->halt();
  tactic_.reset();
  graphCallbacks_.reset();

  // Save anything that we haven't written to disk yet, just in case
  // TODO: This will hopefully at least block things until the writing is
  // done???  Check this.
  if (save) {
    graph_->halt();
  }

  graph_.reset();
#endif

  return true;
}

bool Navigator::busyCallback(std_srvs::Trigger::Request& /*request*/,
                             std_srvs::Trigger::Response& response) {
  response.success = busy_;
  return true;
}
bool Navigator::_setGraphCallback(SetGraphRequest& request,
                                  SetGraphResponse& response) {
  std::lock_guard<std::mutex> lck(queue_lock_);

  if (!this->halt()) {
    response.result = "Cannot change graph while not in ::Idle";
    return false;
  }

  // The default setup functions pull data from ROS param
  nh_.setParam("data_dir", request.path);

  // If you don't explicitly cast to an integer here, ROS apparently cannot
  // tell that this integer is an integer.
  nh_.setParam("graph_index", int(request.graph_id));

  // generate a tactic from the factory
  std::string tactic_namespace = "tactic";
  vtr::navigation::ROSTacticFactory tacticfactory(&nh_, tactic_namespace);
  tactic_ = tacticfactory.makeVerified();

  // get a pointer to the graph
  graph_ = tactic_->poseGraph();

  // initialize a pipeline
  initializePipeline();
  return true;
}

#if 0
void Navigator::publishPath(const pose_graph::LocalizationChain &chain) {
  LOG(INFO) << "Publishing path from: " << chain.trunkVertexId()
            << " To: " << chain.endVertexID();

  auto time = ros::Time::now();
  EdgeTransform identity(true);

  // TODO: make these configurable.
  std::vector<geometry_msgs::TransformStamped> tfs;
  tfs.push_back(rosutil::toTransformStampedMessage(
      "/path_base_link", control_frame_, identity.matrix(), time));
  // loc_stream << static_cast<double>(stamp)/1e9 << "," <<
  // (time_now-time).toSec() << "," << se3Vec(0) << "," << se3Vec(1) << "," <<
  // se3Vec(2) << "\n";
  localizationBroadcaster_.sendTransform(tfs);

  // Use an action client to talk directly to the path tracker
  asrl__messages::FollowPathGoal goal;
  goal.path.controlFrame = control_frame_;
  goal.path.localizationBaseFrame = "/path_base_link";
  goal.path.header.stamp = ros::Time::now();
  goal.path.header.frame_id = "path_base_link";
  goal.path.speedLimits.clear();

  for (auto it = chain.begin(); it != chain.end(); ++it) {
    EdgeTransform pose = chain.pose(it);
    goal.path.poses.push_back(rosutil::toPoseMessage(pose.matrix()));
    goal.path.vertexIdList.push_back(it->v()->id());

    if (speed_ > 0.0) {
      goal.path.speedLimits.push_back(speed_);
    }
  }

  if (speed_ > 0.0) {
    goal.path.speedLimits.front() *= 0.5;
    goal.path.speedLimits.back() *= 0.5;
  }

  pathClient_.sendGoal(goal,
                       boost::bind(&Navigator::_pathCallback, this, _1, _2));
  tracking_ = true;

  // Also publish a path message for UI purposes
  // TODO: Is there some way to listen to the new goal messages so we don't have
  // to publish this twice?
  asrl__messages::Path path_msg;

  path_msg.baseVertexId = chain.trunkVertexId();
  path_msg.path.controlFrame = control_frame_;
  path_msg.path.localizationBaseFrame = "/path_base_link";
  path_msg.path.header.stamp = ros::Time::now();
  path_msg.path.header.frame_id = "path_base_link";
  for (auto it = chain.begin(); it != chain.end(); ++it) {
    EdgeTransform pose = chain.pose(it);
    path_msg.path.poses.push_back(rosutil::toPoseMessage(pose.matrix()));
    path_msg.vertexIdList.push_back(it->v()->id());
    path_msg.path.vertexIdList.push_back(it->v()->id());
  }

  followingPathPublisher_.publish(path_msg);
  return;
}

void Navigator::clearPath() {
  // Make sure we stop doing whatever we were doing before

  // TODO: Remove references to the old path tracker once migration complete.
  if (tactic_->path_tracker_) {
    LOG(INFO) << "Instance of the new path tracker detected. Not clearing path "
                 "for old PT. Stopping and joining new PT.";
    tactic_->path_tracker_->stopAndJoin();
    return;
  }

  if (tracking_) {
    pathClient_.cancelGoal();
    pathClient_.waitForResult();
    tracking_ = false;
  }

  // Publish an empty path message for the UI
  asrl__messages::Path path_msg;

  path_msg.baseVertexId = uint64_t(-1);
  path_msg.path.controlFrame = control_frame_;
  path_msg.path.localizationBaseFrame = "/path_base_link";
  path_msg.path.header.stamp = ros::Time::now();
  path_msg.path.header.frame_id = "path_base_link";
  path_msg.path.poses.clear();
  path_msg.vertexIdList.clear();
  path_msg.path.vertexIdList.clear();

  followingPathPublisher_.publish(path_msg);
}

bool Navigator::_reloadPlannerCallback(std_srvs::Trigger::Request &request,
                                       std_srvs::TriggerResponse &response) {
  _buildPlanner();
  response.success = true;
  return true;
}
#endif
void Navigator::_buildPlanner() {
#if 0
  std::string planner_type;
  nh_.param<std::string>("planner/type", planner_type, "distance");

  if (planner_type == "distance") {
    planner_.reset(new planning::SimplePlanner<RCGraph>(graph_));
  } else if (planner_type == "timedelta") {
    planning::TimeDeltaPlanner::Config planner_config;
    int minutes;
    double hours;

    nh_.param<double>("planner/time_falloff", planner_config.timeFalloff_,
                      0.50);
    nh_.param<double>("planner/days_falloff", planner_config.daysFalloff_,
                      0.25);
    nh_.param<double>("planner/max_contribution", planner_config.maxContrib_,
                      0.95);
    nh_.param<double>("planner/smoothing", planner_config.smoothing_, 0.0);
    nh_.param<int>("planner/max_runs", planner_config.maxRuns_, 5);
    nh_.param<bool>("planner/invert", planner_config.invert_, false);
    nh_.param<int>("planner/max_cache_age", minutes, 20);
    nh_.param<double>("planner/utc_offset", hours, -5.0);

    planner_config.max_cache_age_ = std::chrono::minutes(minutes);
    planner_config.utcOffset_ = std::chrono::hours(int(hours * 3600));

    planner_.reset(new planning::TimeDeltaPlanner(graph_, planner_config));
  } else {
    LOG(ERROR) << "Planner type " << planner_type
               << " not recognized; defaulting to distance planning.";
    planner_.reset(new planning::SimplePlanner<RCGraph>(graph_));
  }

  state_machine_->setPlanner(planner_);
  graphCallbacks_->setPlanner(planner_);
#endif
}
#if 0
void Navigator::_pathDoneCallback(const std_msgs::UInt8 status_msg) {
  actionlib::SimpleClientGoalState state(
      actionlib::SimpleClientGoalState::PENDING);
  FollowPathResultConstPtr result;

  switch (status_msg.data) {
    case actionlib_msgs::GoalStatus::PENDING:
      state = actionlib::SimpleClientGoalState::PENDING;
      break;
    case actionlib_msgs::GoalStatus::ABORTED:
      state = actionlib::SimpleClientGoalState::ABORTED;
      break;
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      state = actionlib::SimpleClientGoalState::SUCCEEDED;
      break;
  }

  _pathCallback(state, result);

  return;
}

void Navigator::_pathCallback(const actionlib::SimpleClientGoalState &state,
                              const FollowPathResultConstPtr &result) {
  std::lock_guard<std::mutex> lck(queue_lock_);

  auto name = this->state_machine_->name();
  LOG(DEBUG) << "[Lock Requested] _pathCallback";
  auto plck = tactic_->lockPipeline();
  LOG(DEBUG) << "[Lock Acquired] _pathCallback";

  if (name != "::Repeat::Follow") {
    LOG(WARNING) << "Got following path response in state "
                 << this->state_machine_->name();

    if (name == "::Repeat::MetricLocalize" || name == "::Repeat::Plan") {
      LOG(ERROR) << "[Navigator] Path tracker was unable to process the "
                    "desired path; dropping to ::Idle";
      this->state_machine_->handleEvents(
          planning::Event(planning::state::Action::Abort), false);
      this->clearPath();
    }

    return;
  }

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    LOG(INFO) << "Path tracking complete";
    this->state_machine_->handleEvents(
        planning::Event(planning::state::Signal::GoalReached), true);
  } else if (state == actionlib::SimpleClientGoalState::ABORTED ||
             state == actionlib::SimpleClientGoalState::LOST ||
             state == actionlib::SimpleClientGoalState::REJECTED) {
    LOG(ERROR) << "[Navigator] Path tracker was unable to process the desired "
                  "path; dropping to ::Idle";
    this->state_machine_->handleEvents(
        planning::Event(planning::state::Action::Abort), true);
  } else {
    LOG(ERROR)
        << "[Navigator] Got a following path response that didn't make sense: "
        << state.getText();
    this->state_machine_->handleEvents(
        planning::Event(planning::state::Action::Abort), true);
  }

  LOG(DEBUG) << "[Lock Released] _pathCallback";
}

void Navigator::updateLocalization(const TransformType &T_leaf_trunk,
                                   const TransformType &T_root_trunk,
                                   const TransformType &T_leaf_trunk_sensor,
                                   uint64_t stamp) {
  // Publish Transformations
  std::string frame_id = "path";
  std::string branch_frame_id = "branch";
  std::string path_base_link = "path_base_link";
  std::string subgraph_base_link_id = "subgraph_base_link";
  std::string topocentric_base_link_id = "topocentric";
  std::vector<geometry_msgs::TransformStamped> tfs;

  auto T_path_leaf = T_root_trunk * T_leaf_trunk.inverse();

  ros::Time time(static_cast<double>(stamp) / 1e9);
  auto time_now = ros::Time::now();
  auto time_diff = time_now - time;
  if (time_diff > ros::Duration(600.)) {
    LOG_N_TIMES(1, WARNING)
        << "loc update is " << time_diff << " old. Using time::now instead.";
    time = time_now;
  }
  // TODO: make these configurable.
  tfs.push_back(rosutil::toTransformStampedMessage(
      "/path_base_link", control_frame_, T_path_leaf.matrix(), time));
  // loc_stream << static_cast<double>(stamp)/1e9 << "," <<
  // (time_now-time).toSec() << "," << se3Vec(0) << "," << se3Vec(1) << "," <<
  // se3Vec(2) << "\n";
  localizationBroadcaster_.sendTransform(tfs);

  // publish the transform between the trunk and leaf in sensor frame
  auto T_leaf_trunk_sensor_msg = rosutil::toTransformStampedMessage(
      "/trunk_sensor", "/leaf_sensor", T_leaf_trunk_sensor.matrix(), time);
  gimbalPublisher_.publish(T_leaf_trunk_sensor_msg);

  // Publish tracking status with full covariance
  asrl__messages::TrackingStatus status;
  status.tf_stamp = time;
  status.header.stamp = time_now;
  status.T_leaf_trunk << T_leaf_trunk;

  // set the vertex IDs
  status.twig = tactic_->chain_.twigVertexId();
  status.branch = tactic_->chain_.branchVertexId();
  status.trunk = tactic_->chain_.trunkVertexId();

  // insert the name of the current state-machine state
  if (state_machine_) status.state = state_machine_->name();

  statusPublisher_.publish(status);
}

void Navigator::publishRobot(const Localization &persistentLoc,
                             uint64_t pathSeq, const Localization &targetLoc) {
  RobotMsg msg;

  msg.pathSeq = pathSeq;
  msg.trunkVertex = persistentLoc.v;
  msg.targetVertex = targetLoc.v;
  msg.T_leaf_trunk << persistentLoc.T;

  if (persistentLoc.T.covarianceSet()) {
    msg.cov_leaf_trunk.push_back(std::sqrt(persistentLoc.T.cov()(0, 0)));
    msg.cov_leaf_trunk.push_back(std::sqrt(persistentLoc.T.cov()(1, 1)));
    msg.cov_leaf_trunk.push_back(std::sqrt(persistentLoc.T.cov()(5, 5)));
  }

  if (targetLoc.localized) {
    msg.T_leaf_target << targetLoc.T;

    if (targetLoc.T.covarianceSet()) {
      msg.cov_leaf_target.push_back(std::sqrt(targetLoc.T.cov()(0, 0)));
      msg.cov_leaf_target.push_back(std::sqrt(targetLoc.T.cov()(1, 1)));
      msg.cov_leaf_target.push_back(std::sqrt(targetLoc.T.cov()(5, 5)));
    }
  }

  if (state_machine_) msg.state = state_machine_->name();
  msg.header.stamp = ros::Time::now();

  // Publish the robot position
  robotPublisher_.publish(msg);
}

void Navigator::publishT_0_q(QueryCachePtr q_data) {
  // Make sure we have a T_0_q.
  if (!q_data->T_0_q.is_valid()) {
    return;
  }

  // Create topocentric-robot frame
  Eigen::Vector3d r_ba_ina = -q_data->T_0_q->r_ab_inb();
  r_ba_ina(2) -= 3;
  auto T_0_q_tran =
      lgmath::se3::Transformation(Eigen::Matrix3d::Identity(), r_ba_ina);

  // Convert to tf stamped transform.
  Eigen::Affine3d T_q_0_affine(q_data->T_0_q->inverse().matrix());
  tf::Transform T_q_0_tf;
  tf::poseEigenToTF(T_q_0_affine, T_q_0_tf);
  auto T_q_0_tf_stamped = tf::StampedTransform(T_q_0_tf, ros::Time::now(),
                                               "base_link", "topocentric");
  T_0_q_broadcaster_.sendTransform(T_q_0_tf_stamped);

  Eigen::Affine3d T_0_q_tran_affine(T_0_q_tran.matrix());
  tf::Transform T_0_q_tran_tf;
  tf::poseEigenToTF(T_0_q_tran_affine, T_0_q_tran_tf);
  auto T_0_q_tran_tf_stamped = tf::StampedTransform(
      T_0_q_tran_tf, ros::Time::now(), "topocentric", "topocentric-robot");
  T_0_q_broadcaster_.sendTransform(T_0_q_tran_tf_stamped);
}

void Navigator::publishPathViz(QueryCachePtr q_data,
                               pose_graph::LocalizationChain &loc_chain) {
  // Skip if cache isn't populated yet.
  if (!q_data->live_id.is_valid()) {
    return;
  }

  // Skip if we've already published this run.
  if (q_data->live_id->majorId() == last_run_viz_) {
    return;
  }

  // Skip if we aren't localized yet.
  if (!loc_chain.isLocalized()) {
    return;
  }

  last_run_viz_ = q_data->live_id->majorId();

  visualization_msgs::MarkerArray clear_array;
  clear_array.markers.push_back(visualization_msgs::Marker());
  clear_array.markers.back().action = visualization_msgs::Marker::DELETEALL;
  path_pub_.publish(clear_array);

  visualization_msgs::MarkerArray loc_chain_markers;
  loc_chain_markers.markers.push_back(visualization_msgs::Marker());

  // Get T_0_root
  auto root_v_id = loc_chain.begin()->v()->id();
  auto evaluator = std::make_shared<PrivilegedEvaluator>();
  evaluator->setGraph(graph_.get());
  auto subgraph = graph_->getSubgraph(VertexId(0, 0), evaluator);
  auto path_0_trunk = subgraph->breadthFirstSearch(root_v_id, VertexId(0, 0));
  auto T_0_root = pose_graph::Eval::ComposeTfAccumulator(
      path_0_trunk->begin(VertexId(0, 0)), path_0_trunk->end(),
      lgmath::se3::Transformation());

  auto &path_marker = loc_chain_markers.markers.back();
  path_marker.header.frame_id = "topocentric";
  path_marker.ns = "loc_chain";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.scale.x = 0.1;
  path_marker.color.r = 0;
  path_marker.color.b = 1;
  path_marker.color.g = 1;
  path_marker.color.a = 1;

  for (auto it = loc_chain.begin(); it != loc_chain.end(); ++it) {
    if (it != loc_chain.begin()) {
      T_0_root = pose_graph::Eval::ComposeTfAccumulator(it - 1, it, T_0_root);
    }

    Eigen::Vector4d origin;
    origin << 0, 0, 0, 1;
    auto origin_t = T_0_root * origin;

    path_marker.points.push_back(geometry_msgs::Point());
    path_marker.points.back().x = origin_t(0);
    path_marker.points.back().y = origin_t(1);
    path_marker.points.back().z = origin_t(2);
  }

  path_pub_.publish(loc_chain_markers);
}
#endif
}  // namespace navigation
}  // namespace vtr
