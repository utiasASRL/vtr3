#include <chrono>
#include <experimental/filesystem>  // experimental not needed in gcc 8.0+

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <asrl/navigation/factories/ros_tactic_factory.h>
// Note: we should have a base tactic rather than using basic tactic as base
#include <asrl/navigation/tactics/base_tactic.h>
#include <asrl/navigation/tactics/basic_tactic.h>

// #include <asrl/navigation/tactics/ParallelTactic.hpp>
// #include <asrl/navigation/assemblies/Assemblies.hpp>
// #include <asrl/navigation/assemblies/RosAssemblyBuilder.hpp>
// #include <asrl/navigation/pipelines/BranchPipeline.hpp>

#include <babelfish_robochunk_robochunk_sensor_msgs/RigImages.h>
#include <robochunk_msgs/XB3CalibrationResponse.pb.h>
// #include <asrl/planning/RosCallbacks.hpp>
#include <asrl/vision/Types.hpp>

#include <asrl/vision/messages/bridge.hpp>

namespace fs = std::experimental::filesystem;

#if 0
/// The following has been moved to vision messages bridge
// TODO: move somewhere common??? ros message conversion utils?
asrl::vision::RigImages copyImages(
    const babelfish_robochunk_robochunk_sensor_msgs::RigImages &ros_images) {
  asrl::vision::RigImages rig_images;
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
        image.data = cv::Mat(ros_image.height, ros_image.width, CV_8UC1);
        memcpy((void *)&ros_image.data, (void *)&image.data,
               ros_image.width * ros_image.height);
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
#endif

/** StereoNavigationNode::fromStampedTransformation
 */
lgmath::se3::Transformation fromStampedTransformation(
    tf::StampedTransform const &T_base_child) {
  // Eigen::Vector3d translation(T_base_child.getOrigin().x(),
  // T_base_child.getOrigin().y(), T_base_child.getOrigin().z());
  Eigen::Matrix4d T;
  T.setIdentity();
  tf::Matrix3x3 C_bc(T_base_child.getRotation());
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      T(row, col) = C_bc[row][col];
    }
  }
  T(0, 3) = T_base_child.getOrigin().x();
  T(1, 3) = T_base_child.getOrigin().y();
  T(2, 3) = T_base_child.getOrigin().z();
  return lgmath::se3::Transformation(T);
}

class ModuleVO {
 public:
  ModuleVO(ros::NodeHandle &nh, fs::path &results_dir) : nh_(nh) {
    outstream_.open(results_dir / "results.csv");
    outstream_ << "timestamp,map vertex (run),map vertex (vertex)\n";

    std::string tactic_namespace = "tactic";
    asrl::navigation::ROSTacticFactory tactic_factory(&nh_, tactic_namespace);
    tactic_ = tactic_factory.makeVerified();

    graph_ = tactic_->poseGraph();

    initializePipeline();

    idx = 0;
  }
#if 0
  ~ModuleVO() { saveGraph(); }

#endif
  void setCalibration(
      std::shared_ptr<asrl::vision::RigCalibration> &calibration) {
    rig_calibration_ = calibration;
  }

  void processImageData(
      std::shared_ptr<robochunk::sensor_msgs::RigImages> &rig_images,
      const robochunk::std_msgs::TimeStamp &stamp) {
    asrl::navigation::QueryCachePtr query_data(
        new asrl::navigation::QueryCache);
    // add the rig names
    auto &rig_names = query_data->rig_names.fallback();
    rig_names->push_back(sensor_frame_);

    // add the images to the cache
    auto &images = query_data->rig_images.fallback();
    images->emplace_back(asrl::messages::copyImages(*rig_images.get()));

    auto &calibration_list = query_data->rig_calibrations.fallback();
    calibration_list->emplace_back(*rig_calibration_.get());

    // add the timestamp
    *query_data->stamp = stamp;

#if 0
    tactic_->runPipeline(query_data);
    if (dynamic_cast<asrl::navigation::ParallelTactic *>(tactic_.get())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    idx++;
    if (idx % 100 == 0) {
      double elapsed = timer.elapsedMs();
      double avg_speed = elapsed / 100;
      LOG(INFO) << "proc. image: node: " << ros::this_node::getName() << "  "
                << "vtx: " << tactic().currentVertexID() << "  "
                << "rate: " << 1000 / avg_speed << " hz";
      timer.reset();
    }
#endif
  }

#if 0
  void saveGraph() {
    asrl::navigation::EdgeTransform t_curr_start(true);
    auto path_itr = graph_->beginDfs(asrl::navigation::VertexId(0, 0));
    for (; path_itr != graph_->end(); ++path_itr) {
      t_curr_start = t_curr_start * path_itr->T();
      if (path_itr->from().isValid()) LOG(INFO) << path_itr->e()->id();
      outstream_
          << boost::lexical_cast<std::string>(
                 static_cast<double>(
                     path_itr->v()->keyFrameTime().nanoseconds_since_epoch()) /
                 1e9)
          << "," << path_itr->v()->id().majorId() << ","
          << path_itr->v()->id().minorId() << "\n";
    }
    outstream_.close();
    graph_->save();
  }

  const asrl::navigation::BasicTactic &tactic() { return *tactic_; }
#endif

 private:
  int idx;
  asrl::common::timing::SimpleTimer timer;
  std::ofstream outstream_;
  void initializePipeline() {
    // normally the state machine would add a run when a goal is started. We
    // spoof that here.
    tactic_->addRun();

    // Create a branch pipeline.
    tactic_->setPipeline(asrl::planning::PipelineType::VisualOdometry);

    // get the co-ordinate frame names
    nh_.param<std::string>("control_frame", control_frame_, "base_link");
    nh_.param<std::string>("sensor_frame", sensor_frame_, "front_xb3");

    // Extract the Vehicle->Sensor transformation.
    tf::StampedTransform Tf_sensor_vehicle;
    for (int idx = 0; idx < 10; ++idx) {
      try {
        tf_listener_.waitForTransform(sensor_frame_, control_frame_,
                                      ros::Time(0), ros::Duration(10));
        tf_listener_.lookupTransform(sensor_frame_, control_frame_,
                                     ros::Time(0), Tf_sensor_vehicle);
        break;
      } catch (tf::TransformException ex) {
        LOG(ERROR) << ex.what();
      }
    }

    T_sensor_vehicle_ = fromStampedTransformation(Tf_sensor_vehicle);
    tactic_->setTSensorVehicle(T_sensor_vehicle_);
  }
  ros::NodeHandle &nh_;
  lgmath::se3::Transformation T_sensor_vehicle_;

  /// @brief TF listener to get the camera->robot transformation.
  tf::TransformListener tf_listener_;
  std::string sensor_frame_;
  std::string control_frame_;

  // Pipeline / tactic
  std::shared_ptr<asrl::navigation::BasicTactic> tactic_;  // base tactic
  std::shared_ptr<asrl::pose_graph::RCGraph> graph_;

  // add the rig images to the query data.
  std::shared_ptr<asrl::vision::RigCalibration> rig_calibration_;
};
