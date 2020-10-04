#include <chrono>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#if false
#include <tf/transform_listener.h>
#endif
#include <vtr_navigation/factories/ros_tactic_factory.hpp>
#if false
#include <vtr_navigation/tactics/basic_tactic.h>
#include <vtr_navigation/tactics/parallel_tactic.h>

#include <babelfish_robochunk_robochunk_sensor_msgs/RigImages.h>
#include <robochunk_msgs/XB3CalibrationResponse.pb.h>
#include <vtr_vision/messages/bridge.h>
#include <vtr_vision/types.h>
#endif

namespace fs = std::filesystem;
using namespace vtr;

#if false
/** StereoNavigationNode::fromStampedTransformation */
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
#endif

class ModuleVO {
 public:
  ModuleVO(const std::shared_ptr<rclcpp::Node> node, fs::path &results_dir)
      : node_(node) {
    outstream_.open(results_dir / "results.csv");
    outstream_ << "timestamp,map vertex (run),map vertex (vertex)\n";

    navigation::ROSTacticFactory tactic_factory{node_, "tactic"};
    tactic_ = tactic_factory.makeVerified();
    graph_ = tactic_->poseGraph();

    initializePipeline();
  }
#if false
  ~ModuleVO() { saveGraph(); }

  void setCalibration(
      std::shared_ptr<vtr::vision::RigCalibration> &calibration) {
    rig_calibration_ = calibration;
  }

  void processImageData(
      std::shared_ptr<robochunk::sensor_msgs::RigImages> &rig_images,
      const robochunk::std_msgs::TimeStamp &stamp) {
    vtr::navigation::QueryCachePtr query_data(new vtr::navigation::QueryCache);
    // add the rig names
    auto &rig_names = query_data->rig_names.fallback();
    rig_names->push_back(sensor_frame_);

    // add the images to the cache
    auto &images = query_data->rig_images.fallback();
    images->emplace_back(vtr::messages::copyImages(*rig_images.get()));

    auto &calibration_list = query_data->rig_calibrations.fallback();
    calibration_list->emplace_back(*rig_calibration_.get());

    // add the timestamp
    *query_data->stamp = stamp;

    tactic_->runPipeline(query_data);
    if (dynamic_cast<vtr::navigation::ParallelTactic *>(tactic_.get())) {
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
  }

  void saveGraph() {
    vtr::navigation::EdgeTransform t_curr_start(true);
    auto path_itr = graph_->beginDfs(vtr::navigation::VertexId(0, 0));
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

  const vtr::navigation::BasicTactic &tactic() { return *tactic_; }
#endif
 private:
  void initializePipeline() {
#if false
    // normally the state machine would add a run when a goal is started. We
    // spoof that here.
    tactic_->addRun();

    // Create a branch pipeline.
    tactic_->setPipeline(vtr::planning::PipelineType::VisualOdometry);

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
#endif
  }

  const std::shared_ptr<rclcpp::Node> node_;
  std::ofstream outstream_;

  int idx = 0;
#if false
  asrl::common::timing::SimpleTimer timer;

  lgmath::se3::Transformation T_sensor_vehicle_;

  /// @brief TF listener to get the camera->robot transformation.
  tf::TransformListener tf_listener_;
  std::string sensor_frame_;
  std::string control_frame_;

#endif
  // Pipeline / tactic
  std::shared_ptr<navigation::BasicTactic> tactic_;
  std::shared_ptr<pose_graph::RCGraph> graph_;
#if false
  // add the rig images to the query data.
  std::shared_ptr<vtr::vision::RigCalibration> rig_calibration_;
#endif
};