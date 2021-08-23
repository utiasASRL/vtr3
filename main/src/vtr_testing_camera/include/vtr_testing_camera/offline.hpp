#pragma once

#include <tf2_ros/transform_listener.h>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_common/utils/filesystem.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/pipelines/pipeline_factory.hpp>
#include <vtr_tactic/tactic.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_vision/pipeline.hpp>

// camera specific
#include <vtr_vision/messages/bridge.hpp>

using RigImagesMsg = vtr_messages::msg::RigImages;
using RigCalibrationMsg = vtr_messages::msg::RigCalibration;

using namespace vtr;

namespace {

tactic::EdgeTransform loadTransform(std::string source_frame,
                                    std::string target_frame) {
  rclcpp::Clock::SharedPtr clock =
      std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer{clock};
  tf2_ros::TransformListener tf_listener{tf_buffer};
  auto tf_source_target = tf_buffer.lookupTransform(
      source_frame, target_frame, tf2::TimePoint(), tf2::durationFromSec(5));
  tf2::Stamped<tf2::Transform> tf2_source_target;
  tf2::fromMsg(tf_source_target, tf2_source_target);
  tactic::EdgeTransform T_source_target(
      common::rosutils::fromStampedTransformation(tf2_source_target));
  T_source_target.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
  return T_source_target;
}

}  // namespace

class OfflineNavigator {
 public:
  OfflineNavigator(const rclcpp::Node::SharedPtr node, std::string output_dir)
      : node_(node) {
    /// data storage directory
    output_dir =
        common::utils::expand_user(common::utils::expand_env(output_dir));

    /// pose graph
    graph_ = pose_graph::RCGraph::LoadOrCreate(output_dir + "/graph.index", 0);
    LOG(INFO) << "[Navigator] Pose graph has " << graph_->numberOfVertices()
              << " vertices.";

    /// state estimation block
    auto pipeline_factory = std::make_shared<tactic::ROSPipelineFactory>(node_);
    pipeline_factory->add<vision::StereoPipeline>();
    auto pipeline = pipeline_factory->make("pipeline");
    tactic_ = std::make_shared<tactic::Tactic>(
        tactic::Tactic::Config::fromROS(node_), node_, pipeline, graph_);

    tactic_->setPublisher(nullptr);  // don't use these publishers in offline

    /// robot, sensor frames
    robot_frame_ =
        node_->declare_parameter<std::string>("robot_frame", "base_link");
    camera_frame_ =
        node_->declare_parameter<std::string>("camera_frame", "front_xb3");
    T_sensor_vehicle_ = loadTransform(camera_frame_, robot_frame_);

    // setup output of results to CSV
    std::stringstream ss;
    ss << "results_run_" << std::setfill('0') << std::setw(6)
       << graph_->numberOfRuns();
    auto run_results_dir = fs::path(fs::path(output_dir) / ss.str());
    fs::create_directories(run_results_dir);
    outstream_.open(run_results_dir / "vo.csv");
    outstream_ << "timestamp,vertex major id (run),vertex minor id "
                  "(vertex),r,,,T (col major)\n";
  }

  ~OfflineNavigator() {
    saveVO();
    graph_->save();
  }

  void setCalibration(std::shared_ptr<vision::RigCalibration> calibration) {
    rig_calibration_ = calibration;
  }

  void process(const RigImagesMsg::SharedPtr msg) {
    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<vision::CameraQueryCache>();

    /// \todo (yuchen) need to differentiate this with stamp
    query_data->rcl_stamp.fallback(node_->now());

    // set time stamp
    query_data->stamp.fallback(msg->vtr_header.sensor_time_stamp);

    // add the rig names
    auto &rig_names = query_data->rig_names.fallback();
    rig_names->push_back(camera_frame_);
    msg->name = camera_frame_;  /// \todo (yuchen) should not be set here

    // fill in the images
    auto &images = query_data->rig_images.fallback();
    images->emplace_back(vtr::messages::copyImages(*msg));

    // fill in the calibration
    auto &calibration_list = query_data->rig_calibrations.fallback();
    calibration_list->emplace_back(*rig_calibration_);

    // fill in the vehicle to sensor transform
    query_data->T_sensor_vehicle.fallback(T_sensor_vehicle_);

    tactic_->runPipeline(query_data);
  }

  /** \brief Save VO estimates to CSV file as way to plot. */
  void saveVO() {
    tactic::EdgeTransform T_curr(true);
    auto root_vid = tactic::VertexId(graph_->numberOfRuns() - 1, 0);
    tactic::TemporalEvaluator::Ptr evaluator(new tactic::TemporalEvaluator());
    evaluator->setGraph((void *)graph_.get());
    auto path_itr = graph_->beginDfs(root_vid, 0, evaluator);

    for (; path_itr != graph_->end(); ++path_itr) {
      T_curr = T_curr * path_itr->T();
      T_curr.reproject(true);
      if (path_itr->from().isValid()) {
        LOG(INFO) << path_itr->e()->id();
      }

      std::streamsize prec = outstream_.precision();
      outstream_ << std::setprecision(21)
                 << (path_itr->v()->keyFrameTime().nanoseconds_since_epoch) /
                        1e9
                 << std::setprecision(prec) << ","
                 << path_itr->v()->id().majorId() << ","
                 << path_itr->v()->id().minorId();
      // flatten r vector to save
      auto tmp = T_curr.r_ab_inb();
      auto r_flat = std::vector<double>(tmp.data(), tmp.data() + 3);
      for (auto v : r_flat) outstream_ << "," << v;

      // also save whole matrix for use in plotting localization
      auto tmp2 = T_curr.matrix();
      auto T_flat = std::vector<double>(tmp2.data(), tmp2.data() + 16);

      for (auto v : T_flat) outstream_ << "," << v;
      outstream_ << "\n";
    }
    outstream_.close();
  }

 protected:
  /** \brief The ROS2 node */
  const rclcpp::Node::SharedPtr node_;

  // vtr building blocks
  tactic::Tactic::Ptr tactic_;
  pose_graph::RCGraph::Ptr graph_;

  /// robot and sensor specific stuff
  std::string robot_frame_;
  std::string camera_frame_;
  /** \brief Calibration for the stereo rig */
  std::shared_ptr<vision::RigCalibration> rig_calibration_;
  lgmath::se3::TransformationWithCovariance T_sensor_vehicle_;

  /** \brief Stream to save position from integrated VO to csv */
  std::ofstream outstream_;
};