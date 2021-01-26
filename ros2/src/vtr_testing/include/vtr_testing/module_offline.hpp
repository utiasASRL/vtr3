#include <filesystem>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_messages/msg/rig_images.hpp>
#include <vtr_navigation/factories/ros_tactic_factory.hpp>
#include <vtr_vision/messages/bridge.hpp>

namespace fs = std::filesystem;
using namespace vtr;

class ModuleOffline {
 public:
  ModuleOffline(const std::shared_ptr<rclcpp::Node> node, fs::path &results_dir)
      : node_(node) {
    navigation::ROSTacticFactory tactic_factory{node_, "tactic"};
    tactic_ = tactic_factory.makeVerified();
    graph_ = tactic_->poseGraph();

    std::stringstream ss;
    ss << "run_" << std::setfill('0') << std::setw(6) << graph_->numberOfRuns();
    auto run_results_dir = fs::path(results_dir / ss.str());
    fs::create_directories(run_results_dir);
    outstream_.open(run_results_dir / "vo.csv");
    outstream_ << "timestamp,vertex major id (run),vertex minor id "
                  "(vertex),r,,,T (col major)\n";
  }

  ~ModuleOffline() { saveGraph(); }

  void setCalibration(std::shared_ptr<vision::RigCalibration> calibration) {
    rig_calibration_ = calibration;
  }

  void processImageData(
      std::shared_ptr<vtr_messages::msg::RigImages> rig_images,
      const vtr_messages::msg::TimeStamp &stamp) {
    navigation::QueryCachePtr query_data{new navigation::QueryCache};

    // add the timestamp
    *query_data->stamp = stamp;

    // add the rig names
    auto &rig_names = query_data->rig_names.fallback();
    rig_names->push_back(sensor_frame_);

    // add the images to the cache
    rig_images->name = sensor_frame_;  // \todo (yuchen) should not be set here
    auto &images = query_data->rig_images.fallback();
    images->emplace_back(messages::copyImages(*rig_images.get()));

    // add calibration data
    auto &calibration_list = query_data->rig_calibrations.fallback();
    calibration_list->emplace_back(*rig_calibration_.get());

    tactic_->runPipeline(query_data);
#if false
    if (dynamic_cast<navigation::ParallelTactic *>(tactic_.get())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
#endif

    idx++;
    if (idx % 100 == 0) {
      double elapsed = timer.elapsedMs();
      double avg_speed = elapsed / 100;
      LOG(INFO) << "proc. image: node: " << node_->get_name() << "  "
                << "vtx: " << tactic_->currentVertexID() << "  "
                << "rate: " << 1000 / avg_speed << " hz";
      timer.reset();
    }
  }

  void saveGraph() {
    saveVO();
    graph_->save();
  }

  void saveVO() {
    navigation::EdgeTransform T_curr(true);
    auto root_vid = navigation::VertexId(graph_->numberOfRuns() - 1, 0);
    navigation::TemporalEvaluatorPtr evaluator(
        new navigation::TemporalEvaluator());
    evaluator->setGraph((void *)graph_.get());
    auto path_itr = graph_->beginDfs(root_vid, 0, evaluator);

    for (; path_itr != graph_->end(); ++path_itr) {
      T_curr = T_curr * path_itr->T();
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
  virtual void initializePipeline() = 0;

  /** \brief The ROS2 node */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief Stream to save position from integrated VO to csv */
  std::ofstream outstream_;

  int idx = 0;
  common::timing::SimpleTimer timer;

  std::string sensor_frame_;
  std::string control_frame_;
  lgmath::se3::TransformationWithCovariance T_sensor_vehicle_;

  // Pipeline / tactic
  std::shared_ptr<navigation::BasicTactic> tactic_;
  std::shared_ptr<pose_graph::RCGraph> graph_;

  // add the rig images to the query data.
  std::shared_ptr<vision::RigCalibration> rig_calibration_;
};