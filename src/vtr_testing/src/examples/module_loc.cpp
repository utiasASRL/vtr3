// #define _GLIBCXX_USE_NANOSLEEP 1

#include <ros/ros.h>
#include <tf/transform_listener.h>
#if 0
#include <vtr/navigation/assemblies.h>
#include <vtr/navigation/factories/ros_assembly_factory.h>
#endif
#include <vtr/navigation/factories/ros_tactic_factory.h>
#include <vtr/navigation/pipelines/branch_pipeline.h>
#include <vtr/navigation/pipelines/metric_localization_pipeline.h>
#include <vtr/navigation/tactics/basic_tactic.h>
#include <vtr/navigation/tactics/parallel_tactic.h>

// robochunk includes
#include <babelfish_robochunk_robochunk_sensor_msgs/RigImages.h>
#include <robochunk_msgs/MessageBase.pb.h>
#include <robochunk_msgs/XB3CalibrationResponse.pb.h>
#include <robochunk/base/DataStream.hpp>
#include <robochunk/util/fileUtils.hpp>

// #include <asrl/planning/RosCallbacks.h>
#include <asrl/common/logging.hpp>
#include <asrl/common/timing/SimpleTimer.hpp>
#include <vtr/vision/messages/bridge.h>

// ** FOLLOWING LINE SHOULD BE USED ONCE AND ONLY ONCE IN WHOLE APPLICATION **
// ** THE BEST PLACE TO PUT THIS LINE IS IN main.cpp RIGHT AFTER INCLUDING
// easylogging++.h **
INITIALIZE_EASYLOGGINGPP

/// \brief Privileged Edge mask. This is used to create a subgraph on priveleged
/// edges.
using PrivilegedEvaluator = asrl::pose_graph::Eval::Mask::Privileged<
    asrl::pose_graph::RCGraph>::Caching;
using PrivilegedEvaluatorPtr = PrivilegedEvaluator::Ptr;

// StereoNavigationNode::fromStampedTransformation
vtr::navigation::EdgeTransform fromStampedTransformation(
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
  return vtr::navigation::EdgeTransform(vtr::navigation::EdgeTransform(T),
                                        true);
}

class ModuleLoc {
 public:
  ModuleLoc(ros::NodeHandle &nh) : nh_(nh) {
    // generate a tactic from the factory
    std::string tactic_namespace = "tactic";
    vtr::navigation::ROSTacticFactory tacticfactory(&nh_, tactic_namespace);
    tactic_ = tacticfactory.makeVerified();

    // get a pointer to the graph
    graph_ = tactic_->poseGraph();

    // initialize a pipeline
    initializePipeline();
  }

  ~ModuleLoc() { saveGraph(); }
  void setCalibration(
      std::shared_ptr<vtr::vision::RigCalibration> &calibration) {
    rig_calibration_ = calibration;
  }

  void processImageData(
      std::shared_ptr<robochunk::sensor_msgs::RigImages> &rig_images,
      const robochunk::std_msgs::TimeStamp &stamp) {
    // make a new query cache
    vtr::navigation::QueryCachePtr query_data(new vtr::navigation::QueryCache);

    // add the rig names to the cache
    auto &rig_names = *query_data->rig_names.fallback();
    rig_names.push_back("front_xb3");

    // add the images to the cache
    auto &images = query_data->rig_images.fallback();
    images->emplace_back(vtr::messages::copyImages(*rig_images.get()));

    // add the calibration to the cache
    auto &calibration_list = query_data->rig_calibrations.fallback();
    calibration_list->emplace_back(*rig_calibration_.get());

    // add the timestamp
    *query_data->stamp = stamp;

    tactic_->runPipeline(query_data);
    if (dynamic_cast<vtr::navigation::ParallelTactic *>(tactic_.get())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void saveGraph() {
    if (save_graph_ == true) graph_->save();
  }

  const vtr::navigation::BasicTactic &tactic() { return *tactic_; }

 private:
  void initializePipeline() {
    // Get the path that we should repeat
    vtr::navigation::BasicTactic::VertexId::Vector sequence;
    sequence.reserve(graph_->numberOfVertices());
    // Extract the privileged sub graph from the full graph.
    PrivilegedEvaluatorPtr evaluator(new PrivilegedEvaluator());
    evaluator->setGraph(graph_.get());
    auto privileged_path = graph_->getSubgraph(0ul, evaluator);
    for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
         ++it) {
      LOG(INFO) << it->v()->id();
      sequence.push_back(it->v()->id());
    }

    // normally the state machine would add a run when a goal is started. We
    // spoof that here.
    tactic_->addRun();

    // Initialize localization
    tactic_->setPath(sequence);

    // Create a Metric Localization pipeline.
    tactic_->setPipeline(asrl::planning::PipelineType::MetricLocalization);

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
    T_sensor_vehicle_.setCovariance(Eigen::Matrix<double, 6, 6>::Zero());
    tactic_->setTSensorVehicle(T_sensor_vehicle_);

    // whether or not to save the graph
    nh_.param<bool>("save_graph", save_graph_, false);
  }

  ros::NodeHandle &nh_;

  /// TF listener to get the camera->robot transformation.
  tf::TransformListener tf_listener_;
  std::string sensor_frame_;
  std::string control_frame_;
  vtr::navigation::EdgeTransform T_sensor_vehicle_;

  // Pipeline / tactic
  std::shared_ptr<vtr::navigation::BasicTactic> tactic_;
  std::shared_ptr<asrl::pose_graph::RCGraph> graph_;

  // add the rig images to the query data.
  std::shared_ptr<vtr::vision::RigCalibration> rig_calibration_;
  bool save_graph_;
};

int main(int argc, char **argv) {
  LOG(INFO) << "Starting Module Loc, beep bop boop";

  // enable parallelisation
  Eigen::initParallel();

  // ros publishers for visualisations
  ros::init(argc, argv, "module_loc");
  ros::NodeHandle nh("~");
  std::string prefix("module_loc/");

  std::string data_directory, sim_run, stream_name;
  int start_index;
  int stop_index;
  nh.param<std::string>("input_data_dir", data_directory, "/path/to/data/");
  nh.param<std::string>("sim_run", sim_run, "/sim_run/");
  nh.param<std::string>("stream_name", stream_name, "/data_stream/");
  nh.param<int>("start_index", start_index, 1);
  nh.param<int>("stop_index", stop_index, 20000);
  robochunk::base::ChunkStream stereo_stream(data_directory + sim_run,
                                             stream_name);

  ModuleLoc vo(nh);

  robochunk::msgs::RobochunkMessage calibration_msg;
  // Check out the calibration
  if (stereo_stream.fetchCalibration(calibration_msg) == true) {
    std::cout << "Calibration fetched..." << std::endl;
    std::shared_ptr<vtr::vision::RigCalibration> rig_calib = nullptr;

    LOG(INFO) << "Trying to extract a sensor_msgs::RigCalibration...";
    std::shared_ptr<robochunk::sensor_msgs::RigCalibration> rig_calibration =
        calibration_msg
            .extractSharedPayload<robochunk::sensor_msgs::RigCalibration>();
    if (rig_calibration == nullptr) {
      LOG(WARNING)
          << "Trying to extract a sensor_msgs::RigCalibration failed, so I'm "
             "going to try to extract a sensor_msgs::XB3CalibrationResponse...";
      std::shared_ptr<robochunk::sensor_msgs::XB3CalibrationResponse>
          xb3_calibration = calibration_msg.extractSharedPayload<
              robochunk::sensor_msgs::XB3CalibrationResponse>();
      if (xb3_calibration == nullptr) {
        LOG(ERROR) << "Trying to extract a sensor_msgs::XB3CalibrationResponse "
                      "failed. Calibration extraction failed!!";
      } else {
        LOG(INFO)
            << "Successfully extracted a sensor_msgs::XB3CalibrationResponse.";
        rig_calib = std::make_shared<vtr::vision::RigCalibration>(
            vtr::messages::copyCalibration(*xb3_calibration));
      }
    } else {
      LOG(INFO) << "Successfully extracted a sensor_msgs::RigCalibration.";
      rig_calib = std::make_shared<vtr::vision::RigCalibration>(
          vtr::messages::copyCalibration(*rig_calibration));
    }

    if (rig_calib != nullptr) {
      printf("received camera calibration!\n");
      vo.setCalibration(rig_calib);
    } else {
      printf("ERROR: intrinsic params is not the correct type: (actual: %s \n",
             calibration_msg.header().type_name().c_str());
      return -1;
    }
  } else {
    printf("ERROR: Could not read calibration message!\n");
  }

  // Seek to an absolute index
  stereo_stream.seek(static_cast<uint32_t>(start_index));

  LOG(INFO) << "Done setup and seeked to: " << start_index;

  // Get the first message
  bool continue_stream = true;
  robochunk::msgs::RobochunkMessage data_msg;
  continue_stream &= stereo_stream.next(data_msg);
  int idx = 0;
  asrl::common::timing::SimpleTimer timer;
  while (continue_stream == true && idx + start_index < stop_index &&
         ros::ok()) {
    auto rig_images =
        data_msg.extractSharedPayload<robochunk::sensor_msgs::RigImages>();
    if (rig_images != nullptr) {
      vo.processImageData(rig_images, data_msg.header().sensor_time_stamp());
    } else {
      LOG(ERROR) << "Data is NULL!";
    }

    idx++;
    if (idx % 100 == 0) {
      double elapsed = timer.elapsedMs();
      double avg_speed = elapsed / idx;
      LOG(INFO) << "node: " << ros::this_node::getName() << "  "
                << "vtx: " << vo.tactic().currentVertexID() << "  "
                << "done: "
                << (static_cast<double>(idx) /
                    static_cast<double>(stop_index - start_index)) *
                       100
                << " %  "
                << "rate: " << 1000 / avg_speed << " hz";
    }

    data_msg.Clear();
    continue_stream &= stereo_stream.next(data_msg);
  }
}
