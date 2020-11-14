#include <vtr_testing/module_offline.hpp>

#include <tf2_ros/transform_listener.h>

#include <vtr_navigation/types.hpp>
#include <vtr_lgmath_extensions/conversions.hpp>

/// \brief Privileged edge mask. Used to create a subgraph on privileged edges
using PrivilegedEvaluator = pose_graph::eval::Mask::Privileged<pose_graph::RCGraph>::Caching;
using PrivilegedEvaluatorPtr = PrivilegedEvaluator::Ptr;


class ModuleLoc : public ModuleOffline {
 public:
  ModuleLoc(const std::shared_ptr<rclcpp::Node> node, fs::path &results_dir)
      : ModuleOffline(node, results_dir) {

    std::stringstream ss;
    ss << "run_" << std::setfill('0') << std::setw(6) << graph_->numberOfRuns();
    auto run_results_dir = fs::path(results_dir / ss.str());
    fs::create_directories(run_results_dir);
    loc_outstream_.open(run_results_dir / "loc.csv");
    loc_outstream_ << "timestamp,query run,query vertex,map run,map vertex,r\n";

    initializePipeline();
  }

  ~ModuleLoc() {
    saveLoc();
  }

 private:
  void initializePipeline() final {
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
    tactic_->setPipeline(mission_planning::PipelineType::MetricLocalization);

    // get the co-ordinate frame names
    control_frame_ =
        node_->declare_parameter<std::string>("control_frame", "base_link");
    sensor_frame_ =
        node_->declare_parameter<std::string>("sensor_frame", "front_xb3");

    // Extract the Vehicle->Sensor transformation.
    rclcpp::Clock::SharedPtr clock =
        std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer tf_buffer{clock};
    tf2_ros::TransformListener tf_listener{tf_buffer};
    auto tf_sensor_vehicle =
        tf_buffer.lookupTransform(sensor_frame_, control_frame_,
                                  tf2::TimePoint(), tf2::durationFromSec(5));
    T_sensor_vehicle_ = fromStampedTransformation(tf_sensor_vehicle);
    T_sensor_vehicle_.setZeroCovariance();
    tactic_->setTSensorVehicle(T_sensor_vehicle_);
  }

  void saveLoc() {
    navigation::EdgeTransform T_q_m(true);

    // get vertices from latest run
    auto root_vid = navigation::VertexId(graph_->numberOfRuns() - 1, 0);
    navigation::TemporalEvaluatorPtr evaluator(new navigation::TemporalEvaluator());
    evaluator->setGraph((void *) graph_.get());
    auto path_itr = graph_->beginDfs(root_vid, 0, evaluator);

    for (; path_itr != graph_->end(); ++path_itr) {

      auto loc_msg = path_itr->v()->retrieveKeyframeData<vtr_messages::msg::LocalizationStatus>("results_localization");

      if (loc_msg != nullptr && loc_msg->success) {

        uint64 q_id_64 = loc_msg->query_id;
        uint q_id_minor = (uint) q_id_64;
        uint q_id_major = (uint) (q_id_64 >> 32);

        uint64 m_id_64 = loc_msg->map_id;
        uint m_id_minor = (uint) m_id_64;
        uint m_id_major = (uint) (m_id_64 >> 32);

        LOG(INFO) << q_id_major << "-" << q_id_minor << ", " << m_id_major << "-" << m_id_minor;

        loc_msg->t_query_map >> T_q_m;

        std::streamsize prec = loc_outstream_.precision();
        loc_outstream_ << std::setprecision(21)
                       << (path_itr->v()->keyFrameTime().nanoseconds_since_epoch) / 1e9
                       << std::setprecision(prec)
                       << "," << q_id_major << "," << q_id_minor << "," << m_id_major << "," << m_id_minor;

        // flatten r vector to save
        auto tmp = T_q_m.r_ab_inb();
        auto T_flat = std::vector<double>(tmp.data(), tmp.data() + 3);
        for (auto v : T_flat) loc_outstream_ << "," << v;
        loc_outstream_ << "\n";
      }
    }
    loc_outstream_.close();
  }

  /** \brief Stream to save position from localization to csv */
  std::ofstream loc_outstream_;

};