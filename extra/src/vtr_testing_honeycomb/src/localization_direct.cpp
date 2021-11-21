#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/storage_options.hpp"

#include "vtr_common/timing/time_utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_logging/logging_init.hpp"
#include "vtr_navigation/navigator.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::navigation;
using namespace vtr::tactic;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

  // odometry sequence directory
  const auto odo_dir_str =
      node->declare_parameter<std::string>("odo_dir", "/tmp");
  fs::path odo_dir{utils::expand_user(utils::expand_env(odo_dir_str))};

  // localization sequence directory
  const auto loc_dir_str =
      node->declare_parameter<std::string>("loc_dir", "/tmp");
  fs::path loc_dir{utils::expand_user(utils::expand_env(loc_dir_str))};

  // Output directory
  const auto data_dir_str =
      node->declare_parameter<std::string>("data_dir", "/tmp");
  fs::path data_dir{utils::expand_user(utils::expand_env(data_dir_str))};

  // Configure logging
  const auto log_to_file = node->declare_parameter<bool>("log_to_file", false);
  const auto log_debug = node->declare_parameter<bool>("log_debug", false);
  const auto log_enabled = node->declare_parameter<std::vector<std::string>>(
      "log_enabled", std::vector<std::string>{});
  std::string log_filename;
  if (log_to_file) {
    // Log into a subfolder of the data directory (if requested to log)
    auto log_name = "vtr-" + timing::toIsoFilename(timing::clock::now());
    log_filename = data_dir / (log_name + ".log");
  }
  configureLogging(log_filename, log_debug, log_enabled);

  LOG(WARNING) << "Odometry Directory: " << odo_dir.string();
  LOG(WARNING) << "Localization Directory: " << loc_dir.string();
  LOG(WARNING) << "Output Directory: " << data_dir.string();

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  // Pipeline
  auto pipeline_factory = std::make_shared<ROSPipelineFactory>(node);
  pipeline_factory->add<lidar::LidarPipeline>();
  auto pipeline = pipeline_factory->make("pipeline");

  // Tactic
  auto tactic = std::make_shared<Tactic>(Tactic::Config::fromROS(node), node,
                                         pipeline, graph);

  tactic->setPipeline(PipelineMode::Following);
  tactic->addRun();

  // Get the path that we should repeat
  VertexId::Vector sequence;
  sequence.reserve(graph->numberOfVertices());
  LOG(WARNING) << "Total number of vertices: " << graph->numberOfVertices();
  // Extract the privileged sub graph from the full graph.
  using LocEvaluator = eval::Mask::Privileged<RCGraph>::Caching;
  LocEvaluator::Ptr evaluator(new LocEvaluator());
  evaluator->setGraph(graph.get());
  auto privileged_path = graph->getSubgraph(0ul, evaluator);
  std::stringstream ss;
  ss << "Repeat vertices: ";
  for (auto it = privileged_path->begin(0ul); it != privileged_path->end();
       ++it) {
    ss << it->v()->id() << " ";
    sequence.push_back(it->v()->id());
  }
  LOG(WARNING) << ss.str();

  tactic->setPath(sequence);

  // Frame and transforms
  std::string robot_frame = "robot";
  std::string lidar_frame = "honeycomb";

  /// robot lidar transformation is hard-coded - check measurements.
  Eigen::Matrix4d T_lidar_robot_mat;
  T_lidar_robot_mat << 1, 0, 0, -0.06, 0, 1, 0, 0, 0, 0, 1, -1.45, 0, 0, 0, 1;
  EdgeTransform T_lidar_robot(T_lidar_robot_mat);
  T_lidar_robot.setZeroCovariance();
  LOG(WARNING) << "Transform from " << robot_frame << " to " << lidar_frame
               << " has been set to" << T_lidar_robot;

  auto tf_static_broadcaster =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  auto msg = tf2::eigenToTransform(Eigen::Affine3d(T_lidar_robot.matrix()));
  msg.header.frame_id = lidar_frame;
  msg.child_frame_id = robot_frame;
  tf_static_broadcaster->sendTransform(msg);

  // Load dataset
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = loc_dir.string();
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;  // default
  storage_options.max_cache_size = 0;    // default
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back("/points");

  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);
  reader.set_filter(filter);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  int frame = 0;
  while (rclcpp::ok() && reader.has_next()) {
    // load rosbag2 message
    auto bag_message = reader.read_next();
    rclcpp::SerializedMessage msg(*bag_message->serialized_data);
    auto points = std::make_shared<sensor_msgs::msg::PointCloud2>();
    serializer.deserialize_message(&msg, points.get());

    LOG(WARNING) << "Loading point cloud frame " << frame << " with timestamp "
                 << std::fixed
                 << points->header.stamp.sec * 1e9 +
                        points->header.stamp.nanosec;

    // Convert message to query_data format and store into query_data
    auto query_data = std::make_shared<lidar::LidarQueryCache>();

    /// \todo (yuchen) need to distinguish this with stamp
    query_data->rcl_stamp.emplace(points->header.stamp);
    storage::Timestamp timestamp =
        points->header.stamp.sec * 1e9 + points->header.stamp.nanosec;
    query_data->stamp.emplace(timestamp);

    // put in the pointcloud msg pointer into query data
    query_data->pointcloud_msg = points;

    // fill in the vehicle to sensor transform and frame names
    query_data->robot_frame.emplace(robot_frame);
    query_data->lidar_frame.emplace(lidar_frame);
    query_data->T_s_r.emplace(T_lidar_robot);

    // execute the pipeline
    tactic->runPipeline(query_data);

    ++frame;
  }

  std::this_thread::sleep_for(5s);
  LOG(WARNING) << "Saving pose graph and reset.";
  tactic.reset();
  graph->save();
  graph.reset();
  LOG(WARNING) << "Saving pose graph and reset. - done!";

  rclcpp::shutdown();
}