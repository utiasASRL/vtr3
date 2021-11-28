#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "vtr_common/timing/time_utils.hpp"
#include "vtr_common/utils/filesystem.hpp"
#include "vtr_lidar/pipeline.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace vtr;
using namespace vtr::common;
using namespace vtr::logging;
using namespace vtr::tactic;
using namespace vtr::lidar;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navigator");

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

  // Pose graph
  auto graph = tactic::Graph::MakeShared((data_dir / "graph").string(), true);

  auto module_factory = std::make_shared<ROSModuleFactory>(node);
  module_factory->add<lidar::ChangeDetectionModule>();

  auto mdl = module_factory->get("localization.change_detection");

  // Parameters
  const unsigned run_id = node->declare_parameter<int>("run_id", 1);

  /// robot lidar transformation is hard-coded - check measurements.
  Eigen::Matrix4d T_lidar_robot_mat;
  T_lidar_robot_mat << 1, 0, 0, -0.06, 0, 1, 0, 0, 0, 0, 1, -1.45, 0, 0, 0, 1;
  EdgeTransform T_lidar_robot(T_lidar_robot_mat);
  T_lidar_robot.setZeroCovariance();

  std::queue<tactic::VertexId> ids;
  size_t depth = 5;

  /// Create a temporal evaluator
  auto evaluator = std::make_shared<tactic::TemporalEvaluator<tactic::Graph>>();
  evaluator->setGraph(graph.get());

  auto subgraph = graph->getSubgraph(tactic::VertexId(run_id, 0), evaluator);
  for (auto it = subgraph->begin(tactic::VertexId(run_id, 0));
       it != subgraph->end(); ++it) {
    const auto vertex = it->v();
    const auto time_range = vertex->timeRange();
    const auto scan_msgs = vertex->retrieve<PointScan<PointWithInfo>>(
        "point_scan", "vtr_lidar_msgs/msg/PointScan", time_range.first,
        time_range.second);
    for (const auto &scan_msg : scan_msgs) {
      lidar::LidarQueryCache qdata;
      qdata.node = node;

      auto locked_scan_msg_ref = scan_msg->sharedLocked();  // lock the msg
      auto &locked_scan_msg = locked_scan_msg_ref.get();

      // get scan timestamp
      const auto stamp = locked_scan_msg.getTimestamp();
      qdata.stamp.emplace(stamp);

      // get T_s_r
      const auto T_s_r = T_lidar_robot;
      qdata.T_s_r.emplace(T_lidar_robot);

      const auto &point_scan = locked_scan_msg.getData();
      // get undistorted lidar scan
      const auto &point_scan_data = point_scan.point_map();
      qdata.undistorted_point_cloud.emplace(point_scan_data);

      // find the privileged vertex it has been localized against
      const auto neighbors = graph->at(vertex->id())->spatialNeighbours();
      if (neighbors.empty()) continue;
      const auto loc_vid = *neighbors.begin();
      const auto &T_ov_s = point_scan.T_vertex_map();
      const auto &T_lv_ov = graph->at(loc_vid, vertex->id())->T();
      const auto &T_r_lv = (T_lv_ov * T_ov_s * T_s_r).inverse();
      qdata.change_detection_async.emplace(loc_vid, T_r_lv);

      // retrieve the localization map from the vertex
      const auto loc_vertex = graph->at(loc_vid);
      const auto map_msg = loc_vertex->retrieve<PointMap<PointWithInfo>>(
          "point_map", "vtr_lidar_msgs/msg/PointMap");
      auto locked_map_msg = map_msg->sharedLocked();
      qdata.curr_map_loc = std::make_shared<PointMap<PointWithInfo>>(
          locked_map_msg.get().getData());

      mdl->runAsync(qdata, graph, nullptr, tactic::Task::Priority{},
                    tactic::Task::DepId{});

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // memory management
    ids.push(it->v()->id());
    if (ids.size() > depth) {
      graph->at(ids.front())->unload();
      ids.pop();
    }
  }

  graph->save();
  graph.reset();

  LOG(WARNING) << "Map Saving done!";

  rclcpp::shutdown();
}