#include <vtr_tactic/modules/lidar/windowed_map_recall_module.hpp>

namespace {
void retrievePointCloudMap(const PointCloudMapMsg::SharedPtr &map_msg,
                           std::vector<PointXYZ> &points,
                           std::vector<PointXYZ> &normals,
                           std::vector<float> &scores) {
  auto N = map_msg->points.size();
  points.reserve(N);
  normals.reserve(N);
  scores.reserve(N);

  for (unsigned i = 0; i < N; i++) {
    const auto &point = map_msg->points[i];
    const auto &normal = map_msg->normals[i];
    const auto &score = map_msg->scores[i];
    // Add all points to the vector container
    points.push_back(PointXYZ(point.x, point.y, point.z));
    normals.push_back(PointXYZ(normal.x, normal.y, normal.z));
    scores.push_back(score);
  }
}
#if false
void migratePointCloudMap(const lgmath::se3::TransformationWithCovariance &T,
                          std::vector<PointXYZ> &points,
                          std::vector<PointXYZ> &normals,
                          std::vector<float> &scores) {
  /// Transform subsampled points into the map frame
  const auto T_mat = T.matrix();
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)points.data(), 3, points.size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat(
      (float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = (T_mat.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (T_mat.block(0, 3, 3, 1)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;
}
#endif
}  // namespace

namespace vtr {
namespace tactic {
namespace lidar {

void WindowedMapRecallModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                            const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config_->map_voxel_size);
  config_->depth = node->declare_parameter<int>(param_prefix + ".depth", config_->depth);
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void WindowedMapRecallModule::runImpl(QueryCache &qdata, MapCache &,
                                      const Graph::ConstPtr &graph) {
  // input
  auto &map_id = *qdata.map_id;

  // load vertex data
  auto run = graph->run(map_id.majorId());
  run->registerVertexStream<PointCloudMapMsg>(
      "pcl_map", true, pose_graph::RegisterMode::Existing);

  /// Recall multiple map
  // // Iterate on the temporal edges to get the window.
  // PrivilegedEvaluator::Ptr evaluator(new PrivilegedEvaluator());
  // evaluator->setGraph((void *)graph.get());
  // std::vector<VertexId> vertices;
  // auto itr = graph->beginBfs(map_id, config_->depth, evaluator);
  // for (; itr != graph->end(); ++itr) {
  //   const auto current_vertex = itr->v();
  //   // add the current, privileged vertex.
  //   vertices.push_back(current_vertex->id());
  //   LOG(ERROR) << "Yuchen (relevant vertices): " << current_vertex->id();
  // }
  // auto sub_graph = graph->getSubgraph(vertices);

  // // cache all the transforms so we only calculate them once
  // pose_graph::PoseCache<pose_graph::RCGraph> pose_cache(graph, map_id);

  // // construct the map
  // auto map = std::make_shared<PointMap>(config_->map_voxel_size);
  // for (auto vid : sub_graph->subgraph().getNodeIds()) {
  //   // get transformation
  //   auto T_root_curr = pose_cache.T_root_query(vid);
  //   // migrate submaps
  //   auto vertex = graph->at(vid);
  //   vertex->load("pcl_map");  /// \todo should  be in retrieveKeyframeData?
  //   const auto &map_msg =
  //       vertex->retrieveKeyframeData<PointCloudMapMsg>("pcl_map");
  //   std::vector<PointXYZ> points;
  //   std::vector<PointXYZ> normals;
  //   std::vector<float> scores;
  //   retrievePointCloudMap(map_msg, points, normals, scores);
  //   migratePointCloudMap(T_root_curr, points, normals, scores);
  //   map->update(points, normals, scores);
  // }

  /// Recall a single map
  auto vertex = graph->at(map_id);
  vertex->load("pcl_map");  /// \todo shouldn't this be in retrieve?
  const auto &map_msg =
      vertex->retrieveKeyframeData<PointCloudMapMsg>("pcl_map");
  std::vector<PointXYZ> points;
  std::vector<PointXYZ> normals;
  std::vector<float> scores;
  retrievePointCloudMap(map_msg, points, normals, scores);
  auto map = std::make_shared<PointMap>(config_->map_voxel_size);
  map->update(points, normals, scores);

  qdata.current_map_loc = map;
}

void WindowedMapRecallModule::visualizeImpl(QueryCache &qdata, MapCache &,
                                            const Graph::ConstPtr &,
                                            std::mutex &) {
  if (!config_->visualize) return;

  if (!map_pub_)
    map_pub_ =
        qdata.node->create_publisher<PointCloudMsg>("curr_loc_map_points", 20);

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (qdata.current_map_loc) {
    for (auto pt : (*qdata.current_map_loc).cloud.pts)
      cloud.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    pcl::toROSMsg(cloud, *pc2_msg);
  }
  pc2_msg->header.frame_id = "localization keyframe";
  pc2_msg->header.stamp = *qdata.rcl_stamp;

  map_pub_->publish(*pc2_msg);
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr