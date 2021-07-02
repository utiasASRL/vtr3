#include <vtr_tactic/modules/lidar/map_recall_module.hpp>

namespace {

#if false
std::shared_ptr<vtr::lidar::PointMap> copyPointcloudMap(
    const float voxel_size, const PointCloudMapMsg::SharedPtr &map_msg) {
  auto N = map_msg->points.size();

  std::vector<PointXYZ> points;
  points.reserve(N);
  std::vector<PointXYZ> normals;
  normals.reserve(N);
  std::vector<float> scores;
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
  auto map = std::make_shared<vtr::lidar::PointMap>(voxel_size, points, normals, scores);
  return map;
}
#endif

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

}  // namespace

namespace vtr {
namespace tactic {
namespace lidar {

void MapRecallModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                    const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config_->map_voxel_size);
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void MapRecallModule::runImpl(QueryCache &qdata, MapCache &,
                              const Graph::ConstPtr &graph) {
  if (*qdata.first_frame) {
    LOG(INFO) << "First keyframe, simply return.";
    return;
  }

  // input
  auto &live_id = *qdata.live_id;

  LOG(DEBUG) << "Loading vertex id: " << live_id.minorId();
  if (qdata.current_map_odo_vid && *qdata.current_map_odo_vid == live_id) {
    LOG(DEBUG) << "Map already loaded, simply return. Map size is: "
               << (*qdata.current_map_odo).cloud.pts.size();
  } else {
    // load map from vertex
    auto vertex = graph->at(live_id);
    const auto &map_msg =
        vertex->retrieveKeyframeData<PointCloudMapMsg>("pcl_map");
    std::vector<PointXYZ> points;
    std::vector<PointXYZ> normals;
    std::vector<float> scores;
    retrievePointCloudMap(map_msg, points, normals, scores);
    auto map = std::make_shared<vtr::lidar::PointMap>(config_->map_voxel_size);
    map->update(points, normals, scores);
    qdata.current_map_odo = map;
    qdata.current_map_odo_vid.fallback(live_id);
    qdata.current_map_odo_T_v_m.fallback(
        lgmath::se3::TransformationWithCovariance(true));
  }
}

void MapRecallModule::visualizeImpl(QueryCache &qdata, MapCache &,
                                    const Graph::ConstPtr &, std::mutex &) {
  if (!config_->visualize) return;

  if (*qdata.first_frame) return;

  if (!map_pub_)
    map_pub_ =
        qdata.node->create_publisher<PointCloudMsg>("curr_map_points", 20);

  /// \note this is slow...
  const auto T_v_m = qdata.current_map_odo_T_v_m->matrix();
  auto points = (*qdata.current_map_odo).cloud.pts;
  // Transform points into the live frame
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)points.data(), 3, points.size());
  Eigen::Matrix3f R_tot = (T_v_m.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (T_v_m.block(0, 3, 3, 1)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (qdata.current_map_odo) {
    for (auto pt : points)
      cloud.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    pcl::toROSMsg(cloud, *pc2_msg);
  }
  pc2_msg->header.frame_id = "odometry keyframe";
  pc2_msg->header.stamp = *qdata.rcl_stamp;

  map_pub_->publish(*pc2_msg);
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr