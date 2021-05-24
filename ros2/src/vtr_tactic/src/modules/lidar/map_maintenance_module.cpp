#include <vtr_tactic/modules/lidar/map_maintenance_module.hpp>

namespace {
PointCloudMapMsg copyPointcloudMap(const std::shared_ptr<PointMap> &map) {
  const auto &points = map->cloud.pts;
  const auto &normals = map->normals;
  const auto &scores = map->scores;
  auto N = points.size();

  PointCloudMapMsg map_msg;
  map_msg.points.reserve(N);
  map_msg.normals.reserve(N);

  for (int i = 0; i < N; i++) {
    // points
    const auto &point = points[i];
    PointXYZMsg point_xyz;
    point_xyz.x = point.x;
    point_xyz.y = point.y;
    point_xyz.z = point.z;
    map_msg.points.push_back(point_xyz);
    // normals
    const auto &normal = normals[i];
    PointXYZMsg normal_xyz;
    normal_xyz.x = normal.x;
    normal_xyz.y = normal.y;
    normal_xyz.z = normal.z;
    map_msg.normals.push_back(normal_xyz);
    // scores
    map_msg.scores = scores;
  }
  return map_msg;
}
}  // namespace

namespace vtr {
namespace tactic {

void MapMaintenanceModule::runImpl(QueryCache &qdata, MapCache &mdata,
                                   const Graph::ConstPtr &graph) {
  // Construct the map if not exist
  if (!qdata.new_map) qdata.new_map.fallback(config_->map_voxel_size);

  // Get input and output data
  // input
  auto &T_s_r = *qdata.T_s_r;
  auto &T_r_m = *qdata.T_r_m_odo;
  // the following has to be copied because we will need to change them
  auto sub_pts = *qdata.preprocessed_pointcloud;
  auto normal_scores = *qdata.normal_scores;
  auto normals = *qdata.normals;
  // output
  auto &new_map = *qdata.new_map;

  // Transform subsampled points into the map frame
  auto T_m_s = (T_r_m.inverse() * T_s_r.inverse()).matrix();
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)sub_pts.data(), 3, sub_pts.size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat(
      (float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = (T_m_s.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (T_m_s.block(0, 3, 3, 1)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;

  // The update function is called only on subsampled points as the others have
  // no normal
  new_map.update(sub_pts, normals, normal_scores);
}

void MapMaintenanceModule::updateGraphImpl(QueryCache &qdata, MapCache &mdata,
                                           const Graph::Ptr &graph,
                                           VertexId live_id) {
  const auto &T_r_m = *qdata.T_r_m_odo;
  const auto &map = qdata.new_map;
  const auto &points = map->cloud.pts;
  const auto &normals = map->normals;
  const auto &scores = map->scores;

  /// Transform subsampled points into the map frame
  const auto T_r_m_mat = T_r_m.matrix();
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)points.data(), 3, points.size());
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> norms_mat(
      (float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = (T_r_m_mat.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (T_r_m_mat.block(0, 3, 3, 1)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;

  /// Store the submap into STPG \todo (yuchen) make this run faster!
  auto map_msg = copyPointcloudMap(qdata.new_map.ptr());
  auto vertex = graph->at(live_id);
  graph->registerVertexStream<PointCloudMapMsg>(live_id.majorId(), "pcl_map");
  vertex->insert("pcl_map", map_msg, *qdata.stamp);

  /// Clean up the current map
  qdata.new_map.clear();
}

void MapMaintenanceModule::visualizeImpl(QueryCache &qdata, MapCache &mdata,
                                         const Graph::ConstPtr &,
                                         std::mutex &) {
  if (!config_->visualize) return;

  if (!map_pub_)
    map_pub_ =
        qdata.node->create_publisher<PointCloudMsg>("new_map_points", 20);

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (qdata.new_map) {
    for (auto pt : (*qdata.new_map).cloud.pts)
      cloud.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    pcl::toROSMsg(cloud, *pc2_msg);
  }
  pc2_msg->header.frame_id = "odometry keyframe";
  pc2_msg->header.stamp = *qdata.rcl_stamp;

  map_pub_->publish(*pc2_msg);
}

}  // namespace tactic
}  // namespace vtr