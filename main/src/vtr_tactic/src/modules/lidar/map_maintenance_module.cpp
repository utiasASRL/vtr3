#include <vtr_tactic/modules/lidar/map_maintenance_module.hpp>

namespace vtr {
namespace tactic {
namespace lidar {

void MapMaintenanceModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                         const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config_->map_voxel_size);
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void MapMaintenanceModule::runImpl(QueryCache &qdata, MapCache &,
                                   const Graph::ConstPtr &) {
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

void MapMaintenanceModule::updateGraphImpl(QueryCache &qdata, MapCache &,
                                           const Graph::Ptr &graph,
                                           VertexId live_id) {
  const auto &T_r_m = *qdata.T_r_m_odo;

  /// get a shared pointer copy
  const auto map = qdata.new_map.ptr();
  auto &points = map->cloud.pts;
  auto &normals = map->normals;
  auto &scores = map->scores;

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

  /// create a new map to rebuild kd-tree \todo optimize this
  qdata.new_map.fallback(config_->map_voxel_size);
  (*qdata.new_map).update(points, normals, scores);
}

void MapMaintenanceModule::visualizeImpl(QueryCache &qdata, MapCache &,
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

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr