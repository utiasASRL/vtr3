#include <vtr_tactic/modules/lidar/map_maintenance_module.hpp>

namespace vtr {
namespace tactic {
namespace lidar {

void MapMaintenanceModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                         const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->map_voxel_size = node->declare_parameter<float>(param_prefix + ".map_voxel_size", config_->map_voxel_size);

  config_->horizontal_resolution = node->declare_parameter<float>(param_prefix + ".horizontal_resolution", config_->horizontal_resolution);
  config_->vertical_resolution = node->declare_parameter<float>(param_prefix + ".vertical_resolution", config_->vertical_resolution);

  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void MapMaintenanceModule::runImpl(QueryCache &qdata, MapCache &,
                                   const Graph::ConstPtr &) {
  // Construct the map if not exist
  if (!qdata.new_map) qdata.new_map.fallback(config_->map_voxel_size);

  // Get input and output data
  // input
  const auto &T_s_r = *qdata.T_s_r;
  const auto &T_r_m = *qdata.T_r_m_odo;
  // the following has to be copied because we will need to change them
  auto points = *qdata.undistorted_pointcloud;
  auto normals = *qdata.undistorted_normals;
  auto normal_scores = *qdata.normal_scores;
  // output
  auto &new_map = *qdata.new_map;

  using PCEigen = Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>;
  // Transform subsampled points into the map frame
  auto T_m_s = (T_r_m.inverse() * T_s_r.inverse()).matrix().cast<float>();
  PCEigen pts_mat((float *)points.data(), 3, points.size());
  PCEigen norms_mat((float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = T_m_s.block<3, 3>(0, 0);
  Eigen::Vector3f T_tot = T_m_s.block<3, 1>(0, 3);
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;
  if (qdata.current_map_odo) {
    const auto &old_map = *qdata.current_map_odo;
    const auto T_old_new = (*qdata.current_map_odo_T_v_m).inverse();
    vtr::lidar::PointMapMigrator map_migrator(T_old_new.matrix(), old_map,
                                              new_map);
    map_migrator.update(points, normals, normal_scores);
  } else {
    // The update function is called only on subsampled points as the others
    // have no normal
    new_map.update(points, normals, normal_scores);
  }

  /// Now perform a dynamic object removal
  //
  vtr::lidar::FrustumGrid frustum_grid(config_->horizontal_resolution,
                                       config_->vertical_resolution,
                                       *qdata.undistorted_pointcloud);
  // convert to live frame coordinates
  const auto T_s_m = T_m_s.inverse();
  auto map_points = new_map.cloud.pts;  // has to be copied unfortunately.
  auto map_normals = new_map.normals;
  PCEigen map_pts_mat((float *)map_points.data(), 3, map_points.size());
  PCEigen map_norms_mat((float *)map_normals.data(), 3, map_normals.size());
  R_tot = T_s_m.block<3, 3>(0, 0);
  T_tot = T_s_m.block<3, 1>(0, 3);
  map_pts_mat = (R_tot * map_pts_mat).colwise() + T_tot;
  map_norms_mat = R_tot * map_norms_mat;
  // and then to polar coordinates
  vtr::lidar::cart2pol_(map_points);
  // check if the point is closer
  size_t i = 0;
  for (const auto &p : map_points) {
    const auto k = frustum_grid.getKey(p);
    if (frustum_grid.find(k)) {
      /// \todo also check normals
      new_map.movabilities[i].first += frustum_grid.isCloser(k, p.x);
      new_map.movabilities[i].second++;
    }
    i++;
  }
}

void MapMaintenanceModule::visualizeImpl(QueryCache &qdata, MapCache &,
                                         const Graph::ConstPtr &,
                                         std::mutex &) {
  if (!config_->visualize) return;

  /// Visualize map points
  if (!map_pub_)
    map_pub_ =
        qdata.node->create_publisher<PointCloudMsg>("new_map_points", 20);

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  if (qdata.new_map) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    auto pcitr = (*qdata.new_map).cloud.pts.begin();
    auto ititr = (*qdata.new_map).movabilities.begin();
    for (; pcitr != (*qdata.new_map).cloud.pts.end(); pcitr++, ititr++) {
      // remove recent points
      if (ititr->second <= 5) continue;
      pcl::PointXYZI pt;
      pt.x = pcitr->x;
      pt.y = pcitr->y;
      pt.z = pcitr->z;
      pt.intensity = (ititr->first / ititr->second) > 0.5 ? 1 : 0;
      cloud.points.push_back(pt);
    }
    pcl::toROSMsg(cloud, *pc2_msg);
    pc2_msg->header.frame_id = "odometry keyframe";
    pc2_msg->header.stamp = *qdata.rcl_stamp;

    map_pub_->publish(*pc2_msg);
  }

  /// Visualize aligned points
  if (!pc_pub_)
    pc_pub_ = qdata.node->create_publisher<PointCloudMsg>("aligned_points", 20);

  auto &T_s_r = *qdata.T_s_r;
  auto &T_r_m = *qdata.T_r_m_odo;
  auto points = *qdata.undistorted_pointcloud;
  // Transform subsampled points into the map frame
  auto T_m_s = (T_r_m.inverse() * T_s_r.inverse()).matrix();
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> pts_mat(
      (float *)points.data(), 3, points.size());
  Eigen::Matrix3f R_tot = (T_m_s.block(0, 0, 3, 3)).cast<float>();
  Eigen::Vector3f T_tot = (T_m_s.block(0, 3, 3, 1)).cast<float>();
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;

  pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  for (auto pt : points)
    cloud2.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
  pcl::toROSMsg(cloud2, *pc2_msg);
  pc2_msg->header.frame_id = "odometry keyframe";
  pc2_msg->header.stamp = *qdata.rcl_stamp;

  pc_pub_->publish(*pc2_msg);
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr