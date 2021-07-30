#include <vtr_tactic/modules/lidar/map_maintenance_module.hpp>

namespace {
PointCloudMsg::SharedPtr toROSMsg(const std::vector<PointXYZ> &points,
                                  const std::vector<float> &intensities,
                                  const std::string &frame_id,
                                  const rclcpp::Time &timestamp) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  auto pcitr = points.begin();
  auto ititr = intensities.begin();
  for (; pcitr != points.end(); pcitr++, ititr++) {
    pcl::PointXYZI pt;
    pt.x = pcitr->x;
    pt.y = pcitr->y;
    pt.z = pcitr->z;
    pt.intensity = *ititr;
    cloud.points.push_back(pt);
  }

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::toROSMsg(cloud, *pc2_msg);
  pc2_msg->header.frame_id = frame_id;
  pc2_msg->header.stamp = timestamp;
  return pc2_msg;
}
}  // namespace

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

  config_->min_num_observations = node->declare_parameter<int>(param_prefix + ".min_num_observations", config_->min_num_observations);
  config_->max_num_observations = node->declare_parameter<int>(param_prefix + ".max_num_observations", config_->max_num_observations);

  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void MapMaintenanceModule::runImpl(QueryCache &qdata, MapCache &,
                                   const Graph::ConstPtr &) {
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    aligned_points_pub_ = qdata.node->create_publisher<PointCloudMsg>("aligned_points", 5);
    movability_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("new_map_pts_mvblty", 5);
    movability_obs_map_pub_ = qdata.node->create_publisher<PointCloudMsg>("new_map_pts_mvblty_obs", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Do not update the map if registration failed.
  if (!(*qdata.odo_success)) {
    CLOG(WARNING, "lidar.map_maintenance")
        << "Point cloud registration failed - not updating the map.";
    return;
  }

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

  // Transform subsampled points into the map frame
  auto T_m_s = (T_r_m.inverse() * T_s_r.inverse()).matrix().cast<float>();
  using PCEigen = Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>>;
  PCEigen pts_mat((float *)points.data(), 3, points.size());
  PCEigen norms_mat((float *)normals.data(), 3, normals.size());
  Eigen::Matrix3f R_tot = T_m_s.block<3, 3>(0, 0);
  Eigen::Vector3f T_tot = T_m_s.block<3, 1>(0, 3);
  pts_mat = (R_tot * pts_mat).colwise() + T_tot;
  norms_mat = R_tot * norms_mat;
  // Update the point map with the set of new points (also initializes
  // movabilities)
  if (qdata.current_map_odo) {
    const auto &old_map = *qdata.current_map_odo;
    const auto T_old_new = (*qdata.current_map_odo_T_v_m).inverse();
    vtr::lidar::IncrementalPointMapMigrator map_migrator(T_old_new.matrix(),
                                                         old_map, new_map);
    map_migrator.update(
        points, normals, normal_scores,
        std::vector<std::pair<int, int>>(points.size(), {0, 0}));
  } else {
    // The update function is called only on subsampled points as the others
    // have no normal
    new_map.update(points, normals, normal_scores,
                   std::vector<std::pair<int, int>>(points.size(), {0, 0}));
  }

  /// Now perform a dynamic object detection
  // create a frustum grid from the undistorted pointcloud (current frame)
  vtr::lidar::FrustumGrid frustum_grid(config_->horizontal_resolution,
                                       config_->vertical_resolution,
                                       *qdata.undistorted_pointcloud);
  // convert our map to live frame coordinates
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
  auto map_points_polar = map_points;
  vtr::lidar::cart2Pol_(map_points_polar);

  // check if the point is closer
  for (size_t i = 0; i < map_points_polar.size(); i++) {
    const auto &p = map_points_polar[i];
    const auto k = frustum_grid.getKey(p);
    // check if we have this point in this scan
    if (!frustum_grid.find(k)) continue;

    // the current point is occluded in the current observation
    if (frustum_grid.isFarther(k, p.x)) continue;

    // update this point only when we have a good normal
    float angle =
        acos(std::min(abs(map_points[i].dot(map_normals[i]) / p.x), 1.0f));
    if (angle > 5 * M_PI / 12) continue;

    // minimum number of observations to be considered static point
    if (new_map.movabilities[i].second < config_->min_num_observations) {
      new_map.movabilities[i].first++;
      new_map.movabilities[i].second++;
      continue;
    }

    if (new_map.movabilities[i].second < config_->max_num_observations) {
      new_map.movabilities[i].first += frustum_grid.isCloser(k, p.x);
      new_map.movabilities[i].second++;
    } else {
      if (frustum_grid.isCloser(k, p.x)) {
        new_map.movabilities[i].first = std::min(
            config_->max_num_observations, new_map.movabilities[i].first + 1);
      } else {
        new_map.movabilities[i].first =
            std::max(0, new_map.movabilities[i].first - 1);
      }
    }
  }

  if (config_->visualize) {
    {
      // publish map and number of observations (movability) of each point
      auto pc2_msg = std::make_shared<PointCloudMsg>();
      pcl::PointCloud<pcl::PointXYZI> cloud;
      auto pcitr = new_map.cloud.pts.begin();
      auto ititr = new_map.movabilities.begin();
      for (; pcitr != new_map.cloud.pts.end(); pcitr++, ititr++) {
        // remove points with bad normal (likely never gets updated)
        if (ititr->second == 0) continue;
        pcl::PointXYZI pt;
        pt.x = pcitr->x;
        pt.y = pcitr->y;
        pt.z = pcitr->z;
        pt.intensity = ititr->second;
        cloud.points.push_back(pt);
      }
      pcl::toROSMsg(cloud, *pc2_msg);
      pc2_msg->header.frame_id = "odometry keyframe";
      pc2_msg->header.stamp = *qdata.rcl_stamp;

      movability_obs_map_pub_->publish(*pc2_msg);
    }
    {
      // publish map and movability of each point
      auto pc2_msg = std::make_shared<PointCloudMsg>();
      pcl::PointCloud<pcl::PointXYZI> cloud;
      auto pcitr = new_map.cloud.pts.begin();
      auto ititr = new_map.movabilities.begin();
      for (; pcitr != new_map.cloud.pts.end(); pcitr++, ititr++) {
        // remove recent points
        if (ititr->second <= 1) continue;
        // remove points that are for sure dynamic
        if (ititr->second >= config_->max_num_observations &&
            ((float)ititr->first / (float)ititr->second) > 0.5)
          continue;
        pcl::PointXYZI pt;
        pt.x = pcitr->x;
        pt.y = pcitr->y;
        pt.z = pcitr->z;
        pt.intensity =
            ((float)ititr->first / (float)ititr->second) > 0.5 ? 1 : 0;
        cloud.points.push_back(pt);
      }
      pcl::toROSMsg(cloud, *pc2_msg);
      pc2_msg->header.frame_id = "odometry keyframe";
      pc2_msg->header.stamp = *qdata.rcl_stamp;

      movability_map_pub_->publish(*pc2_msg);
    }
    {
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

      auto pc2_msg = std::make_shared<PointCloudMsg>();
      pcl::PointCloud<pcl::PointXYZ> cloud2;
      for (auto pt : points)
        cloud2.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
      pcl::toROSMsg(cloud2, *pc2_msg);
      pc2_msg->header.frame_id = "odometry keyframe";
      pc2_msg->header.stamp = *qdata.rcl_stamp;

      aligned_points_pub_->publish(*pc2_msg);
    }
  }
}

void MapMaintenanceModule::visualizeImpl(QueryCache &qdata, MapCache &,
                                         const Graph::ConstPtr &,
                                         std::mutex &) {}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr