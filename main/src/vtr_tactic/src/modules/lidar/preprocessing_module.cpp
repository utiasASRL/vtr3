#include <vtr_tactic/modules/lidar/preprocessing_module.hpp>

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

std::vector<float> getNumberOfNeighbors(const std::vector<PointXYZ> &points,
                                        const float &search_radius) {
  // Squared search radius
  float r2 = search_radius * search_radius;

  // Build KDTree
  PointCloud polar_cloud;
  polar_cloud.pts = points;
  nanoflann::KDTreeSingleIndexAdaptorParams tree_params(10 /* max leaf */);
  auto index = std::make_unique<PointXYZ_KDTree>(3, polar_cloud, tree_params);
  index->buildIndex();

  // Search
  std::vector<float> cluster_point_indices;
  cluster_point_indices.reserve(points.size());
  nanoflann::SearchParams search_params;
  search_params.sorted = false;
  for (size_t i = 0; i < points.size(); i++) {
    // initial guess of neighbors size
    std::vector<std::pair<size_t, float>> inds_dists;
    inds_dists.reserve(10);
    // find neighbors
    float point[3] = {points[i].x, points[i].y, points[i].z};
    size_t num_neighbors =
        index->radiusSearch(point, r2, inds_dists, search_params);
    cluster_point_indices.push_back(num_neighbors);
  }
  return cluster_point_indices;
}

}  // namespace

namespace vtr {
namespace tactic {
namespace lidar {

void PreprocessingModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->num_threads = node->declare_parameter<int>(param_prefix + ".num_threads", config_->num_threads);
#ifdef DETERMINISTIC_VTR
  LOG_IF(config_->num_threads != 1, WARNING) << "Point cloud pre-processor number of threads set to 1 in deterministic mode.";
  config_->num_threads = 1;
#endif
  config_->num_channels = node->declare_parameter<int>(param_prefix + ".num_channels", config_->num_channels);
  config_->vertical_angle_res = node->declare_parameter<float>(param_prefix + ".vertical_angle_res", config_->vertical_angle_res);
  config_->polar_r_scale = node->declare_parameter<float>(param_prefix + ".polar_r_scale", config_->polar_r_scale);
  config_->r_scale = node->declare_parameter<float>(param_prefix + ".r_scale", config_->r_scale);
  config_->h_scale = node->declare_parameter<float>(param_prefix + ".h_scale", config_->h_scale);
  config_->frame_voxel_size = node->declare_parameter<float>(param_prefix + ".frame_voxel_size", config_->frame_voxel_size);

  config_->num_sample1 = node->declare_parameter<int>(param_prefix + ".num_sample1", config_->num_sample1);
  config_->min_norm_score1 = node->declare_parameter<float>(param_prefix + ".min_norm_score1", config_->min_norm_score1);

  config_->num_sample2 = node->declare_parameter<int>(param_prefix + ".num_sample2", config_->num_sample2);
  config_->min_norm_score2 = node->declare_parameter<float>(param_prefix + ".min_norm_score2", config_->min_norm_score2);
  config_->ideal_normal_estimate_dist = node->declare_parameter<float>(param_prefix + ".ideal_normal_estimate_dist", config_->ideal_normal_estimate_dist);

  config_->cluster_num_sample = node->declare_parameter<int>(param_prefix + ".cluster_num_sample", config_->cluster_num_sample);

  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void PreprocessingModule::runImpl(QueryCache &qdata, const Graph::ConstPtr &) {
  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    grid_sampled_pub_ = qdata.node->create_publisher<PointCloudMsg>("grid_sampled_points", 5);
    normal_sampled_pub_ = qdata.node->create_publisher<PointCloudMsg>("normal_sampled_points", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  // Get input point cloud
  auto &points = *qdata.raw_pointcloud;
  auto &points_time = *qdata.raw_pointcloud_time;

  CLOG(DEBUG, "lidar.preprocessing")
      << "raw point cloud size: " << points.size();

  /// Create a copy of points in polar coordinates
  std::vector<PointXYZ> polar_points(points);
  vtr::lidar::cart2Pol_(polar_points, true);

  /// Grid subsampling

  // Get subsampling of the frame in carthesian coordinates
  std::vector<PointXYZ> sampled_points;
  std::vector<size_t> sampled_inds;
  vtr::lidar::gridSubsamplingCenters(points, config_->frame_voxel_size,
                                     sampled_points, sampled_inds);

  // Filter polar points and time
  std::vector<PointXYZ> sampled_polar_points;
  std::vector<double> sampled_points_time;
  sampled_polar_points.reserve(sampled_points.size());
  sampled_points_time.reserve(sampled_points.size());
  for (auto &ind : sampled_inds) {
    sampled_polar_points.push_back(polar_points[ind]);
    sampled_points_time.push_back(points_time[ind]);
  }

  CLOG(DEBUG, "lidar.preprocessing")
      << "grid subsampled point cloud size: " << sampled_points.size();

  /// Compute normals and an icp score

  // Scale polar points
  std::vector<PointXYZ> polar_points_scaled(polar_points);
  std::vector<PointXYZ> sampled_polar_points_scaled(sampled_polar_points);

  // Apply scale to radius and angle horizontal
  vtr::lidar::scaleAndLogRadius(polar_points_scaled, config_->r_scale);
  vtr::lidar::scaleHorizontal(polar_points_scaled, config_->h_scale);
  vtr::lidar::scaleAndLogRadius(sampled_polar_points_scaled, config_->r_scale);
  vtr::lidar::scaleHorizontal(sampled_polar_points_scaled, config_->h_scale);

  // Define the polar neighbors radius in the scaled polar coordinates
  float polar_r = config_->polar_r_scale * config_->vertical_angle_res;

  // Extract normal vectors of sampled points
  std::vector<PointXYZ> normals;
  std::vector<float> norm_scores;
  vtr::lidar::extractNormal(points, polar_points_scaled, sampled_points,
                            sampled_polar_points_scaled, polar_r,
                            config_->num_threads, normals, norm_scores);

  // Better normal score based on distance and incidence angle
  std::vector<float> icp_scores(norm_scores);
  vtr::lidar::smartICPScore(sampled_polar_points, icp_scores);

  /// Filtering based on normal scores (planarity)

  // Remove points with a low normal score
  auto sorted_norm_scores = norm_scores;
  std::sort(sorted_norm_scores.begin(), sorted_norm_scores.end());
  float min_score = sorted_norm_scores[std::max(
      0, (int)sorted_norm_scores.size() - config_->num_sample1)];
  min_score = std::max(config_->min_norm_score1, min_score);
  if (min_score >= 0) {
    filterPointCloud(sampled_points, norm_scores, min_score);
    filterPointCloud(normals, norm_scores, min_score);
    filterFloatVector(sampled_points_time, norm_scores, min_score);
    filterFloatVector(icp_scores, norm_scores, min_score);
    filterFloatVector(norm_scores, min_score);
  }

  CLOG(DEBUG, "lidar.preprocessing")
      << "planarity sampled point size: " << sampled_points.size();

  /// Filter based on a normal directions

  vtr::lidar::smartNormalScore(sampled_points, sampled_polar_points, normals,
                               config_->ideal_normal_estimate_dist,
                               norm_scores);

  sorted_norm_scores = norm_scores;
  std::sort(sorted_norm_scores.begin(), sorted_norm_scores.end());
  min_score = sorted_norm_scores[std::max(
      0, (int)sorted_norm_scores.size() - config_->num_sample2)];
  min_score = std::max(config_->min_norm_score2, min_score);
  if (min_score >= 0) {
    filterPointCloud(sampled_points, norm_scores, min_score);
    filterPointCloud(normals, norm_scores, min_score);
    filterFloatVector(sampled_points_time, norm_scores, min_score);
    filterFloatVector(icp_scores, norm_scores, min_score);
    filterFloatVector(norm_scores, min_score);
  }

  CLOG(DEBUG, "lidar.preprocessing")
      << "normal direction sampled point size: " << sampled_points.size();

  /// Remove isolated points (mostly points on trees)

  float search_radius = 2 * config_->frame_voxel_size;
  auto cluster_scores = getNumberOfNeighbors(sampled_points, search_radius);

  auto sorted_cluster_scores = cluster_scores;
  std::sort(sorted_cluster_scores.begin(), sorted_cluster_scores.end());
  min_score = sorted_cluster_scores[std::max(
      0, (int)sorted_cluster_scores.size() - config_->cluster_num_sample)];
  min_score = std::max((float)1, min_score);  /// \todo config
  if (min_score >= 1) {
    filterPointCloud(sampled_points, cluster_scores, min_score);
    filterPointCloud(normals, cluster_scores, min_score);
    filterFloatVector(sampled_points_time, cluster_scores, min_score);
    filterFloatVector(icp_scores, cluster_scores, min_score);
    filterFloatVector(norm_scores, cluster_scores, min_score);
  }
  CLOG(INFO, "lidar.preprocessing")
      << "cluster point size: " << sampled_points.size();

  CLOG(DEBUG, "lidar.preprocessing")
      << "final subsampled point size: " << sampled_points.size();
  if (config_->visualize)
    grid_sampled_pub_->publish(*toROSMsg(sampled_points, norm_scores,
                                         *qdata.lidar_frame, *qdata.rcl_stamp));

  /// Output
  qdata.preprocessed_pointcloud.fallback(sampled_points);
  qdata.preprocessed_pointcloud_time.fallback(sampled_points_time);
  qdata.normals.fallback(normals);
  qdata.normal_scores.fallback(norm_scores);
  qdata.icp_scores.fallback(icp_scores);
}

void PreprocessingModule::visualizeImpl(QueryCache &qdata,
                                        const Graph::ConstPtr &, std::mutex &) {
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr