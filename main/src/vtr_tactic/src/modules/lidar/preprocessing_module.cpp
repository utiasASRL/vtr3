#include <vtr_tactic/modules/lidar/preprocessing_module.hpp>

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
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void PreprocessingModule::runImpl(QueryCache &qdata, MapCache &,
                                  const Graph::ConstPtr &) {
  // Get input point cloud
  auto &points = *qdata.raw_pointcloud;

  // Create a copy of points in polar coordinates
  std::vector<PointXYZ> polar_points(points);
  cart2pol_(polar_points);

  // Get lidar angle resolution
  // float minTheta, maxTheta;
  // float vertical_angle_res = get_lidar_angle_res(
  //     polar_points, minTheta, maxTheta, config_->num_channels);
  // LOG(DEBUG) << "minTheta is : " << minTheta << ", maxTheta is : "
  //            << maxTheta;
  // LOG(DEBUG) << "vertical_angle_res is : " << vertical_angle_res;
  auto vertical_angle_res = config_->vertical_angle_res;

  // Define the polar neighbors radius in the scaled polar coordinates
  float polar_r = config_->polar_r_scale * vertical_angle_res;  // 1.5
  // Apply log scale to radius coordinate (in place)
  lidar_log_radius(polar_points, polar_r, config_->r_scale);
  // Apply horizontal scaling (smaller neighborhoods in horizontal direction)
  lidar_horizontal_scale(polar_points, config_->h_scale);

  // Get subsampling of the frame in carthesian coordinates
  std::vector<PointXYZ> sampled_points;
  std::vector<size_t> sampled_inds;
  grid_subsampling_centers(points, sampled_points, sampled_inds,
                           config_->frame_voxel_size);

  // Convert sampled_points to polar and rescale
  std::vector<PointXYZ> sampled_polar_points0(sampled_points);
  cart2pol_(sampled_polar_points0);
  std::vector<PointXYZ> sampled_polar_points(sampled_polar_points0);
  lidar_log_radius(sampled_polar_points, polar_r, config_->r_scale);
  lidar_horizontal_scale(sampled_polar_points, config_->h_scale);

  // Extract normal vectors of sampled points
  vector<PointXYZ> normals;
  vector<float> norm_scores;
  extract_lidar_frame_normals(points, polar_points, sampled_points,
                              sampled_polar_points, normals, norm_scores,
                              polar_r, config_->num_threads);

  // Better normal score based on distance and incidence angle
  std::vector<float> icp_scores(norm_scores);
  smart_icp_score(sampled_polar_points0, icp_scores);
  smart_normal_score(sampled_points, sampled_polar_points0, normals,
                     norm_scores);
  // Optionally, filter out all close points (reflected from the robot)
  for (unsigned i = 0; i < norm_scores.size(); i++) {
    if (sampled_polar_points0[i].x < 1.2)
      norm_scores[i] = 0.0;  // (1.2 is the height of the robot)
  }

  // Remove points with a low score
  float min_score = 0.01;
  filter_pointcloud(sampled_points, norm_scores, min_score);
  filter_pointcloud(normals, norm_scores, min_score);
  filter_floatvector(icp_scores, norm_scores, min_score);
  filter_floatvector(norm_scores, min_score);

  // Output
  qdata.preprocessed_pointcloud.fallback(sampled_points);
  qdata.normals.fallback(normals);
  qdata.normal_scores.fallback(norm_scores);
  qdata.icp_scores.fallback(icp_scores);

  LOG(DEBUG) << "Raw point size: " << points.size();
  LOG(DEBUG) << "Subsampled point size: " << sampled_points.size();
}

void PreprocessingModule::visualizeImpl(QueryCache &qdata, MapCache &,
                                        const Graph::ConstPtr &, std::mutex &) {
  if (!config_->visualize) return;

  if (!pc_pub_)
    pc_pub_ = qdata.node->create_publisher<PointCloudMsg>("sampled_points", 20);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (auto pt : *qdata.preprocessed_pointcloud)
    cloud.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::toROSMsg(cloud, *pc2_msg);
  pc2_msg->header.frame_id = "velodyne";
  pc2_msg->header.stamp = *qdata.rcl_stamp;

  pc_pub_->publish(*pc2_msg);
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr