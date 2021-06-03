#include <vtr_tactic/modules/lidar/pcl_preprocessing_module.hpp>

namespace vtr {
namespace tactic {

void PCLPreprocessingModule::runImpl(QueryCache &qdata, MapCache &,
                                     const Graph::ConstPtr &) {
  // Get input and output data
  // input
  auto &f_pts = *qdata.raw_pointcloud;
  // output
  /// \todo (yuchen) see bottom

  // Create a copy of points in polar coordinates
  std::vector<PointXYZ> polar_pts(f_pts);
  cart2pol_(polar_pts);

  //////////////////////

  // Get lidar angle resolution
  float minTheta, maxTheta;
  float lidar_angle_res = get_lidar_angle_res(polar_pts, minTheta, maxTheta,
                                              config_->lidar_n_lines);

  LOG(DEBUG) << "minTheta is : " << minTheta;
  LOG(DEBUG) << "maxTheta is : " << maxTheta;
  LOG(DEBUG) << "lidar_angle_res is : " << lidar_angle_res;

  // Define the polar neighbors radius in the scaled polar coordinates
  float polar_r = config_->polar_r_scale * lidar_angle_res;  // 1.5

  // Apply log scale to radius coordinate (in place)
  lidar_log_radius(polar_pts, polar_r, config_->r_scale);

  //////////////////////

  // Get subsampling of the frame in carthesian coordinates
  std::vector<PointXYZ> sub_pts;
  std::vector<size_t> sub_inds;
  grid_subsampling_centers(f_pts, sub_pts, sub_inds, config_->frame_voxel_size);

  /// \todo (yuchen) find a place to publish points

  //////////////////////

  // Convert sub_pts to polar and rescale
  std::vector<PointXYZ> polar_queries0(sub_pts);
  cart2pol_(polar_queries0);
  std::vector<PointXYZ> polar_queries(polar_queries0);
  lidar_log_radius(polar_queries, polar_r, config_->r_scale);
  lidar_horizontal_scale(polar_queries, config_->h_scale);

  // Init result containers
  vector<PointXYZ> normals;
  vector<float> norm_scores;

  // Apply horizontal scaling (to have smaller neighborhoods in horizontal
  // direction)
  lidar_horizontal_scale(polar_pts, config_->h_scale);

  // Call polar processing function
  extract_lidar_frame_normals(f_pts, polar_pts, sub_pts, polar_queries, normals,
                              norm_scores, polar_r);

  //////////////////////

  // Better normal score based on distance and incidence angle
  std::vector<float> icp_scores(norm_scores);
  smart_icp_score(polar_queries0, icp_scores);
  smart_normal_score(sub_pts, polar_queries0, normals, norm_scores);

  // (yuchen) filter out all close points (reflected from the robot)
  for (unsigned i = 0; i < norm_scores.size(); i++) {
    if (polar_queries0[i].x < 1.2)
      norm_scores[i] = 0.0;  // (1.2 is the height of the robot)
  }

  // Remove points with a low score
  float min_score = 0.01;
  filter_pointcloud(sub_pts, norm_scores, min_score);
  filter_pointcloud(normals, norm_scores, min_score);
  filter_floatvector(icp_scores, norm_scores, min_score);
  filter_floatvector(norm_scores, min_score);

  // output
  qdata.preprocessed_pointcloud.fallback(sub_pts);
  qdata.normals.fallback(normals);
  qdata.normal_scores.fallback(norm_scores);
  qdata.icp_scores.fallback(icp_scores);

  LOG(INFO) << "Raw point size: " << f_pts.size();
  LOG(INFO) << "Subsampled point size: " << sub_pts.size();
}

void PCLPreprocessingModule::visualizeImpl(QueryCache &qdata, MapCache &,
                                           const Graph::ConstPtr &,
                                           std::mutex &) {
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

}  // namespace tactic
}  // namespace vtr