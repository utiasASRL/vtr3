#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "vtr_lidar/point_map_slam.h"

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_map_slam");

  ////////////////////////
  // Init Pointmap SLAM //
  ////////////////////////

  // Parameters initialized with default values
  SLAM_params slam_params;
  ICP_params icp_params;

  // Load params values
  // clang-format off
  slam_params.map_voxel_size = node->declare_parameter<float>("map_voxel_size", slam_params.map_voxel_size);
  slam_params.frame_voxel_size = node->declare_parameter<float>("frame_voxel_size", slam_params.frame_voxel_size);
  slam_params.map2d_pixel_size = node->declare_parameter<float>("map2d_pixel_size", slam_params.map2d_pixel_size);
  slam_params.map2d_max_count = node->declare_parameter<int>("map2d_max_count", slam_params.map2d_max_count);
  slam_params.map2d_zMin = node->declare_parameter<float>("map2d_zMin", slam_params.map2d_zMin);
  slam_params.map2d_zMax = node->declare_parameter<float>("map2d_zMax", slam_params.map2d_zMax);
  slam_params.lidar_n_lines = node->declare_parameter<int>("lidar_n_lines", slam_params.lidar_n_lines);
  slam_params.motion_distortion = node->declare_parameter<bool>("motion_distortion", slam_params.motion_distortion);
  slam_params.h_scale = node->declare_parameter<float>("h_scale", slam_params.h_scale);
  slam_params.r_scale = node->declare_parameter<float>("r_scale", slam_params.r_scale);
  slam_params.outl_rjct_passes = node->declare_parameter<int>("outl_rjct_passes", slam_params.outl_rjct_passes);
  slam_params.outl_rjct_thresh = node->declare_parameter<float>("outl_rjct_thresh", slam_params.outl_rjct_thresh);

  slam_params.odom_frame = node->declare_parameter<string>("odom_frame", slam_params.odom_frame);
  slam_params.map_frame = node->declare_parameter<string>("map_frame", slam_params.map_frame);
  slam_params.base_frame = node->declare_parameter<string>("base_frame", slam_params.base_frame);
  slam_params.filtering = node->declare_parameter<bool>("filter", slam_params.filtering);
  slam_params.gt_filter = node->declare_parameter<bool>("gt_classify", slam_params.gt_filter);

  icp_params.n_samples = (size_t)node->declare_parameter<int>("icp_samples", (size_t)icp_params.n_samples);
  icp_params.max_pairing_dist = node->declare_parameter<float>("icp_pairing_dist", icp_params.max_pairing_dist);
  icp_params.max_planar_dist = node->declare_parameter<float>("icp_planar_dist", icp_params.max_planar_dist);
  icp_params.avg_steps = (size_t)node->declare_parameter<int>("icp_avg_steps", (size_t)icp_params.avg_steps);
  icp_params.max_iter = (size_t)node->declare_parameter<int>("icp_max_iter", (size_t)icp_params.max_iter);
  slam_params.icp_params = icp_params;

  auto start_time = node->declare_parameter<string>("start_time", "");
  slam_params.log_path = fs::path{"/tmp/"} / "logs-" / start_time;
  // clang-format on

  cout << slam_params;  // dump parameters

  /////////////////////
  // Get initial map //
  /////////////////////

  vector<PointXYZ> init_pts;
  vector<PointXYZ> init_normals;
  vector<float> init_scores;

  ///////////////////////
  // Init mapper class //
  ///////////////////////

  // Create a the SLAM class
  PointMapSLAM mapper(node, slam_params, init_pts, init_normals, init_scores);

  ///////////////////////
  // Start subscribing //
  ///////////////////////

  auto lidar_sub = node->create_subscription<PointCloudMsg>(
      "raw_points", 1,
      std::bind(&PointMapSLAM::gotCloud, &mapper, std::placeholders::_1));

  std::cout << "Hello world!" << std::endl;

  // Wait for shutdown
  rclcpp::spin(node);
  rclcpp::shutdown();
}