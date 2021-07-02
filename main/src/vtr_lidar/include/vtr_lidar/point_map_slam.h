#if false
#pragma once

#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>

#include "vtr_lidar/grid_subsampling/grid_subsampling.h"
#include "vtr_lidar/icp/icp.h"
#include "vtr_lidar/pointmap/pointmap.h"
#include "vtr_lidar/polar_processing/polar_processing.h"

using namespace std;
namespace fs = std::filesystem;

using ResultMsg = std_msgs::msg::Bool;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using OccupancyGridMsg = nav_msgs::msg::OccupancyGrid;
using MapMetaDataMsg = nav_msgs::msg::MapMetaData;

// Utilities
// *********

Plane3D extract_ground(vector<PointXYZ> &points, vector<PointXYZ> &normals,
                       float angle_vertical_thresh = M_PI / 6,
                       float dist_thresh = 0.1, int max_iter = 200,
                       bool mode_2D = false);

// SLAM params class
// *****************

class SLAM_params {
 public:
  // Elements
  // ********

  // Number of lines of scan in the lidar
  int lidar_n_lines;

  // Size of the map voxels
  float map_voxel_size;

  // Size of the voxels for frame subsampling
  float frame_voxel_size;

  // Account for motion distortion (false in the case of simulated data)
  bool motion_distortion;

  // Transformation matrix from velodyne frame to base frame
  Eigen::Matrix4d H_velo_base;

  // Params of ICP used in this SLAM
  ICP_params icp_params;

  // Params of frame normal computation
  float h_scale;
  float r_scale;
  int outl_rjct_passes;
  float outl_rjct_thresh;

  // ROS related
  string odom_frame;
  string map_frame;
  string base_frame;
  bool filtering, gt_filter;
  vector<int> loc_labels;
  int verbose;
  float map2d_pixel_size;
  int map2d_max_count;
  float map2d_zMin, map2d_zMax;
  fs::path log_path;

  // Methods
  // *******

  // Constructor
  SLAM_params() : loc_labels(7) {
    lidar_n_lines = 32;
    map_voxel_size = 0.08;
    frame_voxel_size = 0.16;
    motion_distortion = false;
    H_velo_base = Eigen::Matrix4d::Identity(4, 4);

    h_scale = 0.5;
    r_scale = 4.0;
    outl_rjct_passes = 2;
    outl_rjct_thresh = 0.003;

    odom_frame = "odom";
    map_frame = "map";
    base_frame = "base_link";
    filtering = false;
    gt_filter = true;
    std::iota(loc_labels.begin(), loc_labels.end(), 0);
    map2d_pixel_size = 0.08;
    map2d_max_count = 10;
    map2d_zMin = 0.5;
    map2d_zMax = 1.5;
    verbose = 0;
    log_path = "";
  }
};

std::ostream &operator<<(std::ostream &os, const SLAM_params &s);

// SLAM class
// **********

class PointMapSLAM {
 public:
  // Elements
  // ********

  // Parameters
  SLAM_params params;

  // Map used by the algorithm
  PointMap map;

  // Pose of the last mapped frame
  Eigen::Matrix4d last_H;
  vector<Eigen::Matrix4d> all_H;
  vector<rclcpp::Time> f_times;

  // Current pose correction from odometry to map
  Eigen::Matrix4d H_OdomToMap;

  // Current number of aligned frames
  int n_frames;
#if false
  // ROS parameters
  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;
#endif
  const rclcpp::Node::SharedPtr node_;
  rclcpp::Time stamp_;
  rclcpp::Publisher<OccupancyGridMsg>::SharedPtr sst;
  rclcpp::Publisher<MapMetaDataMsg>::SharedPtr sstm;

  rclcpp::Publisher<PointCloudMsg>::SharedPtr pcd_pub;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub;
  rclcpp::Publisher<ResultMsg>::SharedPtr result_pub;

  OccupGrid2D map2D;

  // Methods
  // *******

  // Constructor
  PointMapSLAM(const rclcpp::Node::SharedPtr node, SLAM_params slam_params0,
               vector<PointXYZ> &init_points, vector<PointXYZ> &init_normals,
               vector<float> &init_scores)
      : node_(node),
        map(slam_params0.map_voxel_size, init_points, init_normals,
            init_scores),
        map2D(slam_params0.map2d_pixel_size, slam_params0.map2d_max_count)
#if false
        ,
        tfListener(ros::Duration(30))
#endif
  {
    // Init paramters
    params = slam_params0;
    n_frames = 0;

    // Dummy first last_H
    last_H = Eigen::Matrix4d::Identity(4, 4);
    H_OdomToMap = Eigen::Matrix4d::Identity(4, 4);

    // Publishers
    sst = node_->create_publisher<OccupancyGridMsg>("map", 1);
    sstm = node_->create_publisher<MapMetaDataMsg>("map_metadata", 1);

    pcd_pub = node_->create_publisher<PointCloudMsg>("sampled_points", 10);
    map_pub = node_->create_publisher<PointCloudMsg>("map_points", 10);
    result_pub = node_->create_publisher<ResultMsg>("result", 1);
  }

  // Mapping methods
  void init_map() { return; }
  void gotCloud(const PointCloudMsg::SharedPtr msg);
#if false
  void add_new_frame(vector<PointXYZ> &f_pts, Eigen::Matrix4d &init_H,
                     int verbose = 0);
#endif
  void publish_2D_map();
  void publish_map();
  void publish_points(const rclcpp::Time &stamp, const vector<PointXYZ> &pts);
#if false
  // Debug method
  void save_trajectory(string &path) {
    // Name of the file
    string filepath = path + "map_traj.ply";

    // Variables
    uint64_t num_poses = all_H.size();

    // Convert all poses in points and quaternions
    vector<double> times;
    vector<Eigen::Vector3d> trans;
    vector<Eigen::Quaternion<double>> rots;
    times.reserve(num_poses);
    trans.reserve(num_poses);
    rots.reserve(num_poses);
    for (auto &H : all_H) {
      trans.push_back(H.block(0, 3, 3, 1));
      Eigen::Matrix3d R = H.block(0, 0, 3, 3);
      rots.push_back(Eigen::Quaternion<double>(R));
    }
    for (auto &t : f_times)
      times.push_back(t.toSec());

    // Open file
    npm::PLYFileOut file(filepath);

    // Push fields
    file.pushField(num_poses, 1, npm::PLY_DOUBLE, {"time"}, times);
    file.pushField(num_poses, 3, npm::PLY_DOUBLE, {"pos_x", "pos_y", "pos_z"},
                   trans);
    file.pushField(num_poses, 4, npm::PLY_DOUBLE,
                   {"rot_x", "rot_y", "rot_z", "rot_w"}, rots);
    file.write();
  }
#endif
};
#endif