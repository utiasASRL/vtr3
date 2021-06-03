from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
      Node(package='vtr_lidar',
           executable='vtr_lidar_slam',
           name='point_slam',
           output='screen',
           parameters=[{
               "verbose": 1,
               "odom_frame": "odom",
               "base_frame": "base_link",
               "map_topic": "map",
               "map_frame": "map",
               "filter": False,
               "gt_classify": True,
               "map_voxel_size": 0.03,
               "frame_voxel_size": 0.10,
               "map2d_pixel_size": 0.05,
               "map2d_max_count": 50,
               "map2d_z_min": 0.2,
               "map2d_z_max": 1.2,
               "motion_distortion": False,
               "lidar_n_lines": 64,
               "init_map_day": "2020-10-02-13-39-05",
               "init_map_ind": 1,
               "h_scale": 0.5,
               "r_scale": 4.0,
               "outl_rjct_passes": 2,
               "outl_rjct_thresh": 0.003,
               "icp_samples": 1000,
               "icp_pairing_dist": 2.0,
               "icp_planar_dist": 0.3,
               "icp_avg_steps": 5,
               "icp_max_iter": 100,
           }])
  ])
