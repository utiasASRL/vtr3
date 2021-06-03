import os

osp = os.path

import launch
import launch.actions
import launch.substitutions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  vtr_testing_lidar = get_package_share_directory('vtr_testing_lidar')
  # base configs
  base_config = osp.join(vtr_testing_lidar, 'config')

  return LaunchDescription([
      DeclareLaunchArgument('data_dir', description='Data directory'),
      DeclareLaunchArgument('params', description='Run and data params'),
      DeclareLaunchArgument('clear_data_dir',
                            default_value='false',
                            description='Clear the data dir before launch VTR'),
      Node(
          package='vtr_testing_lidar',
          namespace='vtr',
          executable='vtr_testing_lidar_localization',
          output='screen',
          # prefix=['xterm -e gdb --args'],
          parameters=[
              {
                  "data_dir": LaunchConfiguration("data_dir"),
                  "clear_data_dir": LaunchConfiguration("clear_data_dir"),
                  #   "use_sim_time": LaunchConfiguration("use_sim_time"),
              },
              # configs
              PathJoinSubstitution((base_config, LaunchConfiguration("params")))
          ])
  ])
