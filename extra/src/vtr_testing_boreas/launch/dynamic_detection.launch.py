import os
import os.path as osp

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  # config directory
  config_dir = osp.join(get_package_share_directory('vtr_testing_boreas'),
                        'config')

  return LaunchDescription([
      DeclareLaunchArgument('data_dir', description='Data directory'),
      DeclareLaunchArgument(
          'base_params',
          description='Base parameter file (sensor, robot specific)'),
      Node(
          package='vtr_testing_boreas',
          namespace='vtr',
          executable='vtr_testing_boreas_dynamic_detection',
          output='screen',
          # prefix=['xterm -e gdb --args'],
          parameters=[
              {
                  "data_dir": LaunchConfiguration("data_dir"),
              },
              PathJoinSubstitution(
                  (config_dir, LaunchConfiguration("base_params"))),
          ],
      )
  ])
