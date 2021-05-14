import os

osp = os.path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  vtr_testing_lidar = get_package_share_directory('vtr_testing_lidar')
  # base configs
  base_config = osp.join(vtr_testing_lidar, 'config')

  return LaunchDescription([
      DeclareLaunchArgument('params', description='Run and data params'),
      Node(
          package='vtr_testing_lidar',
          namespace='vtr',
          executable='vtr_testing_lidar_localization',
          # name='lidar_test',
          output='screen',
          # prefix=['xterm -e gdb --args'],
          parameters=[
              PathJoinSubstitution((base_config, LaunchConfiguration("params")))
          ])
  ])
