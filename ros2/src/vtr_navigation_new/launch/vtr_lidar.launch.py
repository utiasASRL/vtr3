import os

osp = os.path

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  vtr_navigation = get_package_share_directory('vtr_navigation_new')
  # base configs
  base_config = osp.join(vtr_navigation, 'config/lidar.yaml')

  return LaunchDescription([
      Node(
          package='vtr_navigation_new',
          namespace='vtr',
          executable='vtr_navigation_new',
          output='screen',
          # prefix=['xterm -e gdb -ex=r --args'],
          parameters=[base_config])
  ])
