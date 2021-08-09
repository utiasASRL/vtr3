import os
import os.path as osp

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  vtr_safety_monitor = get_package_share_directory('vtr_safety_monitor')
  scenario_config = osp.join(vtr_safety_monitor, 'config')

  return LaunchDescription([
      DeclareLaunchArgument('scenario_params', description='Custom parameters'),
      Node(
          package='vtr_safety_monitor',
          executable='safety_monitor_node',
          remappings=[("/vtr/safe_command",
                       "/grizzly_velocity_controller/cmd_vel")],
          output='screen',
          name='safety_monitor',
          namespace='vtr',
          parameters=[
              # for now either online or playback
              PathJoinSubstitution(
                  (scenario_config, LaunchConfiguration("scenario_params")))
          ],
      )
  ])
