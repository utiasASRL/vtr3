import os
import os.path as osp

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  # config directory
  config_dir = osp.join(os.getenv('VTRSRC'), 'config')

  return LaunchDescription([
      DeclareLaunchArgument('data_dir', description='Data directory'),
      DeclareLaunchArgument('clear_data_dir',
                            default_value='false',
                            description='Clear the data dir before launch VTR'),
      DeclareLaunchArgument('use_sim_time',
                            default_value='false',
                            description='Use simulated time for playback'),
      DeclareLaunchArgument(
          'base_params',
          description='Base parameter file (sensor, robot specific)'),
      DeclareLaunchArgument(
          'override_params',
          default_value='',
          description='Override parameter file (scenario specific)'),
      Node(
          package='vtr_navigation',
          namespace='vtr',
          executable='vtr_navigation',
          output='screen',
          #   prefix=['xterm -e gdb --args'],
          parameters=[
              {
                  "data_dir": LaunchConfiguration("data_dir"),
                  "clear_data_dir": LaunchConfiguration("clear_data_dir"),
                  "use_sim_time": LaunchConfiguration("use_sim_time"),
              },
              PathJoinSubstitution(
                  (config_dir, LaunchConfiguration("base_params"))),
              PathJoinSubstitution(
                  (config_dir, LaunchConfiguration("override_params"))),
          ],
      ),
  ])
