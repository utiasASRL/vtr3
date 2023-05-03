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
      DeclareLaunchArgument('data_dir', description='data directory (store log files and result pose graph)'),
      DeclareLaunchArgument('model_dir', description='model directory (folder for PyTorch .pt models)'),
      DeclareLaunchArgument('start_new_graph', default_value='false', description='whether to start a new pose graph'),
      DeclareLaunchArgument('use_sim_time', default_value='false', description='use simulated time for playback'),
      DeclareLaunchArgument('base_params', description='base parameter file (sensor, robot specific)'),
      DeclareLaunchArgument('override_params', default_value='', description='scenario specific parameter overrides'),
      Node(
          package='vtr_navigation',
          namespace='vtr',
          executable='vtr_navigation',
          output='screen',
          # prefix=['xterm -e gdb --args'],
          parameters=[
              {
                  "data_dir": LaunchConfiguration("data_dir"),
                  "model_dir": LaunchConfiguration("model_dir"),
                  "start_new_graph": LaunchConfiguration("start_new_graph"),
                  "use_sim_time": LaunchConfiguration("use_sim_time"),
              },
              PathJoinSubstitution((config_dir, LaunchConfiguration("base_params"))),
              PathJoinSubstitution((config_dir, LaunchConfiguration("override_params"))),
          ],
      ),
  ])
