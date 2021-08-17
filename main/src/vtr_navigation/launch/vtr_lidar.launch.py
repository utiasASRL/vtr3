import os
import os.path as osp

import launch
import launch.actions
import launch.substitutions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  vtr_navigation = get_package_share_directory('vtr_navigation')
  # configs
  base_config = osp.join(vtr_navigation, 'config/lidar/base/base.yaml')
  scenario_config = osp.join(vtr_navigation, 'config/lidar/scenario')

  # path-tracker config
  vtr_path_tracker = get_package_share_directory('vtr_path_tracker')
  pt_config = osp.join(vtr_path_tracker, 'config/lidar/grizzly')
  grizzly_path_tracker_config = [osp.join(pt_config, "params.yaml")]
  grizzly_path_tracker_gains_config = [osp.join(pt_config, "gains.yaml")]

  return LaunchDescription([
      DeclareLaunchArgument('data_dir', description='Data directory'),
      DeclareLaunchArgument('scenario_params',
                            description='Run and data params'),
      DeclareLaunchArgument('clear_data_dir',
                            default_value='false',
                            description='Clear the data dir before launch VTR'),
      DeclareLaunchArgument('use_sim_time',
                            default_value='false',
                            description='Use simulated time for playback'),
      Node(
          package='vtr_navigation',
          namespace='vtr',
          executable='vtr_navigation',
          output='screen',
          # prefix=['xterm -e gdb --args'],
          parameters=[
              {
                  "data_dir": LaunchConfiguration("data_dir"),
                  "clear_data_dir": LaunchConfiguration("clear_data_dir"),
                  "use_sim_time": LaunchConfiguration("use_sim_time"),
              },
              # base_config
              base_config,
              # path-tracker config
              *grizzly_path_tracker_config,
              *grizzly_path_tracker_gains_config,
              # scenario specific configs
              PathJoinSubstitution(
                  (scenario_config, LaunchConfiguration("scenario_params")))
          ])
  ])
