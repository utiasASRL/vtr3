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
      DeclareLaunchArgument('start_new_graph',
                            default_value='false',
                            description='Starts a new pose graph'),
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
      DeclareLaunchArgument(
          'input_dir',
          default_value='/tmp',
          description='ROS bag directory that contains sensor data'),
      DeclareLaunchArgument(
          'stop_frame_idx',
          default_value='1000000',
          description='Frame index to stop at when not listening to topic'),
      DeclareLaunchArgument(
          'listen_to_topic',
          default_value='false',
          description='Listen to ROS topic for acquiring sensor data'),
      Node(
          package='vtr_testing_lidar',
          namespace='vtr',
          executable='vtr_testing_lidar_localization',
          output='screen',
          # prefix=['xterm -e gdb --args'],
          parameters=[
              {
                  "data_dir": LaunchConfiguration("data_dir"),
                  "start_new_graph": LaunchConfiguration("start_new_graph"),
                  "use_sim_time": LaunchConfiguration("use_sim_time"),
                  "input_dir": LaunchConfiguration("input_dir"),
                  "stop_frame_idx": LaunchConfiguration("stop_frame_idx"),
                  "listen_to_topic": LaunchConfiguration("listen_to_topic"),
              },
              PathJoinSubstitution(
                  (config_dir, LaunchConfiguration("base_params"))),
              PathJoinSubstitution(
                  (config_dir, LaunchConfiguration("override_params"))),
          ],
      )
  ])
