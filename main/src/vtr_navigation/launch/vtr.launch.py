import os
import os.path as osp

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import Node


def test(ar):
  print([ar])
  return 1

def generate_launch_description():
    # config directory
    config_dir = osp.join(os.getenv('VTRSRC'), 'config')
    # temp directory
    temp_dir = os.getenv('VTRTEMP')

    commonNodeArgs = {
        "package": 'vtr_navigation',
        "namespace": 'vtr',
        "executable": 'vtr_navigation',
        "output": 'screen',
        #prefix=['xterm -e gdb -ex run --args'],
    }

    return LaunchDescription([
        DeclareLaunchArgument('data_dir', default_value='', description='directory to store graph, if blank use setup UI'),
        DeclareLaunchArgument('start_new_graph', default_value='false', description='whether to start a new pose graph'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='use simulated time for playback'),
        DeclareLaunchArgument('base_params', description='base parameter file (sensor, robot specific)'),
        DeclareLaunchArgument('override_params', default_value='', description='scenario specific parameter overrides'),
        Node(**commonNodeArgs,
            parameters=[
                PathJoinSubstitution((config_dir, LaunchConfiguration("base_params"))),
                {
                    "data_dir": LaunchConfiguration("data_dir"),
                    "start_new_graph": LaunchConfiguration("start_new_graph"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
                PathJoinSubstitution((config_dir, LaunchConfiguration("override_params")))
            ],
            condition=LaunchConfigurationNotEquals('data_dir', '')
        ),
        Node(**commonNodeArgs,
            parameters=[
                PathJoinSubstitution((config_dir, LaunchConfiguration("base_params"))),
                PathJoinSubstitution((temp_dir, "setup_params.yaml")),
                {
                    "start_new_graph": LaunchConfiguration("start_new_graph"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
                PathJoinSubstitution((config_dir, LaunchConfiguration("override_params")))
            ],
            condition=LaunchConfigurationEquals('data_dir', '')
        ),
    ])
