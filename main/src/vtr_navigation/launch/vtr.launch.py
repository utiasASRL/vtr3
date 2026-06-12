import json
import os
import os.path as osp

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import Node


def _json_to_params(json_path: str) -> dict:
    """Recursively flatten nested JSON into dot-separated ROS parameter dict."""
    with open(json_path, 'r') as f:
        data = json.load(f)

    params: dict = {}

    def _recurse(obj, prefix=''):
        if isinstance(obj, dict):
            for k, v in obj.items():
                _recurse(v, f'{prefix}{k}.' if prefix else f'{k}.')
        elif isinstance(obj, list):
            key = prefix.rstrip('.')
            params[key] = obj
        else:
            key = prefix.rstrip('.')
            params[key] = obj

    _recurse(data)
    return params


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
        # "prefix": 'xterm -e gdb -ex run --args',
        #"prefix": 'valgrind --tool=callgrind',
    }

    # Flatten Ouster sensor JSON into ROS params 
    sensor_json_default = osp.join(config_dir, 'os_warthog.json')
    sensor_params = {}
    if osp.isfile(sensor_json_default):
        sensor_params = _json_to_params(sensor_json_default)

    return LaunchDescription([
        DeclareLaunchArgument('data_dir', default_value='', description='directory to store graph, if blank use setup UI'),
        DeclareLaunchArgument('model_dir', default_value="", description='model directory (folder for PyTorch .pt models)'),
        DeclareLaunchArgument('start_new_graph', default_value='false', description='whether to start a new pose graph'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='use simulated time for playback'),
        DeclareLaunchArgument('planner', default_value='cbit', description='use no planner. Publish zero'),
        DeclareLaunchArgument('base_params', description='base parameter file (sensor, robot specific)'),
        DeclareLaunchArgument('override_params', default_value='', description='scenario specific parameter overrides'),
        DeclareLaunchArgument('sensor_json', default_value=sensor_json_default, description='Path to Ouster sensor metadata JSON'),
        Node(**commonNodeArgs,
            parameters=[
                PathJoinSubstitution((config_dir, LaunchConfiguration("base_params"))),
                sensor_params,
              {
                    "data_dir": LaunchConfiguration("data_dir"),
                    "model_dir": LaunchConfiguration("model_dir"),
                    "start_new_graph": LaunchConfiguration("start_new_graph"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "path_planning.type": LaunchConfiguration("planner"),
              },
                PathJoinSubstitution((config_dir, LaunchConfiguration("override_params")))
            ],
            condition=LaunchConfigurationNotEquals('data_dir', '')
        ),
        Node(**commonNodeArgs,
            parameters=[
                PathJoinSubstitution((config_dir, LaunchConfiguration("base_params"))),
                PathJoinSubstitution((temp_dir, "setup_params.yaml")),
                sensor_params,
                {
                    "model_dir": LaunchConfiguration("model_dir"),
                    "start_new_graph": LaunchConfiguration("start_new_graph"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "path_planning.type": LaunchConfiguration("planner"),

                },
                PathJoinSubstitution((config_dir, LaunchConfiguration("override_params")))
            ],
            condition=LaunchConfigurationEquals('data_dir', '')
        ),
    ])
