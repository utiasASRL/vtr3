from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
import os.path as osp

def generate_launch_description():
    package_name = 'vtr_multi_robot'
    vtr_temp = os.getenv('VTRTEMP')
    config_dir = osp.join(os.getenv('VTRSRC'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('base_params', description='base parameter file (sensor, robot specific)'),
        DeclareLaunchArgument('convoy_params', description='convoy parameter file (sensor, robot specific)'),
        DeclareLaunchArgument('robots', default_value='[\"robot1\",\"robot2\"]', description='list of robot names'),
        Node(
            package='vtr_multi_robot',
            executable='convoy_planning_node',
            name='convoy_planning_node',
            output='screen',
            namespace='vtr',
            parameters=[
                PathJoinSubstitution([vtr_temp, 'setup_params.yaml']),
                {'robot_names': LaunchConfiguration('robots'),
                 'log_enabled': ["convoy_route"]}
            ]
        ),
        Node(
            package='vtr_multi_robot',
            executable='convoy_manager.py',
            name='convoy_manager',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_dir, LaunchConfiguration('base_params')]),
                PathJoinSubstitution([config_dir, LaunchConfiguration('convoy_params')]),
                {'robots': LaunchConfiguration('robots')}
            ]
        ),
    ])