#!/usr/bin/env python
import os
osp = os.path

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
  pkg_share = FindPackageShare('vtr_grizzly').find('vtr_grizzly')
  urdf_dir = os.path.join(pkg_share, 'urdf')
  xacro_file = os.path.join(urdf_dir, 'grizzly_utias.urdf.xacro')
  doc = xacro.process_file(xacro_file)
  robot_desc = doc.toprettyxml(indent='  ')
  params = {'robot_description': robot_desc}
  rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                output='both',
                                parameters=[params])

  return launch.LaunchDescription([rsp])
