import os
osp = os.path

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  # Get the config directory
  testing_config = osp.join(get_package_share_directory('vtr_testing'),
                            'config')
  base_config = osp.join(get_package_share_directory('vtr_navigation'),
                         'config/base')
  converter_config = list(
      map(lambda x: osp.join(base_config, "converter", x), [
          "image_conversion.yaml",
          "image_triangulation.yaml",
          "extraction_surf.yaml",
      ]))

  return launch.LaunchDescription([
      launch.actions.DeclareLaunchArgument('scenario_params',
                                           description='Run and data params'),
      launch_ros.actions.Node(
          package='vtr_testing',
          executable='module_vo',
          output='screen',
          # namespace='module_vo',
          name='navigator',
          parameters=[
              {
                  "converter": {
                      "type": "converter",
                      "modules": ["extraction", "triangulation"],
                  }
              },
              # base configs
              *converter_config,
              # scenario parameters
              launch.substitutions.PathJoinSubstitution(
                  (testing_config,
                   launch.substitutions.LaunchConfiguration("scenario_params")))
          ],
      )
  ])
