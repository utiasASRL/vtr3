import os
osp = os.path

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  # base configs
  base_config = osp.join(get_package_share_directory('vtr_navigation'),
                         'config/base')
  base_tactic_config = [osp.join(base_config, "tactic.yaml")]
  base_converter_config = list(
      map(lambda x: osp.join(base_config, "converter", x), [
          "image_conversion.yaml",
          "extraction_surf.yaml",
          "image_triangulation.yaml",
      ]))
  base_quick_vo_config = list(
      map(lambda x: osp.join(base_config, "quick_vo", x), [
          "ASRL_stereo_matcher.yaml",
          "stereo_ransac.yaml",
          "keyframe_opt.yaml",
          "vertex_test.yaml",
      ]))
  base_refined_vo_config = list(
      map(lambda x: osp.join(base_config, "refined_vo", x), [
          "experience_recognition.yaml",
          "window_opt.yaml",
      ]))
  # robot specific configs
  grizzly_config = osp.join(get_package_share_directory('vtr_navigation'),
                            'config/grizzly')
  grizzly_tactic_config = [osp.join(grizzly_config, "tactic.yaml")]
  grizzly_converter_config = list(
      map(lambda x: osp.join(grizzly_config, "converter", x), [
          "image_conversion.yaml",
          "extraction_surf.yaml",
          "image_triangulation.yaml",
      ]))
  grizzly_quick_vo_config = list(
      map(lambda x: osp.join(grizzly_config, "quick_vo", x), [
          "ASRL_stereo_matcher.yaml",
          "stereo_ransac.yaml",
          "keyframe_opt.yaml",
          "vertex_test.yaml",
      ]))
  grizzly_refined_vo_config = list(
      map(lambda x: osp.join(grizzly_config, "refined_vo", x), [
          "window_opt.yaml",
      ]))
  # scenario specific configs
  testing_config = osp.join(get_package_share_directory('vtr_testing'),
                            'config')

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
                  },
                  "quick_vo": {
                      "type": "quick_vo",
                      # "modules": [],
                  },
                  "refined_vo": {
                      "type": "refined_vo",
                      # "modules": [],
                  }
              },
              # base configs
              *base_tactic_config,
              *base_converter_config,
              *base_quick_vo_config,
              *base_refined_vo_config,
              # robot specific configs
              *grizzly_tactic_config,
              *grizzly_converter_config,
              *grizzly_quick_vo_config,
              *grizzly_refined_vo_config,
              # scenario specific configs
              launch.substitutions.PathJoinSubstitution(
                  (testing_config,
                   launch.substitutions.LaunchConfiguration("scenario_params")))
          ],
      )
  ])
