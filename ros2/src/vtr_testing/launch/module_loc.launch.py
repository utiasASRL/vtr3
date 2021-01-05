import os
osp = os.path

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  vtr_grizzly = get_package_share_directory('vtr_grizzly')
  vtr_testing = get_package_share_directory('vtr_testing')
  vtr_navigation = get_package_share_directory('vtr_navigation')
  # base configs
  base_config = osp.join(vtr_navigation, 'config/base')
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
  base_localization_config = list(
      map(lambda x: osp.join(base_config, "localization", x), [
          "map_extraction.yaml",
          "mel_matcher.yaml",
          "stereo_ransac.yaml",
          "mel_opt.yaml",
      ]))
  # robot specific configs
  grizzly_config = osp.join(vtr_navigation, 'config/grizzly')
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
  grizzly_localization_config = list(
      map(lambda x: osp.join(grizzly_config, "localization", x), [
          "map_extraction.yaml",
          "mel_matcher.yaml",
          "stereo_ransac.yaml",
          "mel_opt.yaml",
      ]))
  # scenario specific configs
  testing_config = osp.join(vtr_testing, 'config')

  return launch.LaunchDescription([
      launch.actions.DeclareLaunchArgument('scenario_params',
                                           description='Run and data params'),
      # Launch ModuleLoc
      launch_ros.actions.Node(
          package='vtr_testing',
          executable='module_loc',
          output='screen',
          # namespace='module_loc',
          name='navigator',
          parameters=[
              {
                  "converter": {
                      "type": "converter",
                      "modules": ["extraction", "triangulation"],
                  },
                  "quick_vo": {
                      "type":
                          "quick_vo",
                      "modules": [
                          "recall", "matcher", "ransac", "steam", "vertex_test"
                      ],
                  },
                  "refined_vo": {
                      "type": "refined_vo",
                      "modules": ["recall", "steam"],
                  },
                  "loc": {
                      "type": "loc",
                      "modules": ["sub_map_extraction", "recall", "migration",
                                  "matcher", "ransac", "steam"],        # todo (Ben) add  other MEL modules
                  }
              },
              # base configs
              *base_tactic_config,
              *base_converter_config,
              *base_quick_vo_config,
              *base_refined_vo_config,
              *base_localization_config,
              # robot specific configs
              *grizzly_tactic_config,
              *grizzly_converter_config,
              *grizzly_quick_vo_config,
              *grizzly_refined_vo_config,
              *grizzly_localization_config,
              # scenario specific configs
              launch.substitutions.PathJoinSubstitution(
                  (testing_config,
                   launch.substitutions.LaunchConfiguration("scenario_params")))
          ],
      ),
      # Launch grizzly description to get transformation matrices.
      launch.actions.IncludeLaunchDescription(
          launch.launch_description_sources.PythonLaunchDescriptionSource(
              osp.join(vtr_grizzly,
                       "launch/grizzly_utias_description.launch.py"))),
  ])
