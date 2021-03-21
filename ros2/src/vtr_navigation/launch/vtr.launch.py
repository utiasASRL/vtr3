import os
osp = os.path

import launch
import launch.actions
from launch.actions import DeclareLaunchArgument
import launch.substitutions
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros.actions
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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
          "landmark_recall.yaml",
          "landmark_migration.yaml",
          "random_experiences.yaml",
          "time_of_day_recognition.yaml",
          "collaborative_landmarks.yaml",
          "experience_triage.yaml",
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
          "landmark_recall.yaml",
          "landmark_migration.yaml",
          "random_experiences.yaml",
          "time_of_day_recognition.yaml",
          "collaborative_landmarks.yaml",
          "experience_triage.yaml",
          "mel_matcher.yaml",
          "stereo_ransac.yaml",
          "mel_opt.yaml",
      ]))      
  # scenario specific configs
  scenario_config = osp.join(vtr_navigation, 'config/scenario')

  # path-tracker config
  vtr_path_tracker = get_package_share_directory('vtr_path_tracker')
  pt_config = osp.join(vtr_path_tracker, 'config/grizzly')
  grizzly_path_tracker_config = [osp.join(pt_config, "path_tracker_grizzly.yaml")]
  grizzly_path_tracker_gains_config = [osp.join(pt_config, "gains_grizzly.yaml")]

  return launch.LaunchDescription([
      DeclareLaunchArgument('data_dir', description='Data directory'),
      DeclareLaunchArgument('override_data_dir',
                            description='=Override directory'),
      DeclareLaunchArgument('scenario_params',
                            description='Run and data params'),
      DeclareLaunchArgument('feature_type',
                            default_value='surf',
                            description='Sparse feature to use'),
      DeclareLaunchArgument('robot_type',
                            default_value='grizzly',
                            description='Robot to use'),
      DeclareLaunchArgument('planner_type',
                            default_value='distance',
                            description='Planner to use'),
      # Launch module vo
      launch_ros.actions.Node(
          package='vtr_navigation',
          executable='navigator',
          output='screen',
          # namespace='module_vo',
          name='navigator',
          remappings=[("/cmd_vel", "/grizzly_velocity_controller/cmd_vel")],
          parameters=[
              {
                  "data_dir": LaunchConfiguration("data_dir"),
                  "override_data_dir": LaunchConfiguration("override_data_dir"),
                  # TODO: the following is same as module VO but not the true vtr
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
                      "modules": [
                          "sub_map_extraction", "recall",
                          # "random_experiences",
                          "timeofday_recognition",
                          # "collaborative_landmarks",
                          "experience_triage",
                          "migration", "matcher", "ransac", "steam"
                      ],
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
              # path-tracker config
              *grizzly_path_tracker_config,
              *grizzly_path_tracker_gains_config,
              # scenario specific configs
              PathJoinSubstitution(
                  (scenario_config, LaunchConfiguration("scenario_params")))
          ],
      ),
  ])
