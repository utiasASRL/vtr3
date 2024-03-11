// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file cache.hpp
 * \brief CameraQueryCache class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "sensor_msgs/msg/image.hpp"


#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/types.hpp>
#include <vtr_vision/types.hpp>
#include <vtr_vision_msgs/msg/localization_status.hpp>

namespace vtr {
namespace vision {

struct CameraQueryCache : public tactic::QueryCache {
  PTR_TYPEDEFS(CameraQueryCache);

  // clang-format off

  //Transformation between sensor and robot
  tactic::Cache<const tactic::EdgeTransform> T_s_r;


  tactic::Cache<std::vector<std::string>> rig_names;

  tactic::Cache<std::list<vision::RigImages>> rig_images;
  tactic::Cache<std::list<vision::RigCalibration>> rig_calibrations;
  tactic::Cache<std::vector<vision::RigFeatures>> rig_features;
  tactic::Cache<std::vector<vision::RigLandmarks>> candidate_landmarks;
  tactic::Cache<std::vector<vision::RigExtra>> rig_extra;
  tactic::Cache<sensor_msgs::msg::Image> left_image;
  tactic::Cache<sensor_msgs::msg::Image> right_image;
  

  /// extra image related stuff to be cleaned up
  tactic::Cache<lgmath::se3::TransformationWithCovariance> T_r_m;
  tactic::Cache<lgmath::se3::TransformationWithCovariance> T_r_m_prior;
  tactic::Cache<std::vector<vision::RigMatches>> raw_matches;
  tactic::Cache<std::map<tactic::VertexId, lgmath::se3::TransformationWithCovariance>> T_sensor_vehicle_map;
  tactic::Cache<std::vector<LandmarkFrame>> map_landmarks;
  tactic::Cache<std::vector<vision::RigMatches>> ransac_matches;
  tactic::Cache<std::mutex> steam_mutex;

  /// localization
  tactic::Cache<Eigen::Matrix4Xd>  migrated_points;
  tactic::Cache<Eigen::Matrix3Xd> migrated_points_3d;
  tactic::Cache<Eigen::Matrix<double, 9, Eigen::Dynamic>> migrated_covariance;
  tactic::Cache<Eigen::Matrix<double, 2, Eigen::Dynamic>> projected_map_points;
  tactic::Cache<std::vector<vtr_vision_msgs::msg::Match>> migrated_landmark_ids;


  tactic::Cache<vtr_vision_msgs::msg::LocalizationStatus> localization_status;
  tactic::Cache<tactic::GraphBase::Ptr> localization_map;
  tactic::Cache<tactic::RunIdSet> recommended_experiences;

  tactic::Cache<std::vector<bool>> migrated_validity;
  tactic::Cache<MigrationMap> landmark_offset_map;


  tactic::Cache<LandmarkMap> landmark_map;
  tactic::Cache<SteamPoseMap> pose_map;

  tactic::Cache<std::mutex> vis_mutex;

  // clang-format on
};
}  // namespace vision
}  // namespace vtr