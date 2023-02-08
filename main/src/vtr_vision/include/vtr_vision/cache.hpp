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

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/types.hpp>
// #include <vtr_vision/types.hpp>

#include <vtr_messages/msg/localization_status.hpp>

namespace vtr {
namespace vision {

struct CameraQueryCache : public tactic::QueryCache {
//   using Ptr = std::shared_ptr<CameraQueryCache>;
  PTR_TYPEDEFS(LidarQueryCache);

  CameraQueryCache()
      : QueryCache(),
        camera_frame("camera_frame", janitor_.get()),
        steam_mutex("steam_mutex", janitor_.get()),
        vis_mutex("vis_mutex", janitor_.get()),
        T_sensor_vehicle("T_sensor_vehicle", janitor_.get()),
        rig_names("rig_names", janitor_.get()),
        rig_images("rig_images", janitor_.get()),
        rig_calibrations("rig_calibrations", janitor_.get()),
        rig_features("rig_features", janitor_.get()),
        rig_extra("rig_extra", janitor_.get()),
        candidate_landmarks("candidate_landmarks", janitor_.get()),
        // extra image related stuff to be cleaned up
        success("success", janitor_.get()),
        T_r_m("T_r_m", janitor_.get()),
        T_r_m_prior("T_r_m_prior", janitor_.get()),
        raw_matches("raw_matches", janitor_.get()),
        T_sensor_vehicle_map("T_sensor_vehicle_map", janitor_.get()),
        map_landmarks("map_landmarks", janitor_.get()),
        ransac_matches("ransac_matches", janitor_.get()),
        steam_failure("steam_failure", janitor_.get()),
        landmark_map("landmark_map", janitor_.get()),
        pose_map("pose_map", janitor_.get()),
        recommended_experiences("recommended_experiences", janitor_.get()),
        localization_map("localization_map", janitor_.get()),
        migrated_points("migrated_points", janitor_.get()),
        migrated_points_3d("migrated_points_3d", janitor_.get()),
        migrated_covariance("migrated_covariance", janitor_.get()),
        migrated_validity("migrated_validity", janitor_.get()),
        projected_map_points("projected_map_points", janitor_.get()),
        migrated_landmark_ids("migrated_landmark_ids", janitor_.get()),
        landmark_offset_map("landmark_offset_map", janitor_.get()),
        stereo_landmark_noise("landmark_noise", janitor_.get()),
        localization_status("localization_status", janitor_.get()),
        loc_timer("loc_solve_time", janitor_.get()) {}
  // clang-format off
  /// image related stuff
  common::cache_ptr<std::string> camera_frame;
  common::cache_ptr<std::mutex> steam_mutex;
  common::cache_ptr<std::mutex> vis_mutex;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_sensor_vehicle;
  common::cache_ptr<std::vector<std::string>> rig_names;
  common::cache_ptr<std::list<vision::RigImages>> rig_images;
  common::cache_ptr<std::list<vision::RigCalibration>> rig_calibrations;
  common::cache_ptr<std::vector<vision::RigFeatures>> rig_features;
  common::cache_ptr<std::vector<vision::RigLandmarks>> candidate_landmarks;
  common::cache_ptr<std::vector<vision::RigExtra>> rig_extra;

  /// extra image related stuff to be cleaned up
  common::cache_ptr<bool, true> success;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_prior;
  common::cache_ptr<std::vector<vision::RigMatches>> raw_matches;
  common::cache_ptr<std::map<tactic::VertexId, lgmath::se3::TransformationWithCovariance>> T_sensor_vehicle_map;
  common::cache_ptr<std::vector<LandmarkFrame>> map_landmarks;
  common::cache_ptr<std::vector<vision::RigMatches>> ransac_matches;
  common::cache_ptr<bool, true> steam_failure;
  // odometry and mapping (including bungle adjustment)
  common::cache_ptr<LandmarkMap> landmark_map;
  common::cache_ptr<SteamPoseMap> pose_map;
  // localization
  common::cache_ptr<tactic::RunIdSet> recommended_experiences;
  common::cache_ptr<pose_graph::RCGraphBase::Ptr> localization_map;
  common::cache_ptr<Eigen::Matrix4Xd> migrated_points;
  common::cache_ptr<Eigen::Matrix3Xd> migrated_points_3d;
  common::cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>> migrated_covariance;
  common::cache_ptr<std::vector<bool>> migrated_validity;
  common::cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>> projected_map_points;
  common::cache_ptr<std::vector<vtr_messages::msg::Match>> migrated_landmark_ids;
  common::cache_ptr<MigrationMap> landmark_offset_map;
  common::cache_ptr<std::unordered_map<int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>> stereo_landmark_noise;
  common::cache_ptr<vtr_messages::msg::LocalizationStatus> localization_status;
  common::cache_ptr<common::timing::SimpleTimer> loc_timer;
  // clang-format on
};
}  // namespace vision
}  // namespace vtr