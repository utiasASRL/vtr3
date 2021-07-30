#pragma once

#include "rclcpp/rclcpp.hpp"

#include <vtr_tactic/types.hpp>
#include <vtr_tactic/utils/cache_container.hpp>

#include <vtr_common/timing/simple_timer.hpp>

#include <lgmath/se3/TransformationWithCovariance.hpp>

#include <vtr_messages/msg/time_stamp.hpp>

// lidar stuff
#include <vtr_lidar/grid_subsampling/grid_subsampling.hpp>
#include <vtr_lidar/pointmap/pointmap.hpp>
#include <vtr_lidar/polar_processing/polar_processing.hpp>

// image stuff
#include <vtr_messages/msg/localization_status.hpp>
#include <vtr_messages/msg/match.hpp>

// common messages
using TimeStampMsg = vtr_messages::msg::TimeStamp;

namespace vtr {
namespace tactic {

struct QueryCache : public common::CacheContainer {
  using Ptr = std::shared_ptr<QueryCache>;

  QueryCache()
      : placeholder("placeholder", janitor_.get()),
        // temp
        node("node", janitor_.get()),
        // common
        stamp("stamp", janitor_.get()),
        rcl_stamp("rcl_stamp", janitor_.get()),
        first_frame("first_frame", janitor_.get()),
        live_id("live_id", janitor_.get()),
        map_id("map_id", janitor_.get()),
        T_r_m_odo("T_r_m_odo", janitor_.get()),
        T_r_m_loc("T_r_m_loc", janitor_.get()),
        keyframe_test_result("keyframe_test_result", janitor_.get()),
        odo_success("odo_success", janitor_.get()),
        loc_success("loc_success", janitor_.get()),
        trajectory("trajectory", janitor_.get()),
        robot_frame("robot_frame", janitor_.get()),
        // lidar related stuff
        lidar_frame("lidar_frame", janitor_.get()),
        T_s_r("T_s_r", janitor_.get()),
        raw_pointcloud_time("raw_pointcloud_time", janitor_.get()),
        raw_pointcloud("raw_pointcloud", janitor_.get()),
        preprocessed_pointcloud_time("preprocessed_pointcloud_time", janitor_.get()),
        preprocessed_pointcloud("preprocessed_pointcloud", janitor_.get()),
        normals("normals", janitor_.get()),
        undistorted_pointcloud("undistorted_pointcloud", janitor_.get()),
        undistorted_normals("undistorted_normals", janitor_.get()),
        icp_scores("icp_scores", janitor_.get()),
        normal_scores("normal_scores", janitor_.get()),
        matched_points_ratio("matched_points_ratio", janitor_.get()),
        current_map_odo("current_map_odo", janitor_.get()),
        current_map_odo_vid("current_map_odo_vid", janitor_.get()),
        current_map_odo_T_v_m("current_map_odo_T_v_m", janitor_.get()),
        current_map_loc("current_map_loc", janitor_.get()),
        new_map("new_map", janitor_.get()),
        // image related stuff
        camera_frame("camera_frame", janitor_.get()),
        steam_mutex("steam_mutex", janitor_.get()),
        T_sensor_vehicle("T_sensor_vehicle", janitor_.get()),
        rig_names("rig_names", janitor_.get()),
        rig_images("rig_images", janitor_.get()),
        rig_calibrations("rig_calibrations", janitor_.get()),
        rig_features("rig_features", janitor_.get()),
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
        loc_timer("loc_solve_time", janitor_.get()){};

  common::cache_ptr<float, true> placeholder;

  // temp
  common::cache_ptr<rclcpp::Node> node;

  // common
  common::cache_ptr<TimeStampMsg> stamp;
  common::cache_ptr<rclcpp::Time> rcl_stamp;
  common::cache_ptr<bool> first_frame;
  common::cache_ptr<VertexId> live_id;
  common::cache_ptr<VertexId> map_id;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_odo;  //
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_loc;  //
  common::cache_ptr<KeyframeTestResult> keyframe_test_result;
  common::cache_ptr<bool, true> odo_success;
  common::cache_ptr<bool, true> loc_success;
  common::cache_ptr<steam::se3::SteamTrajInterface> trajectory;
  common::cache_ptr<std::string> robot_frame;

  /// lidar related stuff
  common::cache_ptr<std::string> lidar_frame;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_s_r;  //
  common::cache_ptr<std::vector<double>> raw_pointcloud_time;
  common::cache_ptr<std::vector<PointXYZ>> raw_pointcloud;
  common::cache_ptr<std::vector<double>> preprocessed_pointcloud_time;
  common::cache_ptr<std::vector<PointXYZ>> preprocessed_pointcloud;
  common::cache_ptr<std::vector<PointXYZ>> normals;
  common::cache_ptr<std::vector<PointXYZ>> undistorted_pointcloud;
  common::cache_ptr<std::vector<PointXYZ>> undistorted_normals;
  common::cache_ptr<std::vector<float>> icp_scores;
  common::cache_ptr<std::vector<float>> normal_scores;
  common::cache_ptr<float> matched_points_ratio;

  common::cache_ptr<lidar::IncrementalPointMap> current_map_odo;
  common::cache_ptr<VertexId> current_map_odo_vid;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> current_map_odo_T_v_m;
  common::cache_ptr<lidar::IncrementalPointMap> current_map_loc;
  common::cache_ptr<lidar::IncrementalPointMap> new_map;

  /// image related stuff
  common::cache_ptr<std::string> camera_frame;
  common::cache_ptr<std::shared_ptr<std::mutex>> steam_mutex;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_sensor_vehicle;
  common::cache_ptr<std::vector<std::string>> rig_names;
  common::cache_ptr<std::list<vision::RigImages>> rig_images;
  common::cache_ptr<std::list<vision::RigCalibration>> rig_calibrations;
  common::cache_ptr<std::vector<vision::RigFeatures>> rig_features;
  common::cache_ptr<std::vector<vision::RigLandmarks>> candidate_landmarks;

  /// extra image related stuff to be cleaned up
  common::cache_ptr<bool, true> success;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_prior;
  common::cache_ptr<std::vector<vision::RigMatches>> raw_matches;
  common::cache_ptr<std::map<VertexId, lgmath::se3::TransformationWithCovariance>> T_sensor_vehicle_map;
  common::cache_ptr<std::vector<LandmarkFrame>> map_landmarks;
  common::cache_ptr<std::vector<vision::RigMatches>> ransac_matches;
  common::cache_ptr<bool, true> steam_failure;
  // odometry and mapping (including bungle adjustment)
  common::cache_ptr<LandmarkMap> landmark_map;
  common::cache_ptr<SteamPoseMap> pose_map;  // window optimization
  // localization
  common::cache_ptr<RunIdSet> recommended_experiences;
  common::cache_ptr<pose_graph::RCGraphBase::Ptr> localization_map;
  common::cache_ptr<Eigen::Matrix4Xd> migrated_points;
  common::cache_ptr<Eigen::Matrix3Xd> migrated_points_3d;
  common::cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>> migrated_covariance;
  common::cache_ptr<std::vector<bool>> migrated_validity;
  common::cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>> projected_map_points;
  common::cache_ptr<std::vector<vtr_messages::msg::Match>> migrated_landmark_ids;
  common::cache_ptr<MigrationMap> landmark_offset_map;
  common::cache_ptr<std::unordered_map<int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>> stereo_landmark_noise;  // \todo check what this is
  common::cache_ptr<vtr_messages::msg::LocalizationStatus> localization_status;
  common::cache_ptr<common::timing::SimpleTimer> loc_timer;
};

struct MapCache : public common::CacheContainer {
  using Ptr = std::shared_ptr<MapCache>;

  MapCache()
      : placeholder("placeholder", janitor_.get()),
        node("node", janitor_.get()),
        map_id("map_id", janitor_.get()),
        T_r_m("T_r_m", janitor_.get()),
        T_r_m_loc("T_r_m_loc", janitor_.get())
        /*,
        // lidar related stuff
        current_map("current_map", janitor_.get()),
        current_map_loc("current_map_loc", janitor_.get()),
        new_map("new_map", janitor_.get()),
        // stereo related stuff
        success("success", janitor_.get()),
        T_r_m_prior("T_r_m_prior", janitor_.get()),
        T_sensor_vehicle_map("T_sensor_vehicle_map", janitor_.get()),
        map_landmarks("map_landmarks", janitor_.get()),
        raw_matches("raw_matches", janitor_.get()),
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
        loc_timer("loc_solve_time", janitor_.get())*/
        {};
  // clang-format off
  common::cache_ptr<float, true> placeholder;

  // temporary stuff to be cleaned up
  common::cache_ptr<const rclcpp::Node::SharedPtr> node;
  common::cache_ptr<VertexId> map_id;
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m;      //
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_loc;  //

  // /// Image related stuff
  // common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_r_m_prior;
  // common::cache_ptr<std::vector<vision::RigMatches>> raw_matches;
  // common::cache_ptr<std::vector<vision::RigMatches>> ransac_matches;
  // common::cache_ptr<bool, true> steam_failure;
  // common::cache_ptr<bool, true> success;
  // common::cache_ptr<std::map<VertexId, lgmath::se3::TransformationWithCovariance>> T_sensor_vehicle_map;
  // common::cache_ptr<std::vector<LandmarkFrame>> map_landmarks;
  // // odometry and mapping (including bungle adjustment)
  // common::cache_ptr<LandmarkMap> landmark_map;
  // common::cache_ptr<SteamPoseMap> pose_map; // window optimization
  // common::cache_ptr<std::unordered_map<int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>> stereo_landmark_noise;  // \todo check what this is
  // // localization
  // common::cache_ptr<vtr_messages::msg::LocalizationStatus> localization_status;
  // common::cache_ptr<pose_graph::RCGraphBase::Ptr> localization_map;
  // common::cache_ptr<RunIdSet> recommended_experiences;
  // common::cache_ptr<Eigen::Matrix4Xd> migrated_points;
  // common::cache_ptr<Eigen::Matrix3Xd> migrated_points_3d;
  // common::cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>> migrated_covariance;
  // common::cache_ptr<std::vector<bool>> migrated_validity;
  // common::cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>> projected_map_points;
  // common::cache_ptr<std::vector<vtr_messages::msg::Match>> migrated_landmark_ids;
  // common::cache_ptr<MigrationMap> landmark_offset_map;
  // common::cache_ptr<common::timing::SimpleTimer> loc_timer;
};
}  // namespace tactic
}  // namespace vtr