/// Frame data container for the navigation assemblies.
/// (see asrl__common namedFruits sample for general usage info)
#pragma once

#include <functional>
#include <memory>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include <asrl/common/utils/CacheContainer.hpp>

////////////////////////////////////////////////////////////////////////////////
// This awful mess of forward declarations is to avoid the awful sprawl of
// includes you see in Caches.cpp. If you need to add a cache variable,
// strongly prefer forward declaring, or having it in a very minimal header.
// If you have undefined references, add the explicit instatiation in the cpp.
namespace lgmath {
namespace se3 {
class Transformation;
class TransformationWithCovariance;
}  // namespace se3
}  // namespace lgmath
namespace asrl {
namespace common {
namespace timing {
class SimpleTimer;
}  // namespace timing
}  // namespace common
namespace pose_graph {
class LocalizationChain;
// class RCGraph;
class RCGraphBase;
class VertexId;
}  // namespace pose_graph
namespace status_msgs {
class LocalizationStatus;
}
namespace vision {
class LandmarkId;
class RigCalibration;
class IMUCalibration;
class RigFeatures;
class RigLandmarks;
class RigMatches;
class RigImages;
}  // namespace vision
namespace vision_msgs {
class Match;
}  // namespace vision_msgs
namespace terrain_assessment {
class Patch;
typedef std::shared_ptr<Patch> PatchPtr;
typedef std::vector<PatchPtr> PatchPtrs;
}  // namespace terrain_assessment
namespace navigation {
typedef std::set<uint32_t> RunIdSet;
class LandmarkInfo;
class LandmarkFrame;
typedef std::vector<vision::LandmarkId> LandmarkIdVec;
typedef std::unordered_map<vision::LandmarkId, LandmarkInfo> LandmarkMap;
typedef std::unordered_map<vision::LandmarkId, int> MigrationMap;
class SteamPose;
typedef std::map<pose_graph::VertexId, SteamPose> SteamPoseMap;
typedef std::map<pose_graph::VertexId,
                 lgmath::se3::TransformationWithCovariance>
    SensorVehicleTransformMap;
class RansacData;
class SteamPose;
}  // namespace navigation
namespace steam_extensions {
namespace mono {
class LandmarkNoiseEvaluator;
}  // namespace mono
}  // namespace steam_extensions
}  // namespace asrl

namespace lgmath {
namespace se3 {
class Transformation;
class TransformationWithCovariance;
}  // namespace se3
}  // namespace lgmath

namespace cv {
template <typename T>
class Point_;
typedef Point_<float> Point2f;
}  // namespace cv
namespace pcl {
class PointXYZ;
template <typename T>
class PointCloud;
}  // namespace pcl

namespace robochunk {
namespace sensor_msgs {
class Image;
class RigImages;
}  // namespace sensor_msgs
namespace std_msgs {
class TimeStamp;
}  // namespace std_msgs
}  // namespace robochunk

namespace steam {
namespace se3 {
class SteamTrajInterface;
}  // namespace se3
namespace stereo {
class LandmarkNoiseEvaluator;
}  // namespace stereo
}  // namespace steam

////////////////////////////////////////////////////////////////////////////////

namespace asrl {
namespace navigation {

/// @brief The channels that can be stored in the container
///        See the .cpp for type definitions
struct QueryCache : public common::CacheContainer {
  /// Registers each member with the janitor for cleanup.
  /// There is a compile error if you fail to register with the janitor.
  QueryCache()
      : rig_names("rig_names", janitor_.get()),
        rig_images("rig_images", janitor_.get()),
        rig_features("rig_features", janitor_.get()),
        rig_calibrations("rig_calibrations", janitor_.get()),
        new_vertex_flag("new_vertex_flag", janitor_.get()),
        stamp("stamp", janitor_.get()),
#if 0
        imu_calibrations("imu_calibrations", janitor_.get()),
        joy("joy", janitor_.get()),
#endif
        candidate_landmarks("candidate_landmarks", janitor_.get()),
        live_id("live_id", janitor_.get()),
        T_sensor_vehicle("T_sensor_vehicle", janitor_.get()),
        trajectory("trajectory", janitor_.get()),
#if 0
        T_0_q("T_0_q", janitor_.get()),
        pointclouds("pointclouds", janitor_.get()),
        lookahead_patches("lookahead_patches", janitor_.get()),
        training_patches("training_patches", janitor_.get()),
        position("position", janitor_.get()),
        orientation("orientation", janitor_.get()),
        angular_velocity("angular_velocity", janitor_.get()),
        linear_acceleration("linear_acceleration", janitor_.get()),
#endif
        steam_mutex("steam_mutex", janitor_.get())
#if 0
        imu_bias("imu_bias", janitor_.get())
#endif
  {
  }
  // Indicates whether the frame failed vertex creation test criteria or should
  // become a candidate
  common::cache_ptr<int> new_vertex_flag;

  // Timestamp
  common::cache_ptr<robochunk::std_msgs::TimeStamp, true> stamp;
  // Rig Names
  common::cache_ptr<std::vector<std::string>> rig_names;
  // Raw images
  common::cache_ptr<std::list<vision::RigImages>> rig_images;

  // Process frames (features ...)
  common::cache_ptr<std::vector<vision::RigFeatures>> rig_features;

  // Calibrations
  common::cache_ptr<std::list<vision::RigCalibration>> rig_calibrations;
#if 0
  common::cache_ptr<std::list<vision::IMUCalibration>> imu_calibrations;

  // Joy
  // \todo Joy ros->proto conversion had a problem, so this vector of buttons is
  // just
  //       a placeholder until the full proto msg can be used instead.
  common::cache_ptr<std::vector<int>> joy;
#endif
  // Landmarks generated
  common::cache_ptr<std::vector<vision::RigLandmarks>> candidate_landmarks;
  // Vertex ID if it's a keyframe
  common::cache_ptr<pose_graph::VertexId> live_id;
  // SE3 Transform from the vehicle to sensor
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_sensor_vehicle;
  // trajectory estimator
  common::cache_ptr<steam::se3::SteamTrajInterface> trajectory;

#if 0
  // GPS LLA position

  // Topocentric T_0_q
  common::cache_ptr<lgmath::se3::Transformation> T_0_q;

  // Terrain assessment stuff.
  common::cache_ptr<
      std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>>>
      pointclouds;
  common::cache_ptr<terrain_assessment::PatchPtrs> lookahead_patches;
  common::cache_ptr<
      std::vector<std::map<uint32_t, asrl::terrain_assessment::PatchPtrs>>>
      training_patches;

  // the closest estimated GPS UTM position
  common::cache_ptr<Eigen::Vector3d> position;
  // the IMU or wheel odometry orientation
  common::cache_ptr<Eigen::Quaterniond> orientation;
  // the IMU angular velocity
  common::cache_ptr<Eigen::Vector3d> angular_velocity;
  // the IMU linear acceleration
  common::cache_ptr<Eigen::Vector3d> linear_acceleration;
#endif

  // steam optimization isn't thread safe. avoids different steam modules from
  // conflicting
  common::cache_ptr<std::shared_ptr<std::mutex>> steam_mutex;

#if 0
  // IMU bias
  common::cache_ptr<Eigen::Matrix<double, 6, 1>> imu_bias;
#endif
};

struct MapCache : public common::CacheContainer {
  /// Registers each member with the janitor for cleanup.
  /// There is a compile error if you fail to register with the janitor.
  MapCache()
      : rig_images("rig_images", janitor_.get()),
#if 0
    mono_images("mono_images", janitor_.get()),
#endif
        map_id("map_id", janitor_.get()),
#if 0
    localization_map("localization_map", janitor_.get()),
#endif
        map_landmarks("map_landmarks", janitor_.get()),
        raw_matches("raw_matches", janitor_.get()),
        ransac_matches("ransac_matches", janitor_.get()),
        triangulated_matches("triangulated_matches", janitor_.get()),
        T_q_m("T_q_m", janitor_.get()),
        T_q_m_prior("T_q_m_prior", janitor_.get()),
        T_q_m_prev("T_q_m_prev", janitor_.get()),
        stamp_prev("stamp_prev", janitor_.get()),
        trajectory_prev("trajectory_prev", janitor_.get()),
        H_q_m("H_q_m", janitor_.get()),
        H_q_m_prior("H_q_m_prior", janitor_.get()),
        plane_coefficients("plane_coefficients", janitor_.get()),
#if 0
    depth_prior("depth_prior", janitor_.get()),
    pixel_prior("pixel_prior", janitor_.get()),
#endif
        map_status("map_status", janitor_.get()),
        success("success", janitor_.get()),
#if 0
    descriptor_distance("desc_dist", janitor_.get()),
    recommended_experiences("recommended_experiences", janitor_.get()),
    recommended_landmarks("recommended_landmarks", janitor_.get()),
    ransac_data("ransac_data", janitor_.get()),
#endif
        landmark_map("landmark_map", janitor_.get()),
        pose_map("pose_map", janitor_.get()),
        T_sensor_vehicle_map("T_sensor_vehicle_map", janitor_.get()),
#if 0
    migrated_points("migrated_points", janitor_.get()),
#endif
        migrated_points_3d("migrated_points_3d", janitor_.get()),
        migrated_validity("migrated_validity", janitor_.get()),
        migrated_covariance("migrated_covariance", janitor_.get()),
        projected_map_points("projected_map_points", janitor_.get()),
#if 0
    landmark_offset_map("landmark_offset_map", janitor_.get()),
#endif
        migrated_landmark_ids("migrated_landmark_ids", janitor_.get()),
#if 0
    loc_timer("loc_solve_time", janitor_.get()),
#endif
        stereo_landmark_noise("landmark_noise", janitor_.get()),
        mono_landmark_noise("landmark_noise", janitor_.get()),
        steam_failure("steam_failure", janitor_.get())
#if 0
    localization_chain("localization_chain", janitor_.get()),
    localization_status("localization_status", janitor_.get())
#endif
  {
  }
  //////////////////////////////////////////////////////////////////////////////
  // Data members

  // Raw images
  common::cache_ptr<std::list<robochunk::sensor_msgs::RigImages>> rig_images;
#if 0
  common::cache_ptr<std::list<robochunk::sensor_msgs::Image>> mono_images;
#endif
  // Graph indices
  common::cache_ptr<pose_graph::VertexId, true> map_id;
#if 0
  // The localization sub-map
  common::cache_ptr<std::shared_ptr<pose_graph::RCGraphBase>> localization_map;
#endif
  // Core map data
  // TODO make the vectors live inside LandmarkFrame
  common::cache_ptr<std::vector<LandmarkFrame>> map_landmarks;
  // raw matches from a matching module
  common::cache_ptr<std::vector<vision::RigMatches>> raw_matches;
  // matches that have passed RANSAC
  common::cache_ptr<std::vector<vision::RigMatches>> ransac_matches;
#if 0
  common::cache_ptr<RansacData> ransac_data;
#endif
  // matches that have passed triangulation (monocular)
  common::cache_ptr<std::vector<vision::RigMatches>> triangulated_matches;
  // matches that have passed RANSAC for localization
  //  common::cache_ptr<vision::LandmarkMatches> localization_matches;
  // The previous pose estimate (in the vehicle frame)
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_q_m_prev;

  // Timestamp for previous cache
  common::cache_ptr<robochunk::std_msgs::TimeStamp, true> stamp_prev;
  // Previous trajectory estimator
  common::cache_ptr<steam::se3::SteamTrajInterface> trajectory_prev;
  // a prior pose estimate
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_q_m_prior;
  // Final estimate (in the vehicle frame)
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_q_m;
  // a homography estimate
  common::cache_ptr<Eigen::Matrix3d> H_q_m;
  // a prior homography estimate
  common::cache_ptr<Eigen::Matrix3d> H_q_m_prior;
  // the plane equation for the structure in monocular lancaster VO
  common::cache_ptr<Eigen::Vector4f> plane_coefficients;

#if 0
  // a landmark depth prior for monocular matching
  common::cache_ptr<double> depth_prior;

  // a landmark depth prior for monocular matching
  common::cache_ptr<cv::Point2f> pixel_prior;

#endif
  // Is the map initialised for each rig?
  // This is for monocular VO, which can't progress until the map has been
  // initialized
  common::cache_ptr<int, true> map_status;
  // Was the most recent step a success?
  common::cache_ptr<bool, true> success;
  // Did steam fail on the last attempt to optimize?
  common::cache_ptr<bool, true> steam_failure;

#if 0
  // Used for progressive ransac (hit at match quality)
  common::cache_ptr<std::vector<float>> descriptor_distance;

  /// The experiences we should use for multi-experience localization.
  /// Produced by experience recognition, consumed by submap extraction.
  /// The experiences are specified by run id, and sorted in a set.
  /// The privileged run will be used whether or not it's in the set
  common::cache_ptr<RunIdSet> recommended_experiences;
  common::cache_ptr<LandmarkIdVec> recommended_landmarks;
#endif
  // Landmark map (windowed optimization)
  common::cache_ptr<LandmarkMap> landmark_map;
#if 0
  common::cache_ptr<Eigen::Matrix4Xd> migrated_points;
#endif
  common::cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>>
      migrated_covariance;
  common::cache_ptr<Eigen::Matrix3Xd> migrated_points_3d;
  common::cache_ptr<std::vector<bool>> migrated_validity;
  common::cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>>
      projected_map_points;
  common::cache_ptr<std::vector<vision_msgs::Match*>> migrated_landmark_ids;
  common::cache_ptr<std::unordered_map<
      int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>>
      stereo_landmark_noise;
  common::cache_ptr<std::unordered_map<
      int, boost::shared_ptr<steam_extensions::mono::LandmarkNoiseEvaluator>>>
      mono_landmark_noise;
#if 0
  common::cache_ptr<MigrationMap> landmark_offset_map;

#endif
  // Pose map (windowed optimization)
  common::cache_ptr<SteamPoseMap> pose_map;
  // Vehicle-Sensor transform map (windowed optimization)
  common::cache_ptr<SensorVehicleTransformMap> T_sensor_vehicle_map;
#if 0
  common::cache_ptr<asrl::common::timing::SimpleTimer> loc_timer;

  // Localization chain
  common::cache_ptr<pose_graph::LocalizationChain> localization_chain;

  // Localization status
  common::cache_ptr<status_msgs::LocalizationStatus> localization_status;
#endif
};

}  // namespace navigation
}  // namespace asrl
