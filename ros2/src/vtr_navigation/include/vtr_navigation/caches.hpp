#pragma once

#include <map>
#include <set>
#include <unordered_map>
#if false
#include <functional>
#include <memory>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

#include <vtr_common/utils/cache_container.hpp>
#include <vtr_messages/msg/localization_status.hpp>
#include <vtr_messages/msg/match.hpp>
#include <vtr_messages/msg/time_stamp.hpp>

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

namespace steam {
namespace se3 {
class SteamTrajInterface;
}  // namespace se3
namespace stereo {
class LandmarkNoiseEvaluator;
}  // namespace stereo
}  // namespace steam

namespace vtr {
namespace common {
namespace timing {
class SimpleTimer;
}
}  // namespace common

namespace pose_graph {
class VertexId;
class RCGraphBase;
class LocalizationChain;
}  // namespace pose_graph

namespace vision {
class RigImages;
class RigCalibration;
class RigFeatures;
class RigLandmarks;
class RigMatches;
class LandmarkId;
}  // namespace vision

namespace navigation {
class LandmarkFrame;
class LandmarkInfo;
using RunIdSet = std::set<uint32_t>;
using MigrationMap = std::unordered_map<vision::LandmarkId, int>;
using LandmarkMap = std::unordered_map<vision::LandmarkId, LandmarkInfo>;
using SensorVehicleTransformMap =
    std::map<pose_graph::VertexId, lgmath::se3::TransformationWithCovariance>;
class SteamPose;
using SteamPoseMap = std::map<pose_graph::VertexId, SteamPose>;
}  // namespace navigation

}  // namespace vtr

#if false  // \todo yuchen old code as reference
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
namespace vision_msgs {
class Match;
}  // namespace vision_msgs
namespace terrain_assessment {
class Patch;
typedef std::shared_ptr<Patch> PatchPtr;
typedef std::vector<PatchPtr> PatchPtrs;
}  // namespace terrain_assessment
}  // namespace asrl

namespace vtr {
namespace vision {
class LandmarkId;
class RigCalibration;
class IMUCalibration;
class RigFeatures;
class RigLandmarks;
class RigMatches;
class RigImages;
}  // namespace vision
namespace navigation {
typedef std::set<uint32_t> RunIdSet;
class LandmarkInfo;
class LandmarkFrame;
typedef std::vector<vision::LandmarkId> LandmarkIdVec;
typedef std::unordered_map<vision::LandmarkId, LandmarkInfo> LandmarkMap;
typedef std::unordered_map<vision::LandmarkId, int> MigrationMap;
class SteamPose;
typedef std::map<asrl::pose_graph::VertexId, SteamPose> SteamPoseMap;
typedef std::map<asrl::pose_graph::VertexId,
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
}  // namespace vtr

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
#endif
////////////////////////////////////////////////////////////////////////////////

namespace vtr {
namespace navigation {

/** \brief The channels that can be stored in the container */
struct QueryCache : public common::CacheContainer {
  /// Registers each member with the janitor for cleanup.
  /// There is a compile error if you fail to register with the janitor.
  QueryCache()
      : stamp("stamp", janitor_.get()),
        rig_names("rig_names", janitor_.get()),
        rig_images("rig_images", janitor_.get()),
        rig_calibrations("rig_calibrations", janitor_.get()),
        rig_features("rig_features", janitor_.get()),
        candidate_landmarks("candidate_landmarks", janitor_.get()),
        T_sensor_vehicle("T_sensor_vehicle", janitor_.get()),
        live_id("live_id", janitor_.get()),
        steam_mutex("steam_mutex", janitor_.get()),
        trajectory("trajectory", janitor_.get()),
        new_vertex_flag("new_vertex_flag", janitor_.get()) {
  }

  common::cache_ptr<vtr_messages::msg::TimeStamp, true> stamp;

  common::cache_ptr<std::vector<std::string>> rig_names;
  common::cache_ptr<std::list<vision::RigImages>> rig_images;
  common::cache_ptr<std::list<vision::RigCalibration>> rig_calibrations;
  // Process frames (features ...)
  common::cache_ptr<std::vector<vtr::vision::RigFeatures>> rig_features;
  // Landmarks generated
  common::cache_ptr<std::vector<vtr::vision::RigLandmarks>> candidate_landmarks;

  // SE3 Transform from the vehicle to sensor
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_sensor_vehicle;

  // Vertex ID if it's a keyframe
  common::cache_ptr<pose_graph::VertexId> live_id;

  // steam optimization isn't thread safe.
  // avoids different steam modules from conflicting
  common::cache_ptr<std::shared_ptr<std::mutex>> steam_mutex;

  // trajectory estimator
  common::cache_ptr<steam::se3::SteamTrajInterface> trajectory;

  // Indicates whether the frame failed vertex creation test criteria or should
  // become a candidate
  common::cache_ptr<int> new_vertex_flag;
};

struct MapCache : public common::CacheContainer {
  /// Registers each member with the janitor for cleanup.
  /// There is a compile error if you fail to register with the janitor.
  MapCache()
      : success("success", janitor_.get()),
        map_status("map_status", janitor_.get()),
        recommended_experiences("recommended_experiences", janitor_.get()),
        ransac_matches("ransac_matches", janitor_.get()),
        localization_map("localization_map", janitor_.get()),
        map_landmarks("map_landmarks", janitor_.get()),
        triangulated_matches("triangulated_matches", janitor_.get()),
        map_id("map_id", janitor_.get()),
        T_q_m_prior("T_q_m_prior", janitor_.get()),
        T_sensor_vehicle_map("T_sensor_vehicle_map", janitor_.get()),
        raw_matches("raw_matches", janitor_.get()),
        steam_failure("steam_failure", janitor_.get()),
        T_q_m("T_q_m", janitor_.get()),
        landmark_map("landmark_map", janitor_.get()),
        migrated_points("migrated_points", janitor_.get()),
        migrated_covariance("migrated_covariance", janitor_.get()),
        migrated_points_3d("migrated_points_3d", janitor_.get()),
        migrated_validity("migrated_validity", janitor_.get()),
        projected_map_points("projected_map_points", janitor_.get()),
        migrated_landmark_ids("migrated_landmark_ids", janitor_.get()),
        landmark_offset_map("landmark_offset_map", janitor_.get()),
        pose_map("pose_map", janitor_.get()),
        loc_timer("loc_solve_time", janitor_.get()),
        stereo_landmark_noise("landmark_noise", janitor_.get()),
        localization_chain("localization_chain", janitor_.get()),
        localization_status("localization_status", janitor_.get()) {
  }

  // Was the most recent step a success?
  common::cache_ptr<bool, true> success;

  // Is the map initialised for each rig?
  // This is for monocular VO, which can't progress until the map has been
  // initialized
  common::cache_ptr<int, true> map_status;

  common::cache_ptr<RunIdSet> recommended_experiences;

  // matches that have passed RANSAC
  common::cache_ptr<std::vector<vision::RigMatches>> ransac_matches;

  common::cache_ptr<std::shared_ptr<pose_graph::RCGraphBase>> localization_map;

  // Core map data  \todo make the vectors live inside LandmarkFrame
  common::cache_ptr<std::vector<LandmarkFrame>> map_landmarks;

  // matches that have passed triangulation (monocular)
  common::cache_ptr<std::vector<vision::RigMatches>> triangulated_matches;

  // Graph indices
  common::cache_ptr<pose_graph::VertexId, true> map_id;

  // a prior pose estimate
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_q_m_prior;

  // Vehicle-Sensor transform map (windowed optimization)
  common::cache_ptr<SensorVehicleTransformMap> T_sensor_vehicle_map;

  // raw matches from a matching module
  common::cache_ptr<std::vector<vtr::vision::RigMatches>> raw_matches;

  // Did steam fail on the last attempt to optimize?
  common::cache_ptr<bool, true> steam_failure;

  // Final estimate (in the vehicle frame)
  common::cache_ptr<lgmath::se3::TransformationWithCovariance> T_q_m;

  // Landmark map (windowed optimization)
  common::cache_ptr<LandmarkMap> landmark_map;

  // Landmark Migration module cache variables
  common::cache_ptr<Eigen::Matrix4Xd> migrated_points;
  common::cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>>
      migrated_covariance;
  common::cache_ptr<Eigen::Matrix3Xd> migrated_points_3d;
  common::cache_ptr<std::vector<bool>> migrated_validity;
  common::cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>>
      projected_map_points;
  common::cache_ptr<std::vector<vtr_messages::msg::Match>>
      migrated_landmark_ids;
  common::cache_ptr<MigrationMap> landmark_offset_map;

  // Pose map (windowed optimization)
  common::cache_ptr<SteamPoseMap> pose_map;

  common::cache_ptr<common::timing::SimpleTimer> loc_timer;

  common::cache_ptr<std::unordered_map<
      int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>>
      stereo_landmark_noise;

  // Localization chain
  common::cache_ptr<pose_graph::LocalizationChain> localization_chain;

  // Localization status
  common::cache_ptr<vtr_messages::msg::LocalizationStatus> localization_status;
};

}  // namespace navigation
}  // namespace vtr
