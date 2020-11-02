// delay the implementation of caches until types are concrete
#include <lgmath.hpp>

#include <vtr_common/utils/cache_container.inl>
#include <vtr_messages/msg/time_stamp.hpp>
#include <vtr_navigation/caches.hpp>
#include <vtr_navigation/types.hpp>
#include <vtr_vision/types.hpp>

#if false
// get the concrete type definitions
#include <asrl/messages/LocalizationStatus.pb.h>
#include <asrl/common/timing/SimpleTimer.hpp>
#include <asrl/pose_graph/path/LocalizationChain.hpp>
// #include <asrl/steam_extensions/evaluator/common/MonoCameraErrorEval.hpp>
// #include <asrl/terrain_assessment/Patch.hpp>
// #include <asrl/vision/stereo_proc/Types.hpp>

//#include <robochunk_babelfish/JoyBabelfish.pb.h>
#include <robochunk_msgs/Images.pb.h>
#include <robochunk_msgs/RigCalibration.pb.h>
#endif

namespace vtr {
namespace common {
// clang-format off
template class cache_ptr<vtr_messages::msg::TimeStamp, true>;  // stamp

template class cache_ptr<std::vector<std::string>>;  // rig_names
template class cache_ptr<std::list<vision::RigImages>>;  // rig_images
template class cache_ptr<std::list<vision::RigCalibration>>;  // rig_calibrations
template class cache_ptr<vision::SuiteFeatures>;  // rig_features
template class cache_ptr<vision::SuiteLandmarks>;  // candidate_landmarks

template class cache_ptr<lgmath::se3::TransformationWithCovariance>;  // T_sensor_vehicle, T_q_m_prior, T_q_m

template class cache_ptr<navigation::VertexId>;  // live_id

template class cache_ptr<std::shared_ptr<std::mutex>>;  // steam_mutex

template class cache_ptr<bool, true>;  // success, steam_failure

template class cache_ptr<int, true>;  // map_status

template class cache_ptr<std::vector<vision::RigMatches>>;  // ransac_matches, triangulated_matches, raw_matches

template class cache_ptr<std::vector<navigation::LandmarkFrame>>;  // map_landmarks

template class cache_ptr<navigation::VertexId, true>;  // map_id

template class cache_ptr<navigation::SensorVehicleTransformMap>;  // T_sensor_vehicle_map

template class cache_ptr<steam::se3::SteamTrajInterface>;  // trajectory

template class cache_ptr<int>;  // new_vertex_flag

template class cache_ptr<navigation::LandmarkMap>;  // landmark_map

template class cache_ptr<navigation::SteamPoseMap>;  // pose_map

// clang-format on

#if false  // \todo yuchen old code as reference
template class cache_ptr<int>;
template class cache_ptr<robochunk::std_msgs::TimeStamp, true>;
template class cache_ptr<std::vector<std::string>>;
template class cache_ptr<std::list<vtr::vision::RigImages>>;
template class cache_ptr<vtr::vision::SuiteFeatures>;  // RigFeatures
template class cache_ptr<std::list<vtr::vision::RigCalibration>>;
#if 0
template class cache_ptr<std::list<vtr::vision::IMUCalibration>>;
#endif
template class cache_ptr<std::vector<int>>;
template class cache_ptr<vtr::vision::SuiteLandmarks>;  // RigLandmarks
template class cache_ptr<vtr::navigation::VertexId>;
template class cache_ptr<vtr::navigation::EdgeTransform>;
template class cache_ptr<steam::se3::SteamTrajInterface>;
template class cache_ptr<lgmath::se3::Transformation>;
#if 0
template class cache_ptr<std::vector<vtr::vision::PointcloudPtr>>;
template class cache_ptr<terrain_assessment::Patch::Ptrs>;
template class cache_ptr<std::vector<std::map<
    vtr::navigation::Graph::RunIdType, asrl::terrain_assessment::Patch::Ptrs>>>;
template class cache_ptr<vtr::navigation::Position>;
template class cache_ptr<vtr::navigation::Orientation>;
#endif
template class cache_ptr<std::shared_ptr<std::mutex>>;
template class cache_ptr<std::list<robochunk::sensor_msgs::RigImages>>;
template class cache_ptr<std::list<robochunk::sensor_msgs::Image>>;
template class cache_ptr<vtr::navigation::VertexId, true>;
template class cache_ptr<std::shared_ptr<pose_graph::RCGraphBase>>;
template class cache_ptr<std::vector<vtr::navigation::LandmarkFrame>>;
template class cache_ptr<vtr::vision::SuiteMatches>;
#if 0
template class cache_ptr<vtr::navigation::RansacData>;
#endif
template class cache_ptr<Eigen::Matrix3d>;
template class cache_ptr<Eigen::Vector4f>;
template class cache_ptr<Eigen::Matrix<double, 6, 1>>;
template class cache_ptr<double>;
template class cache_ptr<vtr::vision::Point>;
template class cache_ptr<bool, true>;
template class cache_ptr<int, true>;
template class cache_ptr<std::vector<float>>;
template class cache_ptr<vtr::navigation::RunIdSet>;
template class cache_ptr<vtr::navigation::LandmarkIdVec>;
template class cache_ptr<vtr::navigation::LandmarkMap>;
template class cache_ptr<Eigen::Matrix4Xd>;
template class cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>>;
template class cache_ptr<Eigen::Matrix3Xd>;
template class cache_ptr<std::vector<bool>>;
template class cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>>;
template class cache_ptr<std::vector<vision_msgs::Match*>>;
template class cache_ptr<std::unordered_map<
    int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>>;
template class cache_ptr<std::unordered_map<
    int,
    boost::shared_ptr<vtr::steam_extensions::mono::LandmarkNoiseEvaluator>>>;
template class cache_ptr<vtr::navigation::MigrationMap>;
template class cache_ptr<vtr::navigation::SteamPoseMap>;
template class cache_ptr<vtr::navigation::SensorVehicleTransformMap>;
template class cache_ptr<asrl::common::timing::SimpleTimer>;
template class cache_ptr<pose_graph::LocalizationChain>;
template class cache_ptr<status_msgs::LocalizationStatus>;
#endif
}  // namespace common
}  // namespace vtr
