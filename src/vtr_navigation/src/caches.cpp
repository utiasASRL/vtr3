//#include <robochunk_babelfish/JoyBabelfish.pb.h>
#include <robochunk_msgs/Images.pb.h>
#include <robochunk_msgs/RigCalibration.pb.h>

// delay the implementation of caches until types are concrete
#include <vtr/navigation/caches.h>
#include <vtr/navigation/types.h>

// get the concrete type definitions
#include <asrl/messages/LocalizationStatus.pb.h>
#include <asrl/common/timing/SimpleTimer.hpp>
#include <asrl/common/utils/CacheContainer.impl>
#include <asrl/pose_graph/path/LocalizationChain.hpp>
// #include <asrl/steam_extensions/evaluator/common/MonoCameraErrorEval.hpp>
// #include <asrl/terrain_assessment/Patch.hpp>
#include <vtr/vision/types.h>
// #include <asrl/vision/stereo_proc/Types.hpp>

namespace asrl {
namespace common {
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
    int, boost::shared_ptr<vtr::steam_extensions::mono::LandmarkNoiseEvaluator>>>;
template class cache_ptr<vtr::navigation::MigrationMap>;
template class cache_ptr<vtr::navigation::SteamPoseMap>;
template class cache_ptr<vtr::navigation::SensorVehicleTransformMap>;
template class cache_ptr<asrl::common::timing::SimpleTimer>;
template class cache_ptr<pose_graph::LocalizationChain>;
template class cache_ptr<status_msgs::LocalizationStatus>;
}  // namespace common
}  // namespace asrl
