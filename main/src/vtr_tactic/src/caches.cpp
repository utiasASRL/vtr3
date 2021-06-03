#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/utils/cache_container.inl>

namespace vtr {
namespace common {
// clang-format off

template class cache_ptr<int>;
template class cache_ptr<float>;
template class cache_ptr<bool>;
template class cache_ptr<bool, true>;
template class cache_ptr<std::vector<std::string>>;
template class cache_ptr<std::vector<bool>>;


template class cache_ptr<tactic::KeyframeTestResult>;
template class cache_ptr<tactic::VertexId>;
template class cache_ptr<TimeStampMsg>;
template class cache_ptr<lgmath::se3::TransformationWithCovariance>;
template class cache_ptr<rclcpp::Time>;
template class cache_ptr<const rclcpp::Node::SharedPtr>;
template class cache_ptr<rclcpp::Node>;
template class cache_ptr<std::vector<float>>;
template class cache_ptr<std::vector<PointXYZ>>;
template class cache_ptr<Eigen::Matrix4d>;
template class cache_ptr<PointMap>;
template class cache_ptr<std::vector<std::shared_ptr<PointMap>>>;

// image related stuff
template class cache_ptr<std::shared_ptr<std::mutex>>;
template class cache_ptr<std::map<tactic::VertexId, lgmath::se3::TransformationWithCovariance>>;
template class cache_ptr<std::list<vision::RigImages>>;
template class cache_ptr<std::list<vision::RigCalibration>>;
template class cache_ptr<std::vector<vision::RigFeatures>>;
template class cache_ptr<std::vector<vision::RigLandmarks>>;
template class cache_ptr<std::vector<tactic::LandmarkFrame>>;
template class cache_ptr<std::vector<vision::RigMatches>>;
template class cache_ptr<Eigen::Matrix<double, 2, Eigen::Dynamic>>;
template class cache_ptr<Eigen::Matrix<double, 9, Eigen::Dynamic>>;
template class cache_ptr<std::vector<vtr_messages::msg::Match>>;
template class cache_ptr<Eigen::Matrix3Xd>;
template class cache_ptr<Eigen::Matrix4Xd>;
template class cache_ptr<tactic::RunIdSet>;
template class cache_ptr<steam::se3::SteamTrajInterface>;
template class cache_ptr<std::unordered_map<int, boost::shared_ptr<steam::stereo::LandmarkNoiseEvaluator>>>;
template class cache_ptr<tactic::LandmarkMap>;
template class cache_ptr<tactic::SteamPoseMap>;
template class cache_ptr<pose_graph::RCGraphBase::Ptr>;
template class cache_ptr<vtr_messages::msg::LocalizationStatus>;
template class cache_ptr<tactic::MigrationMap>;
template class cache_ptr<common::timing::SimpleTimer>;

}  // namespace common
}  // namespace vtr