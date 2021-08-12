#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/utils/cache_container.inl>

namespace vtr {
namespace common {

template class cache_ptr<int>;
template class cache_ptr<float>;
template class cache_ptr<bool>;
template class cache_ptr<bool, true>;
template class cache_ptr<std::string>;
template class cache_ptr<std::vector<std::string>>;
template class cache_ptr<std::vector<bool>>;
template class cache_ptr<std::vector<float>>;
template class cache_ptr<std::vector<double>>;

template class cache_ptr<rclcpp::Node>;
template class cache_ptr<rclcpp::Time>;
template class cache_ptr<vtr_messages::msg::TimeStamp>;
template class cache_ptr<tactic::VertexId>;
template class cache_ptr<lgmath::se3::TransformationWithCovariance>;
template class cache_ptr<tactic::KeyframeTestResult>;
template class cache_ptr<steam::se3::SteamTrajInterface>;

}  // namespace common
}  // namespace vtr