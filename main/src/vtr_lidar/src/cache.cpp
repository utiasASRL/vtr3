/**
 * \file cache.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/cache.hpp>
#include <vtr_tactic/utils/cache_container.inl>

namespace vtr {
namespace common {
template class cache_ptr<sensor_msgs::msg::PointCloud2>;
template class cache_ptr<std::vector<lidar::PointXYZ>>;
template class cache_ptr<Eigen::Matrix4d>;
template class cache_ptr<lidar::IncrementalPointMap>;
template class cache_ptr<lidar::SingleExpPointMap>;
template class cache_ptr<lidar::MultiExpPointMap>;
}  // namespace common
}  // namespace vtr