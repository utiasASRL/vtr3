/**
 * \file utils.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <cstring>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vtr_lidar/cloud/cloud.hpp>

namespace vtr {
namespace lidar {
using PointCloudMsg = sensor_msgs::msg::PointCloud2;

/**
 * \brief Converts a PointCloud2 message to the format used by the lidar
 * pipeline.
 * \param[in] msg
 * \param[out] pts
 * \param[out] ts
 */
void copyPointcloud(const PointCloudMsg::SharedPtr msg,
                    std::vector<PointXYZ> &pts, std::vector<double> &ts);

}  // namespace lidar
}  // namespace vtr