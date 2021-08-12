#pragma once

#include <cstring>

#include <vtr_lidar/cloud/cloud.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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