#include <vtr_lidar/utils.hpp>

namespace {

inline int getPointCloud2FieldIndex(const sensor_msgs::msg::PointCloud2 &cloud,
                                    const std::string &field_name) {
  // Get the index we need
  for (size_t d = 0; d < cloud.fields.size(); ++d)
    if (cloud.fields[d].name == field_name) return (d);
  return (-1);
}

}  // namespace

namespace vtr {
namespace lidar {

void copyPointcloud(const PointCloudMsg::SharedPtr msg,
                    std::vector<PointXYZ> &pts, std::vector<double> &ts) {
  size_t N = (size_t)(msg->width * msg->height);
  // Copy over points
  pts.reserve(N);
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
       iter_y(*msg, "y"), iter_z(*msg, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    pts.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
  }

  // Copy over time stamp of each point
  if (getPointCloud2FieldIndex(*msg, "t") != -1) {
    ts.reserve(N);
    for (sensor_msgs::PointCloud2ConstIterator<double> iter(*msg, "t");
         iter != iter.end(); ++iter) {
      ts.push_back(*iter);
    }
  } else {
    double time_stamp =
        msg->header.stamp.sec + (double)msg->header.stamp.nanosec / 1e9;
    ts = std::vector<double>(N, time_stamp);
  }
}

}  // namespace lidar
}  // namespace vtr