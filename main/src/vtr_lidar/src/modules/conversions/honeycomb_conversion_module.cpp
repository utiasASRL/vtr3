/**
 * \file honeycomb_conversion_module.cpp
 * \brief HoneycombConversionModule class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vtr_lidar/modules/conversions/honeycomb_conversion_module.hpp>

namespace vtr {
namespace lidar {

using namespace tactic;

void HoneycombConversionModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->visualize = node->declare_parameter<bool>(param_prefix + ".visualize", config_->visualize);
  // clang-format on
}

void HoneycombConversionModule::runImpl(QueryCache &qdata0,
                                        const Graph::ConstPtr &) {
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);

  /// Create a node for visualization if necessary
  if (config_->visualize && !publisher_initialized_) {
    // clang-format off
    raw_pub_ = qdata.node->create_publisher<PointCloudMsg>("raw_points", 5);
    // clang-format on
    publisher_initialized_ = true;
  }

  /// Input
  const auto &msg = qdata.pointcloud_msg.ptr();

  // Copy over points and time
  const auto raw_pointcloud_cart = std::make_shared<std::vector<PointXYZ>>();
  const auto raw_pointcloud_pol = std::make_shared<std::vector<PointXYZ>>();
  const auto N = (size_t)(msg->width * msg->height);
  raw_pointcloud_cart->reserve(N);
  raw_pointcloud_pol->reserve(N);
  // clang-format off
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_rho(*msg, "range"), iter_theta(*msg, "pitch"), iter_phi(*msg, "yaw");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_beam_side(*msg, "beam_side");
  // clang-format on
  float phi0, phi1;
  size_t i0 = 0, i1 = 0;
  constexpr double PI2 = 2 * M_PI;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rho,
                                 ++iter_theta, ++iter_phi, ++iter_beam_side) {
    raw_pointcloud_cart->emplace_back(*iter_x, *iter_y, *iter_z);
    const auto theta = (*iter_theta) * M_PI / 180;
    auto phi = (*iter_phi) * M_PI / 180;
    if (*iter_beam_side == 0) {
      if (i0 && (phi - phi0) > M_PI)
        phi -= 2 * M_PI;
      else if (i0 && (phi - phi0) < -M_PI)
        phi += 2 * M_PI;
      phi0 = phi;
      i0++;
    } else if (*iter_beam_side == 1) {
      phi += PI2;
      if (i1 && (phi - phi1) > M_PI)
        phi -= 2 * M_PI;
      else if (i1 && (phi - phi1) < -M_PI)
        phi += 2 * M_PI;
      phi1 = phi;
      i1++;
    }
    raw_pointcloud_pol->emplace_back(*iter_rho, theta, phi);
  }

  /// Time Stamp of points \todo handle rotational effect
  const double time =
      msg->header.stamp.sec + (double)msg->header.stamp.nanosec / 1e9;
  const auto raw_pointcloud_time =
      std::make_shared<std::vector<double>>(raw_pointcloud_cart->size(), time);

  /// Output
  qdata.raw_pointcloud_time = raw_pointcloud_time;
  qdata.raw_pointcloud_cart = raw_pointcloud_cart;
  qdata.raw_pointcloud_pol = raw_pointcloud_pol;

  /// Visualize
  if (config_->visualize) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    auto cartitr = qdata.raw_pointcloud_cart->begin();
    auto politr = qdata.raw_pointcloud_pol->begin();
    for (; cartitr != qdata.raw_pointcloud_cart->end(); cartitr++, politr++) {
      pcl::PointXYZI pt;
      pt.x = cartitr->x;
      pt.y = cartitr->y;
      pt.z = cartitr->z;
      pt.intensity = politr->z;
      cloud.points.push_back(pt);
    }
    auto pc2_msg = std::make_shared<PointCloudMsg>();
    pcl::toROSMsg(cloud, *pc2_msg);
    pc2_msg->header.frame_id = "robot";
    pc2_msg->header.stamp = *qdata.rcl_stamp;

    raw_pub_->publish(*pc2_msg);
  }
}

}  // namespace lidar
}  // namespace vtr