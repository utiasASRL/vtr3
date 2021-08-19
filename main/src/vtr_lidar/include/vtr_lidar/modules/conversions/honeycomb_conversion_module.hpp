/**
 * \file honeycomb_conversion_module.hpp
 * \brief HoneycombConversionModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/cache.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace lidar {

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

/**
 * \brief A specialized point cloud converter for data from Waymo Honeycomb
 * LiDAR.
 */
class HoneycombConversionModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.honeycomb_converter";

  /** \brief Config parameters. */
  struct Config {
    bool visualize = false;
  };

  HoneycombConversionModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr raw_pub_;
};

}  // namespace lidar
}  // namespace vtr