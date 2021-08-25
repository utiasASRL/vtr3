/**
 * \file velodyne_conversion_module.hpp
 * \brief VelodyneConversionModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/cache.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace lidar {

/**
 * \brief A specialized point cloud converter for data from Velodyne Alpha Prime
 * LiDAR.
 */
class VelodyneConversionModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.velodyne_converter";

  /** \brief Config parameters. */
  struct Config {
    bool visualize = false;
  };

  VelodyneConversionModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  std::shared_ptr<Config> config_;
};

}  // namespace lidar
}  // namespace vtr