/**
 * \file visual_map_maintenance_module.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Maintains the sliding visual landmark map (IntensityFeatureMap) for
 * odometry, mirroring OdometryMapMaintenanceModuleV2 for lidar points:
 *
 *   1. takes the live intensity features that odometry motion-undistorted
 *      to the scan end (qdata.undistorted_intensity_features)
 *   2. transforms them into the odometry map frame using the scan-end pose
 *   3. merges them into the sliding feature map (voxel + descriptor dedup)
 *   4. ages landmarks and drops those not re-observed for a while
 *
 * Run this module after the lidar `mapping` module in the odometry list.
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

/** \brief Sliding visual landmark map maintenance for odometry. */
class VisualMapMaintenanceModule : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.visual_map_maintenance";

  /** \brief Config parameters. */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    /// Landmark dedup voxel size (metres)
    float feature_voxel_size = 0.3;
    /// Scans a landmark survives without re-observation (negative = infinite)
    float feature_life_time = 20.0;
    /// Max ORB Hamming distance to merge as the same landmark
    float descriptor_merge_dist = 50.0;

    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr& node,
                            const std::string& param_prefix);
  };

  VisualMapMaintenanceModule(
      const Config::ConstPtr& config,
      const std::shared_ptr<tactic::ModuleFactory>& module_factory = nullptr,
      const std::string& name = static_name)
      : tactic::BaseModule(module_factory, name), config_(config) {}

 private:
  void run_(tactic::QueryCache& qdata, tactic::OutputCache& output,
            const tactic::Graph::Ptr& graph,
            const tactic::TaskExecutor::Ptr& executor) override;

  Config::ConstPtr config_;

  /** \brief Landmark visualization publisher */
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;
  bool publisher_initialized_ = false;

  VTR_REGISTER_MODULE_DEC_TYPE(VisualMapMaintenanceModule);
};

}  // namespace lidar
}  // namespace vtr
