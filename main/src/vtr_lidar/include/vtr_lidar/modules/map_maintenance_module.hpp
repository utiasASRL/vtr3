#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/cache.hpp>
#include <vtr_lidar/pointmap/pointmap.hpp>
#include <vtr_lidar/ray_tracing.hpp>
#include <vtr_tactic/modules/base_module.hpp>

// visualization
#include <sensor_msgs/msg/point_cloud2.hpp>
using PointCloudMsg = sensor_msgs::msg::PointCloud2;

namespace vtr {
namespace tactic {
namespace lidar {

/** \brief */
class MapMaintenanceModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.map_maintenance";

  /** \brief Config parameters. */
  struct Config {
    float map_voxel_size = 0.2;
    // dynamic objects remocal
    float horizontal_resolution = 0.001;
    float vertical_resolution = 0.001;
    int min_num_observations = 0;
    int max_num_observations = 20;

    bool visualize = false;
  };

  MapMaintenanceModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(QueryCache &qdata, const Graph::ConstPtr &graph) override;

  void visualizeImpl(QueryCache &, const Graph::ConstPtr &) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr aligned_points_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr movability_map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr movability_obs_map_pub_;
};

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr