#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include "vtr_lidar/pointmap/pointmap.h"

#include <vtr_tactic/modules/base_module.hpp>

// temp
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vtr_messages_lidar/msg/pointcloud_map.hpp>
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointXYZMsg = vtr_messages_lidar::msg::PointXYZ;
using PointCloudMapMsg = vtr_messages_lidar::msg::PointcloudMap;

namespace vtr {
namespace tactic {

/** \brief Preprocess raw pointcloud points and compute normals */
class MapMaintenanceModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "map_maintenance";

  /** \brief Collection of config parameters */
  struct Config {
    float map_voxel_size = 0.03;
    bool visualize = false;
  };

  MapMaintenanceModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  void updateGraphImpl(QueryCache &qdata, MapCache &mdata,
                       const Graph::Ptr &graph, VertexId live_id) override;

  /** \brief Visualization */
  void visualizeImpl(QueryCache &, MapCache &, const Graph::ConstPtr &,
                     std::mutex &) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;
};

}  // namespace tactic
}  // namespace vtr