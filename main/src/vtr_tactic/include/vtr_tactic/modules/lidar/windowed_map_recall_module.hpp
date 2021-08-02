#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_tactic/modules/base_module.hpp>

#include <vtr_pose_graph/path/pose_cache.hpp>

// temp
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vtr_messages_lidar/msg/point_map.hpp>
using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointXYZMsg = vtr_messages_lidar::msg::PointXYZ;
using PointMapMsg = vtr_messages_lidar::msg::PointMap;

namespace vtr {
namespace tactic {
namespace lidar {

/** \brief Preprocess raw pointcloud points and compute normals */
class WindowedMapRecallModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.windowed_map_recall";

  /** \brief Collection of config parameters */
  struct Config {
    float single_exp_map_voxel_size = 0.1;
    float multi_exp_map_voxel_size = 0.3;
    int depth = 1;
    int num_additional_exps = 0;
    bool visualize = false;
  };

  WindowedMapRecallModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Visualization */
  void visualizeImpl(QueryCache &, MapCache &, const Graph::ConstPtr &,
                     std::mutex &) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr observation_map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr experience_map_pub_;
};

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr