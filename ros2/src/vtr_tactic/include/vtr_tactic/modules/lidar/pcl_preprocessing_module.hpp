#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/polar_processing/polar_processing.h>
#include <vtr_tactic/modules/base_module.hpp>

// temp
#include <sensor_msgs/msg/point_cloud2.hpp>
using PointCloudMsg = sensor_msgs::msg::PointCloud2;

namespace vtr {
namespace tactic {

/** \brief Preprocess raw pointcloud points and compute normals */
class PCLPreprocessingModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "pcl_preprocessing";

  /** \brief Collection of config parameters */
  struct Config {
    int lidar_n_lines = 64;
    float polar_r_scale = 1.5;
    float r_scale = 4.0;
    float h_scale = 0.5;
    float frame_voxel_size = 0.1;
    bool visualize = false;
  };

  PCLPreprocessingModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Visualization */
  void visualizeImpl(QueryCache &, MapCache &, const Graph::ConstPtr &,
                     std::mutex &) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  rclcpp::Publisher<PointCloudMsg>::SharedPtr pc_pub_;
};

}  // namespace tactic
}  // namespace vtr