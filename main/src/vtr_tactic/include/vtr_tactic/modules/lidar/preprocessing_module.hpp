#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/polar_processing/polar_processing.hpp>
#include <vtr_tactic/modules/base_module.hpp>

// temp
#include <sensor_msgs/msg/point_cloud2.hpp>
using PointCloudMsg = sensor_msgs::msg::PointCloud2;

namespace vtr {
namespace tactic {
namespace lidar {

/** \brief Preprocess raw pointcloud points and compute normals */
class PreprocessingModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.preprocessing";

  /** \brief Collection of config parameters */
  struct Config {
    int num_threads = 1;
    int num_channels = 64;
    float vertical_angle_res = 0.00745;
    float polar_r_scale = 1.5;
    float r_scale = 4.0;
    float h_scale = 0.5;
    float frame_voxel_size = 0.1;
    int num_sample1 = 100000;
    int num_sample2 = 100000;
    float min_norm_score1 = 0.0;
    float min_norm_score2 = 0.01;
    bool visualize = false;
  };

  PreprocessingModule(const std::string &name = static_name)
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
  rclcpp::Publisher<PointCloudMsg>::SharedPtr pc_pub_;
};

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr