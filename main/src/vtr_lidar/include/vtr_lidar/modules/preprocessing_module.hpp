/**
 * \file preprocessing_module.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <pcl_conversions/pcl_conversions.h>

#include <vtr_lidar/cache.hpp>
#include <vtr_lidar/polar_processing/polar_processing.hpp>
#include <vtr_tactic/modules/base_module.hpp>

// visualization
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vtr {
namespace lidar {

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

/** \brief Preprocess raw pointcloud points and compute normals */
class PreprocessingModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.preprocessing";

  /** \brief Config parameters. */
  struct Config {
    int num_threads = 1;
    int num_channels = 64;
    float vertical_angle_res = 0.00745;
    float polar_r_scale = 1.5;
    float r_scale = 4.0;
    float h_scale = 0.5;
    float frame_voxel_size = 0.1;
    int num_sample1 = 100000;
    float min_norm_score1 = 0.0;
    int num_sample2 = 100000;
    float min_norm_score2 = 0.01;
    float ideal_normal_estimate_dist = 2.0;
    int cluster_num_sample = 100000;
    bool visualize = false;
  };

  PreprocessingModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  void visualizeImpl(tactic::QueryCache &,
                     const tactic::Graph::ConstPtr &) override;

  std::shared_ptr<Config> config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr grid_sampled_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr normal_sampled_pub_;
};

}  // namespace lidar
}  // namespace vtr