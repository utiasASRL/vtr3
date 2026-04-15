/**
 * \file localization_map_recall_module_lv.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Extends LocalizationMapRecallModule to also recall intensity features
 * stored at teach vertices for visual localization.
 */
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vtr_lidar/cache.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#include "vtr_tactic/task_queue.hpp"

namespace vtr {
namespace lidar {

class LocalizationMapRecallModuleLV : public tactic::BaseModule {
 public:
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.localization_map_recall_lv";

  /** \brief Config parameters */
  struct Config : public tactic::BaseModule::Config {
    PTR_TYPEDEFS(Config);

    std::string map_version = "multi_exp_point_map";
    bool visualize = false;

    static ConstPtr fromROS(const rclcpp::Node::SharedPtr &node,
                            const std::string &param_prefix);
  };

  LocalizationMapRecallModuleLV(
      const Config::ConstPtr &config,
      const std::shared_ptr<tactic::ModuleFactory> &module_factory = nullptr,
      const std::string &name = static_name)
      : tactic::BaseModule{module_factory, name}, config_(config) {}

 private:
  void run_(tactic::QueryCache &qdata, tactic::OutputCache &output,
            const tactic::Graph::Ptr &graph,
            const tactic::TaskExecutor::Ptr &executor) override;

  Config::ConstPtr config_;

  /** \brief for visualization only */
  bool publisher_initialized_ = false;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr map_pub_;
  rclcpp::Publisher<PointCloudMsg>::SharedPtr test_map_pub_;

  VTR_REGISTER_MODULE_DEC_TYPE(LocalizationMapRecallModuleLV);
};

}  // namespace lidar
}  // namespace vtr
