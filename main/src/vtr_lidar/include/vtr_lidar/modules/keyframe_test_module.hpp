/**
 * \file keyframe_test_module.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_lidar/cache.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace lidar {

/** \brief Preprocesses raw pointcloud points and computes normals. */
class KeyframeTestModule : public tactic::BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.keyframe_test";

  /** \brief Config parameters. */
  struct Config {
    float min_translation = 0;
    float min_rotation = 0;
    float max_translation = 10;
    float max_rotation = 30;
    float min_matched_points_ratio = 0.5;
    int max_num_points = 100000;
  };

  KeyframeTestModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  std::shared_ptr<Config> config_;
};

}  // namespace lidar
}  // namespace vtr