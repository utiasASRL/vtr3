#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {
namespace lidar {

/** \brief Preprocesses raw pointcloud points and computes normals. */
class KeyframeTestModule : public BaseModule {
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
    int max_num_frames = 50;
  };

  KeyframeTestModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(QueryCache &qdata, const Graph::ConstPtr &graph) override;

  std::shared_ptr<Config> config_;
};

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr