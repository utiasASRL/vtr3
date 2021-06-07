#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {
namespace stereo {

/**
 * \brief A module that generates landmarks from image features. The landmark
 * point is 3D for stereo camera.
 * \details
 * requires: qdata.[rig_features, rig_calibrations]
 * outputs: qdata.[candidate_landmarks]
 *
 * This module converts stereo-matched features into landmarks with 3D points in
 * the first camera's frame. The landmarks are candidate as it has not been
 * matched to previous experiences.
 */
class ImageTriangulationModule : public BaseModule {
 public:
  /** \brief Static module identifier. \todo change this to static_name */
  static constexpr auto static_name = "image_triangulation";

  /** \brief Collection of config parameters */
  struct Config {
    bool visualize_features;
    bool visualize_stereo_features;
    float min_triangulation_depth;
    float max_triangulation_depth;
  };

  ImageTriangulationModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  /**
   * \brief Generates landmarks from image features. The landmark point is 3D
   * for stereo camera.
   */
  void runImpl(QueryCache &qdata, MapCache &, const Graph::ConstPtr &) override;

  /** \brief Visualizes features and stereo features. */
  void visualizeImpl(QueryCache &qdata, MapCache &, const Graph::ConstPtr &,
                     std::mutex &vis_mtx) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace stereo
}  // namespace tactic
}  // namespace vtr