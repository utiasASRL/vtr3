#pragma once

#include <vtr_navigation/modules/base_module.hpp>

namespace vtr {
namespace navigation {

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
  static constexpr auto type_str_ = "image_triangulation";

  /** \brief Collection of config parameters */
  struct Config {
    bool visualize_features;
    bool visualize_stereo_features;
    float min_triangulation_depth;
    float max_triangulation_depth;
  };

  ImageTriangulationModule(std::string name = type_str_) : BaseModule{name} {};

  void setConfig(std::shared_ptr<Config> &config) {
    config_ = config;
  }

 protected:
  /**
   * \brief Generates landmarks from image features. The landmark point is 3D
   * for stereo camera.
   */
  void run(QueryCache &qdata, MapCache &,
           const std::shared_ptr<const Graph> &) override;

  /** \brief Visualizes features and stereo features. */
  void visualize(QueryCache &qdata, MapCache &,
                 const std::shared_ptr<const Graph> &,
                 std::mutex &vis_mtx) override;

 private:
  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace navigation
}  // namespace vtr
