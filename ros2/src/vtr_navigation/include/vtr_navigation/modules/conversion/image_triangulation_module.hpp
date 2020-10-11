#pragma once

#include <vtr_navigation/modules/base_module.hpp>

namespace vtr {
namespace navigation {

/**
 * \brief A module that generates landmarks from image features. The landmark
 * point is 3D for stereo camera.
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

  ImageTriangulationModule(std::string name = type_str_) : BaseModule{name} {}

  /**
   * \brief Generates landmarks from image features. The landmark point is 3D
   * for stereo camera.
   */
  void run(QueryCache &qdata, MapCache &,
           const std::shared_ptr<const Graph> &) override;

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 protected:
  /** \brief Visualizes features and stereo features. */
  void visualizeImpl(QueryCache &qdata, MapCache &,
                     const std::shared_ptr<const Graph> &,
                     std::mutex &vis_mtx) override;

 private:
  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace navigation
}  // namespace vtr
