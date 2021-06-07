#pragma once

#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/visualize.hpp>
#include <vtr_vision/features/extractor/base_feature_extractor.hpp>
#include <vtr_vision/features/extractor/extractor_configs.hpp>

namespace vtr {
namespace tactic {
namespace stereo {

/**
 * \brief A module that converts images from RGB to grayscale or other forms,
 * and extracts features using surf/orb in parallel.
 * \details
 * requires: qdata.[rig_images]
 * outputs: qdata.[rig_images, rig_features]
 *
 * This module stores the converted images (gray scaled, color constant) in
 * extra channels of each image of qdata.rig_images.
 * The features corresponding to each channel are stored in qdata.rig_features.
 * Only stereo matched features are stored.
 */
class ConversionExtractionModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "conversion_extraction";

  /** \brief Collection of config parameters */
  struct Config {
    std::string feature_type = "ASRL_GPU_SURF";

    vision::ORBConfiguration opencv_orb_params;
#if GPUSURF_ENABLED
    asrl::GpuSurfConfiguration gpu_surf_params;
    asrl::GpuSurfStereoConfiguration gpu_surf_stereo_params;
#endif

    /** \brief The collection of user requested image conversions. */
    std::vector<std::string> conversions;

    /**
     * \brief The collection of color constant weights.
     * \details The size of this vector must be greater or equal to the number
     * of requested color constant conversions.
     */
    std::vector<double> color_constant_weights;

    /** \brief Histogram equalization flag for color constant transformation. */
    bool color_constant_histogram_equalization;

    /** \brief Flag for displaying the raw detected features */
    bool visualize_raw_features = false;
  };

  ConversionExtractionModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  /**
   * \brief Uses multi-threading to perform image conversion (i.e. RGB to
   * grayscale and CC),and feature extraction in parallel for each rig, channel
   * and camera.
   */
  void runImpl(QueryCache &qdata, MapCache &, const Graph::ConstPtr &) override;

  /**
   * \brief Visualizes raw features that were extracted on all images and their
   * conversions.
   */
  void visualizeImpl(QueryCache &qdata, MapCache &,
                     const std::shared_ptr<const Graph> &,
                     std::mutex &vis_mtx) override;

  void createExtractor();

  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;

  /** \brief Feature Extractor */
  std::shared_ptr<vision::BaseFeatureExtractor> extractor_;
};

}  // namespace stereo
}  // namespace tactic
}  // namespace vtr