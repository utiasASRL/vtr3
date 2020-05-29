#pragma once

#include <asrl/navigation/modules/base_module.h>

#include <asrl/vision/features/extractor/BaseFeatureExtractor.hpp>
#include <asrl/vision/features/extractor/ExtractorConfigs.hpp>
// #include <asrl/navigation/Visualize.hpp>

namespace asrl {
namespace navigation {

/** \brief Convert images from RGB to grayscale and extract features in
 * parallel.
 */
class ConversionExtractionModule : public BaseModule {
 public:
  static constexpr auto type_str_ = "conversion_extraction";
  struct Config {
    // Extractor Configuration
    std::string feature_type;
    asrl::vision::ORBConfiguration opencv_orb_params;
#if GPUSURF_ENABLED
    asrl::GpuSurfConfiguration gpu_surf_params;
    asrl::GpuSurfStereoConfiguration gpu_surf_stereo_params;
#endif
    /// @brief The collection of user requested image conversions.
    std::vector<std::string> conversions;

    /// @brief The collection of color constant weights.
    /// @details The size of this vector must be greater or equal to the
    ///          number of requested color constant conversions.
    std::vector<double> color_constant_weights;

    /// @brief Histogram equalization flag for color constant transformations.
    bool color_constant_histogram_equalization;

    /// @brief display the raw detected features
    bool visualize_raw_features;
  };

  /// @brief TODO Construct with settings...
  ConversionExtractionModule() {}

  /// @brief Given two frames and matches detects the inliers that fit
  ///        the given model, and provides an initial guess at transform T_q_m.
  virtual void run(QueryCache &qdata, MapCache &,
                   const std::shared_ptr<const Graph> &);

  /// @brief Update the graph with Extracted features
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId){};
#if 0
  void setConfig(std::shared_ptr<Config> &config);
#endif

 protected:
#if 0
  /** \brief Visualization implementation
  */
  virtual void visualizeImpl(QueryCache &qdata, MapCache &,
                             const std::shared_ptr<const Graph> &,
                             std::mutex &vis_mtx);
#endif
 private:
  /// @brief Algorithm Configuration
  std::shared_ptr<Config> config_;

  /// @brief Feature Extractor
  std::shared_ptr<vision::BaseFeatureExtractor> extractor_;
};

}  // namespace navigation
}  // namespace asrl
