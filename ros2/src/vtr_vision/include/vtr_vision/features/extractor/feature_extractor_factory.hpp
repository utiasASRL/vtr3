#pragma once

#include <vtr_vision/features/extractor/base_feature_extractor.hpp>

#include <vtr_vision/features/extractor/extractor_configs.hpp>
#include <vtr_vision/features/extractor/orb_feature_extractor.hpp>
#if GPUSURF_ENABLED
#include <vtr_vision/features/extractor/cuda/gpu_surf_feature_extractor.hpp>
#endif

namespace vtr {
namespace vision {
/** \brief Creates a feature extractor based on the input string.
 */
class FeatureExtractorFactory {
 public:
  /**
   * \brief Creates a feature extractor based on the input string.
   * \param[in] type The input_string describing the feature extractor type
   * \return a shared pointer to the instantiated feature extractor
   * \throws std::runtime_error if the type is not found.
   */
  static std::shared_ptr<BaseFeatureExtractor> createExtractor(
      const std::string &type);

 private:
};

}  // namespace vision
}  // namespace vtr_vision
