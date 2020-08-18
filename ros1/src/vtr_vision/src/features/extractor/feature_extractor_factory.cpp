#include <vtr/vision/features/extractor/feature_extractor_factory.h>

namespace vtr {
namespace vision {

/**
 * \brief Creates a feature extractor based on the input string.
 * \param[in] The input_string describing the feature extractor type
 * \return a shared pointer to the instantiated feature extractor
 * \throws std::runtime_error if the type is not found.
 */
std::shared_ptr<BaseFeatureExtractor> FeatureExtractorFactory::createExtractor(
    const std::string &type) {
  std::shared_ptr<BaseFeatureExtractor> extractor = nullptr;

  if (type.compare("OPENCV_ORB") == 0) {
    // CPU Based Feature Extractors
    extractor.reset(new vtr::vision::OrbFeatureExtractor());
  } else if (type.compare("ASRL_GPU_SURF") == 0) {
    // CUDA Based Feature Extractors
#if GPUSURF_ENABLED
    extractor.reset(new vtr::vision::GpuSurfFeatureExtractor());
#else
    std::string err_str =
        "Attempted to make extractor of type " + type +
        " when CUDA or GPUSURF isn't enabled." +
        "\nMake sure that CUDA and GPUSURF are installed on your system";
    LOG(ERROR) << err_str;
#endif
  } else {
    std::string err_str = "Could not find extractor of type " + type;
    LOG(ERROR) << err_str;
    throw std::runtime_error(err_str);
  }

  return extractor;
}

}  // namespace vision
}  // namespace vtr
