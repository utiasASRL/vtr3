#pragma once

#include <mutex>

#include <vtr_vision/features/extractor/base_feature_extractor.hpp>

#include <vtr_logging/logging.hpp>
#include <asrl/vision/gpusurf/GpuSurfDetector.hpp>
#include <asrl/vision/gpusurf/GpuSurfStereoDetector.hpp>

namespace vtr {
namespace vision {

/////////////////////////////////////////////////////////////////////////
/// @class Feature extractor for the ASRL GPU Speeded Up Robust
/// Features
///        (GPU-SURF).
/// @details This class accepts greyscale images and computes SURF features
///          on either mono images or stereo pairs with left-right matching.
/////////////////////////////////////////////////////////////////////////
class GpuSurfFeatureExtractor : public BaseFeatureExtractor {
 public:
  /////////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /// @param SURF Stereo configuration
  /////////////////////////////////////////////////////////////////////////
  GpuSurfFeatureExtractor() = default;

  /////////////////////////////////////////////////////////////////////////
  /// @brief Destructor
  /////////////////////////////////////////////////////////////////////////
  virtual ~GpuSurfFeatureExtractor(){};

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the underlying GPUSURF engine.
  void initialize(asrl::GpuSurfConfiguration &config);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the underlying stereo GPUSURF engine.
  void initialize(asrl::GpuSurfStereoConfiguration &config);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Extracts a list of descriptors and keypoints from a single image.
  /// @param[in] image the input image.
  /// @return the extracted features (keypoints, descriptors, info)
  virtual Features extractFeatures(const cv::Mat &image);

  /////////////////////////////////////////////////////////////////////////
  /// Extracts a list of descriptors and keypoints from a set of
  ///   two rectified stereo images.
  /// @param[in] left the left image.
  /// @param[in] right the right image.
  /// @return the extracted features that have been pruned and match-aligned.
  virtual ChannelFeatures extractStereoFeatures(
      const cv::Mat &left, const cv::Mat &right);

 private:
  Features SURFToFrame(
      const std::vector<asrl::Keypoint> &keypoints,
      const std::vector<float> &descriptors);

#if 0
  /////////////////////////////////////////////////////////////////////////
  /// @brief Computes the keypoints and descriptors
  /// @details In stereo mode, this function will produce a frame
  /// @param[in] The collection of input images.
  /// @param[in,out] keypoints the collection of keypoints to be populated
  /// @param[in,out] descriptors the collection of descriptors to be populated
  void computeStereoFeatures(
      std::vector<cv::Mat> &images,
      std::vector<std::vector<asrl::Keypoint>> &keypoints,
      std::vector<float> &descriptors);
#endif

  /// The SURF configuration.
  asrl::GpuSurfConfiguration config_;

  /// The SURF configuration.
  asrl::GpuSurfStereoConfiguration stereo_config_;

  /// The underlying GPU SURF detector stereo engine.
  std::unique_ptr<asrl::GpuSurfStereoDetector> stereo_detector_;

  /// The underlying GPU SURF detector engine.
  std::unique_ptr<asrl::GpuSurfDetector> detector_;

  /// The mutex that prevents multiple threads on the GPU at once.
  static std::mutex gpu_mutex_;
};

}  // namespace vision
}  // namespace vtr_vision
