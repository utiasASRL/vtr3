#pragma once

#include <mutex>


#include <vtr_vision/features/extractor/base_feature_extractor.hpp>

#ifdef VTR_VISION_LEARNED
#include <torch/script.h> 
#include <torch/torch.h>
#include <vtr_vision/features/extractor/learned_feature_configuration.hpp>
#endif

#include <vtr_logging/logging.hpp>

namespace vtr {
namespace vision {

/////////////////////////////////////////////////////////////////////////
/// @class LearnedFeatureExtractor
/// @brief Feature extractor for the Learned Features.
/// @details This class accepts RGB images and computes learned features
///          on either mono images or stereo pairs with left-right matching.
/////////////////////////////////////////////////////////////////////////
class LearnedFeatureExtractor : public BaseFeatureExtractor {
 public:
  /////////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /// @param Learned Feature Stereo configuration
  /////////////////////////////////////////////////////////////////////////
  LearnedFeatureExtractor() = default;

  /////////////////////////////////////////////////////////////////////////
  /// @brief Destructor
  /////////////////////////////////////////////////////////////////////////
  virtual ~LearnedFeatureExtractor(){};

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the underlying Learned Feature engine.
  void initialize(LearnedFeatureConfiguration &config);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the underlying stereo Learned Feature engine.
  void initialize(LearnedFeatureStereoConfiguration &config);

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
  virtual ChannelFeatures extractStereoFeatures(const cv::Mat &left,
                                                const cv::Mat &right);

  /////////////////////////////////////////////////////////////////////////
  /// Extracts a list of descriptors and keypoints from a set of
  ///   two rectified stereo images.
  /// @param[in] left the left image.
  /// @param[in] right the right image.
  /// @return the extracted features that have been pruned and match-aligned.
  virtual ChannelFeatures extractStereoFeaturesDisp(const cv::Mat &left,
                                                    const cv::Mat &disp);

  /////////////////////////////////////////////////////////////////////////
  /// Extracts a list of descriptors and keypoints from a set of
  ///   two rectified stereo images.
  /// @param[in] left the left image.
  /// @param[in] right the right image.
  /// @return the extracted features that have been pruned and match-aligned.
  virtual ChannelFeatures extractStereoFeaturesDispExtra(const cv::Mat &left,
                                                         const cv::Mat &disp,
                                                         const cv::Mat 
                                                         &keypoints,
                                                         const cv::Mat 
                                                         &descriptors,
                                                         const cv::Mat 
                                                         &scores);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Extracts a list of descriptors and keypoints from a single image.
  /// @param[in] image the input image.
  /// @return the extracted features (keypoints, descriptors, info)
  virtual ChannelExtra extractFeaturesExtra(const cv::Mat &image);


 private:
  Features learnedFeatureToFrame(const torch::Tensor &keypoints,
                                 const torch::Tensor &point_descriptors,
                                 const torch::Tensor &point_scores);

  ChannelFeatures learnedFeaturesToStereoKeypoints(
                                        const torch::Tensor &keypoints, 
                                        const torch::Tensor &point_descriptors,
                                        const torch::Tensor &point_scores,
                                        const torch::Tensor &point_disparities);

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> 
          extractLearnedFeaturesSparse(const cv::Mat &image);

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> 
          extractLearnedFeaturesDense(const cv::Mat &image);

  torch::Tensor getDisparity(const cv::Mat& left, const cv::Mat& right, 
                             const LearnedFeatureStereoConfiguration config);

  torch::Tensor getDisparityTensor(const cv::Mat& disp);


  /// The Learned Feature configuration.
  LearnedFeatureConfiguration config_;

  /// The Learned Feature stereo configuration.
  LearnedFeatureStereoConfiguration stereo_config_;

  /// The underlying Learned Feature detector engine.
  torch::jit::script::Module detector_;

  /// The mutex that prevents multiple threads on the GPU at once.
  static std::mutex gpu_mutex_;
};

}  // namespace vision
}  // namespace vtr
