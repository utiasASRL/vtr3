#pragma once

#include <memory>
#include <vector>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 2
#include <opencv2/features2d/features2d.hpp>
#elif CV_MAJOR_VERSION >= 3  // vtr3 change: opencv 4+
#include <opencv2/xfeatures2d.hpp>
#if defined(HAVE_OPENCV_CUDAFEATURES2D)
#include <opencv2/cudafeatures2d.hpp>
#endif
#endif
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vtr/vision/features/extractor/base_feature_extractor.h>
#include <vtr/vision/features/extractor/orb_configuration.h>

typedef cv::xfeatures2d::StarDetector StarDetector;
typedef cv::UMat uMat;

namespace vtr {
namespace vision {

/////////////////////////////////////////////////////////////////////////
/// @class Feature extractor for the OpenCV ORB Feature Extractor.
/// @details This class accepts greyscale images and computes ORB features
///          on either mono images or stereo pairs with left-right matching.
/////////////////////////////////////////////////////////////////////////
class OrbFeatureExtractor : public BaseFeatureExtractor {
 public:
  /////////////////////////////////////////////////////////////////////////
  // @param ORB configuration
  OrbFeatureExtractor(){};

  /////////////////////////////////////////////////////////////////////////
  virtual ~OrbFeatureExtractor(){};

  /////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the underlying ORB engine.
  void initialize(ORBConfiguration& config);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Extracts a list of descriptors and keypoints from a single image.
  /// @param[in] The input image.
  /// @return A Frame consisting of a list of keypoints and a list of
  /// descriptors, corresponding to the keypoints.
  virtual asrl::vision::Features extractFeatures(const cv::Mat& image);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Extracts a list of descriptors and keypoints from a set of
  ///        two rectified stereo images.
  /// @param[in] The collection of input images.
  /// @return A StereoFrame consisting of two Frames and a list of
  /// correspondences as a MatchList
  virtual asrl::vision::ChannelFeatures extractStereoFeatures(
      const cv::Mat& left, const cv::Mat& right);

 private:
  /////////////////////////////////////////////////////////////////////////
  /// @brief Detect features on an image pyramid using STAR
  void detectOnPyramid(const uMat& image, asrl::vision::Keypoints& keypoints,
                       const cv::Mat& mask);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Compute the orientation independently if using STAR using the ORB
  /// moments method:
  ///        https://gilscvblog.com/2013/10/04/a-tutorial-on-binary-descriptors-part-3-the-orb-descriptor/
  void computeAngles(const uMat& image, asrl::vision::Keypoints& keypoints);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Detect features on an image using the default ORB Harris/FAST
  /// detector
  void detectWithORB(const uMat& image, asrl::vision::Keypoints& keypoints,
                     const cv::Mat& mask);

  /////////////////////////////////////////////////////////////////////////
  /// @brief Bin keypoints
  void binKeypoints(cv::Size size, asrl::vision::Keypoints& keypoints);

  /////////////////////////////////////////////////////////////////////////
  /// @brief The ORB configuration.
  /////////////////////////////////////////////////////////////////////////
  ORBConfiguration config_;

#if CV_MAJOR_VERSION == 2
  cv::ORB* detector_;
  std::shared_ptr<StarDetector> stardetector_;
#elif CV_MAJOR_VERSION >= 3  // vtr3 change: opencv 4+
  cv::Ptr<cv::ORB> detector_;
  cv::Ptr<cv::xfeatures2d::StarDetector> stardetector_;
#if defined(HAVE_OPENCV_CUDAFEATURES2D) && CV_MINOR_VERSION > 1
  cv::Ptr<cv::cuda::ORB> cudadetector_;
#endif  // defined(HAVE_OPENCV_CUDAFEATURES2D)
#endif  // CV_MAJOR_VERSION
};

}  // namespace vision
}  // namespace vtr
