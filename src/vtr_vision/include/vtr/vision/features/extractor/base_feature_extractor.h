#pragma once

#include <future>
#include <queue>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <asrl/common/logging.hpp>
#include <asrl/vision/Types.hpp>

namespace vtr {
namespace vision {

/**
 * The base class for the visual feature extractors.
 * The extractor runs in parallel when processing multiple images in a request.
 * This behaviour can be disabled by derived classes by locking a static mutex.
 * You can use asrl::common::thread_pool for asynchronous requests
 */
class BaseFeatureExtractor {
  using My_t = BaseFeatureExtractor;

 public:
  BaseFeatureExtractor() = default;
  virtual ~BaseFeatureExtractor() {}

  /** \brief Extracts features from a single opencv image.
   */
  virtual asrl::vision::Features extractFeatures(const cv::Mat &image) = 0;

  /** \brief Extracts features from a stereo opencv image pair.
   */
  virtual asrl::vision::ChannelFeatures extractStereoFeatures(
      const cv::Mat &left, const cv::Mat &right) = 0;

  /** \brief Extracts features from a single vtr image.
   */
  asrl::vision::Features extractFeatures(const asrl::vision::Image &image);

  /** \brief Extracts features from a stereo vtr image pair.
   */
  asrl::vision::ChannelFeatures extractStereoFeatures(
      const asrl::vision::Image &left, const asrl::vision::Image &right);

  /** \brief Extracts features from an vtr stereo rig channel image (2-camera).
   */
  asrl::vision::ChannelFeatures extractStereoFeatures(
      const asrl::vision::ChannelImages &channel);

  /** \brief Extracts features from an vtr rig channel image (multi-camera).
   *
   * \param[in] fully_matched will remove all non-stereo matching features,
   *            aligning the features so matches share indices (2 cams only).
   */
  asrl::vision::ChannelFeatures extractChannelFeatures(
      const asrl::vision::ChannelImages &channel, bool fully_matched);

  /** \brief Extracts features from an vtr rig image (multi-channel,
   * multi-camera).
   *
   * \param[in] fully_matched will remove all non-stereo matching features,
   * aligning the features so matches share indices (2 cams only).
   */
  asrl::vision::RigFeatures extractRigFeatures(
      const asrl::vision::RigImages &rig, bool fully_matched);

  void setRigCalibration(const asrl::vision::RigCalibration &calib) {
    calib_ = calib;
  }

  // the rig calibration to use for guided matching in the stereo case
  asrl::vision::RigCalibration calib_;
};

}  // namespace vision
}  // namespace vtr
