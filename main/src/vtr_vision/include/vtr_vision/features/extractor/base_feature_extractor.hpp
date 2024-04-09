// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file base_feature_extractor.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <future>
#include <queue>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <vtr_logging/logging.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

/**
 * The base class for the visual feature extractors.
 * The extractor runs in parallel when processing multiple images in a request.
 * This behaviour can be disabled by derived classes by locking a static mutex.
 * You can use vtr_vision::common::thread_pool for asynchronous requests
 */
class BaseFeatureExtractor {
  using My_t = BaseFeatureExtractor;

 public:
  BaseFeatureExtractor() = default;
  virtual ~BaseFeatureExtractor() = default;

  /** \brief Extracts features from a single opencv image.
   */
  virtual Features extractFeatures(const cv::Mat &image) = 0;

  /** \brief Extracts features from a stereo opencv image pair.
   */
  virtual ChannelFeatures extractStereoFeatures(const cv::Mat &left,
                                                const cv::Mat &right) = 0;

  /** \brief Extracts features from a stereo opencv image pair.
   */
  virtual ChannelFeatures extractStereoFeaturesDisp(
      const cv::Mat &left, const cv::Mat &disp) = 0;

  /** \brief Extracts features from a stereo opencv image pair.
   */
  virtual ChannelFeatures extractStereoFeaturesDispExtra(
      const cv::Mat &left, const cv::Mat &disp, const cv::Mat &ekypoints, 
      const cv::Mat &descriptors, const cv::Mat &scores) = 0;

  /** \brief Extracts features from a stereo opencv image pair.
   */
  virtual ChannelExtra extractFeaturesExtra(const cv::Mat &left) = 0;

  /** \brief Extracts features from a single vtr_vision image.
   */
  Features extractFeatures(const Image &image);

  /** \brief Extracts features from a stereo vtr_vision image pair.
   */
  ChannelFeatures extractStereoFeatures(const Image &left, const Image &right);

  /** \brief Extracts features from a stereo vtr_vision image pair.
   */
  ChannelFeatures extractStereoFeaturesDisp(
      const Image &left, const Image &right, const Image &disp);

  /** \brief Extracts features from a stereo vtr_vision image pair.
   */
  ChannelFeatures extractStereoFeaturesDispExtra(
      const Image &left, const Image &right, const Image &disp, 
      const Extra &extra);

  /** \brief Extracts features from a stereo vtr_vision image pair.
   */
  ChannelExtra extractFeaturesExtra(const Image &left);

   /** \brief Extracts features from an vtr_vision stereo rig channel image
   * (2-camera).
   */
  ChannelFeatures extractStereoFeatures(const ChannelImages &channel);

  /** \brief Extracts features from an vtr_vision stereo rig channel image (2-camera).
   */
  ChannelFeatures extractStereoFeaturesDisp(
      const ChannelImages &channel, const ChannelImages &channel_disp);

  /** \brief Extracts features from an vtr_vision stereo rig channel image (2-camera).
   */
  ChannelFeatures extractStereoFeaturesDispExtra(
      const ChannelImages &channel, const ChannelImages &channel_disp,
      const ChannelExtra &channel_extra);

  ChannelExtra extractFeaturesExtra(const ChannelImages &channel);

  /**
   * \brief Extracts features from an vtr_vision rig channel image
   * (multi-camera). \param[in] channel \todo \param[in] fully_matched will
   * remove all non-stereo matching features, aligning the features so matches
   * share indices (2 cams only).
   */
  ChannelFeatures extractChannelFeatures(const ChannelImages &channel,
                                         bool fully_matched);

  /** 
   * \brief Extracts features from an vtr_vision rig channel image (multi-camera).
   * \param[in] channel \todo
   * \param[in] fully_matched will remove all non-stereo matching features,
   *            aligning the features so matches share indices (2 cams only).
   */
  ChannelFeatures extractChannelFeaturesDisp(
      const ChannelImages &channel, 
      const ChannelImages &channel_disp, 
      bool fully_matched);

  /** 
   * \brief Extracts features from an vtr_vision rig channel image (multi-camera).
   * \param[in] channel \todo
   * \param[in] fully_matched will remove all non-stereo matching features,
   *            aligning the features so matches share indices (2 cams only).
   */
  ChannelFeatures extractChannelFeaturesDispExtra(
      const ChannelImages &channel, 
      const ChannelImages &channel_disp,
      const ChannelExtra &channel_extra, 
      bool fully_matched);

  /** 
   * \brief Extracts features from an vtr_vision rig channel image (multi-camera).
   * \param[in] channel \todo
   * \param[in] fully_matched will remove all non-stereo matching features,
   *            aligning the features so matches share indices (2 cams only).
   */
  ChannelExtra extractChannelFeaturesExtra(const ChannelImages &channel);

  /**
   * \brief Extracts features from an vtr_vision rig image (multi-channel,
   * multi-camera).
   * \param[in] rig \todo
   * \param[in] fully_matched will remove all non-stereo matching features,
   * aligning the features so matches share indices (2 cams only).
   */
  RigFeatures extractRigFeatures(const RigImages &rig, bool fully_matched);

  void setRigCalibration(const RigCalibration &calib) { calib_ = calib; }

  // the rig calibration to use for guided matching in the stereo case
  RigCalibration calib_;
};

}  // namespace vision
}  // namespace vtr
